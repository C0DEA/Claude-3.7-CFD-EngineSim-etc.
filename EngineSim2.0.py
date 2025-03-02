import numpy as np
import pygame
import math
import time
from pygame.locals import *
import threading
import os
from pathlib import Path

# Constants for the VR38DETT V6 engine from Nissan GTR
ENGINE_DISPLACEMENT = 3.8  # Liters
CYLINDERS = 6
CYLINDER_ANGLE_V6 = 60  # Degrees between cylinder banks
FIRING_ORDER = [1, 4, 3, 6, 2, 5]  # V6 firing order
IDLE_RPM = 750
MAX_RPM = 7100
REDLINE_RPM = 6800

# Torque curve for VR38DETT (approximate values)
RPM_POINTS = [1000, 2000, 3000, 4000, 5000, 6000, 7000]
TORQUE_POINTS = [350, 450, 520, 570, 580, 560, 490]  # Nm

# Create interpolation function for torque
try:
    from scipy.interpolate import interp1d
    torque_function = interp1d(RPM_POINTS, TORQUE_POINTS, kind='cubic', 
                              bounds_error=False, fill_value=(TORQUE_POINTS[0], TORQUE_POINTS[-1]))
except ImportError:
    # Fallback if scipy is not available
    def torque_function(rpm):
        # Simple linear interpolation
        for i in range(len(RPM_POINTS)-1):
            if RPM_POINTS[i] <= rpm <= RPM_POINTS[i+1]:
                t = (rpm - RPM_POINTS[i]) / (RPM_POINTS[i+1] - RPM_POINTS[i])
                return TORQUE_POINTS[i] + t * (TORQUE_POINTS[i+1] - TORQUE_POINTS[i])
        
        # Handle out of range values
        if rpm < RPM_POINTS[0]:
            return TORQUE_POINTS[0]
        return TORQUE_POINTS[-1]

# Display settings
WIDTH, HEIGHT = 1024, 768
PISTON_COLORS = [(220, 220, 220), (200, 200, 200), (180, 180, 180)]
ROD_COLORS = [(180, 180, 180), (160, 160, 160), (140, 140, 140)]
CRANKSHAFT_COLOR = (100, 100, 100)
BLOCK_COLOR = (80, 80, 80)
CYLINDER_COLOR = (60, 60, 60)

class Engine:
    def __init__(self):
        self.rpm = IDLE_RPM
        self.target_rpm = IDLE_RPM
        self.throttle = 0.0  # 0.0 to 1.0
        self.crank_angle = 0.0  # degrees
        self.piston_positions = [0.0] * CYLINDERS
        self.torque = 0.0
        self.horsepower = 0.0
        self.running = False  # Engine starts off
        self.starting = False
        self.start_time = 0
        
        # Engine geometry parameters
        self.stroke = 88.4  # mm
        self.bore = 95.5   # mm
        self.rod_length = 158.0  # mm
        self.compression_ratio = 9.0
        
        # Crank pin offsets for V6 - each pin is offset by specific angle
        self.crank_pin_angles = [0, 120, 240, 0, 120, 240]  # Degrees
        
        # Calculate firing events (when each cylinder fires in the cycle)
        self.firing_events = []
        
        for i, cylinder in enumerate(FIRING_ORDER):
            # Calculate at what angle in the 720° cycle each cylinder fires
            # For a 4-stroke engine, cylinders fire every 720°/num_cylinders
            firing_angle = (i * 720 / CYLINDERS) % 720
            self.firing_events.append((cylinder - 1, firing_angle))
            
        # Sort by firing angle
        self.firing_events.sort(key=lambda x: x[1])
        
    def start_engine(self):
        """Start the engine"""
        if not self.running:
            self.starting = True
            self.start_time = time.time()
            self.rpm = 300  # Cranking RPM
            
    def stop_engine(self):
        """Stop the engine"""
        self.running = False
        self.rpm = 0
        self.throttle = 0.0
        
    def calculate_piston_positions(self):
        """Calculate piston positions based on crank angle and V6 geometry"""
        for i in range(CYLINDERS):
            # Get the cylinder number (0-based)
            cylinder_idx = i
            
            # For V6 engine, determine which bank the cylinder is in
            if cylinder_idx % 2 == 0:  # Cylinders 0, 2, 4 (1, 3, 5 in 1-based indexing)
                bank_angle = -CYLINDER_ANGLE_V6 / 2  # Left bank
            else:  # Cylinders 1, 3, 5 (2, 4, 6 in 1-based indexing)
                bank_angle = CYLINDER_ANGLE_V6 / 2  # Right bank
                
            # Get crank pin angle for this cylinder
            crank_pin_angle = self.crank_pin_angles[cylinder_idx]
            
            # Calculate piston position based on crank angle, bank angle and crank pin angle
            # Each cylinder's crank rotation is offset by its crank pin angle
            effective_angle = (self.crank_angle + crank_pin_angle) % 720
            
            # Convert to radians
            effective_rad = math.radians(effective_angle)
            bank_rad = math.radians(bank_angle)
            
            # Calculate piston movement along its cylinder bore axis
            # Using the rod-crank formula modified for V-engine
            r = self.stroke / 2  # Crank radius (half of stroke)
            
            # Offset for connecting rod angle due to bank angle
            # This is key for V-engines - the connecting rod doesn't move straight up/down
            crank_x = r * math.sin(effective_rad)
            crank_y = r * math.cos(effective_rad)
            
            # Project the crank position onto the cylinder axis
            # For a V-engine, we need to account for the bank angle
            projected_crank = crank_x * math.sin(bank_rad) - crank_y * math.cos(bank_rad)
            
            # Calculate piston position along cylinder axis
            # Using approximate formula for piston position in V-engine
            rod_angle = math.asin(projected_crank / self.rod_length)
            piston_pos = projected_crank / math.sin(rod_angle) if rod_angle != 0 else self.rod_length
            
            # Normalize to 0-1 range where 0 is TDC (Top Dead Center) and 1 is BDC (Bottom Dead Center)
            piston_pos_normalized = (piston_pos + r) / (2 * r)
            piston_pos_normalized = max(0, min(1, piston_pos_normalized))  # Clamp to 0-1 range
            
            # Store position
            self.piston_positions[i] = piston_pos_normalized
            
    def update(self, dt):
        """Update engine state based on throttle and time step"""
        if self.starting:
            # Engine starting sequence
            time_since_start = time.time() - self.start_time
            
            if time_since_start < 1.0:
                # Cranking phase
                self.rpm = 300  # Cranking RPM
            elif time_since_start < 2.0:
                # Revving up phase
                self.rpm = 300 + (IDLE_RPM - 300) * (time_since_start - 1.0)
            else:
                # Start complete
                self.starting = False
                self.running = True
                self.rpm = IDLE_RPM
        
        if not self.running and not self.starting:
            # Engine is off
            self.rpm = max(0, self.rpm - 500 * dt)  # Engine slows down
            if self.rpm < 10:
                self.rpm = 0
            
            # Update crank angle at a slower rate when engine is turning off
            if self.rpm > 0:
                degrees_per_second = self.rpm / 60 * 360
                self.crank_angle = (self.crank_angle + degrees_per_second * dt) % 720
            
            self.torque = 0
            self.horsepower = 0
            return
            
        # Engine is running or starting
        # Update RPM based on throttle
        rpm_rate = 2000  # RPM change per second at full throttle
        
        if self.throttle > 0.1:
            # Accelerate based on throttle
            rpm_change = rpm_rate * self.throttle * dt
            self.target_rpm = IDLE_RPM + (MAX_RPM - IDLE_RPM) * self.throttle
        else:
            # Decelerate when throttle is low
            rpm_change = -rpm_rate * 0.5 * dt
            self.target_rpm = IDLE_RPM
            
        # Apply RPM limits
        self.target_rpm = max(IDLE_RPM, min(self.target_rpm, MAX_RPM))
        
        # Smoothly transition to target RPM
        rpm_diff = self.target_rpm - self.rpm
        self.rpm += rpm_diff * min(3.0 * dt, 1.0)
        
        # Ensure RPM stays within limits
        self.rpm = max(IDLE_RPM if self.running else 0, min(self.rpm, MAX_RPM))
        
        # Update crank angle based on RPM
        # RPM = revolutions per minute, so RPM/60 = revolutions per second
        # Each revolution is 360 degrees, so RPM/60*360 = degrees per second
        degrees_per_second = self.rpm / 60 * 360
        self.crank_angle = (self.crank_angle + degrees_per_second * dt) % 720  # Full 4-stroke cycle is 720 degrees
        
        # Calculate piston positions
        self.calculate_piston_positions()
        
        # Calculate torque and horsepower
        self.torque = float(torque_function(self.rpm))
        # HP = Torque * RPM / 5252
        self.horsepower = self.torque * self.rpm / 5252
        
        # Adjust for throttle
        throttle_factor = 0.2 + 0.8 * self.throttle
        self.torque *= throttle_factor
        self.horsepower *= throttle_factor


class RealisticEngineSoundManager:
    def __init__(self, engine):
        self.engine = engine
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
        
        # Create multiple mixer channels for overlapping sounds
        self.channels = [pygame.mixer.Channel(i) for i in range(CYLINDERS + 2)]  # +2 for start sound and idle
        
        # Attempt to load the explosion sound
        try:
            self.explosion_sound = pygame.mixer.Sound('sound.wav')
        except:
            # Create a fallback sound if file not found
            print("Warning: sound.wav not found, using synthesized sound")
            self.explosion_sound = self.create_fallback_explosion_sound()
        
        # Create starting sound
        self.start_sound = self.create_engine_start_sound()
        
        # Track firing events
        self.last_firing_angle = -1000  # Large negative number to ensure first firing
        self.next_firing_idx = 0
        
        # Idle sound for when engine is just running without any cylinder explosions
        self.idle_sound = self.create_idle_sound()
        self.idle_channel = self.channels[CYLINDERS]
        self.idle_playing = False
        
        # Starting sound channel
        self.start_channel = self.channels[CYLINDERS + 1]
        
    def create_fallback_explosion_sound(self):
        """Create a synthesized explosion sound if the WAV file is not found"""
        sample_rate = 44100
        duration = 0.2  # seconds
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate a percussive sound
        t = np.linspace(0, duration, len(buffer), False)
        
        # Create attack with high frequency energy, then decay
        for i, time in enumerate(t):
            if time < 0.02:  # Sharp attack
                buffer[i] = np.random.normal(0, 1) * (time / 0.02)
            else:
                # Exponential decay with some randomness
                decay = np.exp(-(time - 0.02) * 20)
                buffer[i] = np.random.normal(0, 0.5) * decay
        
        # Add some low frequency content
        buffer += 0.7 * np.sin(2 * np.pi * 80 * t) * np.exp(-t * 10)
        
        # Normalize
        buffer = 0.8 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
        
    def create_engine_start_sound(self):
        """Create engine starting sound"""
        sample_rate = 44100
        duration = 2.0  # seconds
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate a starting sound - initial cranking followed by revving
        t = np.linspace(0, duration, len(buffer), False)
        
        # Initial cranking phase
        cranking_freq = 15  # Hz - slow cranking
        for i, time in enumerate(t):
            if time < 1.0:  # Cranking phase
                # Slow cranking motor
                buffer[i] = 0.5 * np.sin(2 * np.pi * cranking_freq * time)
                # Add some mechanical noise
                buffer[i] += 0.3 * np.random.normal(0, 0.3)
                # Motor sound increases
                buffer[i] *= (0.3 + 0.7 * time)
            else:
                # Revving up phase
                rev_factor = (time - 1.0)
                rev_freq = 50 + 150 * rev_factor
                buffer[i] = 0.7 * np.sin(2 * np.pi * rev_freq * time)
                # Add higher frequencies
                buffer[i] += 0.4 * np.sin(2 * np.pi * rev_freq * 2 * time)
                buffer[i] += 0.2 * np.sin(2 * np.pi * rev_freq * 3 * time)
                # Add some noise
                buffer[i] += 0.2 * np.random.normal(0, 0.5)
        
        # Apply volume envelope
        envelope = np.ones_like(buffer)
        fade_out = 0.1  # seconds
        fade_samples = int(fade_out * sample_rate)
        envelope[-fade_samples:] = np.linspace(1, 0, fade_samples)
        buffer = buffer * envelope
        
        # Normalize
        buffer = 0.9 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
        
    def create_idle_sound(self):
        """Create an idle engine sound"""
        sample_rate = 44100
        duration = 1.0  # seconds
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate an idle sound - low rumble
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base frequency for idle
        idle_freq = 30  # Hz 
        
        # Create a complex waveform with harmonics for idle sound
        for i, time in enumerate(t):
            # Base rumble
            buffer[i] = 0.5 * np.sin(2 * np.pi * idle_freq * time)
            # Add harmonics
            buffer[i] += 0.3 * np.sin(2 * np.pi * idle_freq * 2 * time)
            buffer[i] += 0.15 * np.sin(2 * np.pi * idle_freq * 3 * time)
            buffer[i] += 0.1 * np.sin(2 * np.pi * idle_freq * 4 * time)
            # Add some noise for texture
            buffer[i] += 0.1 * np.random.normal(0, 0.3)
        
        # Normalize
        buffer = 0.5 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
        
    def play_start_sound(self):
        """Play the engine start sound"""
        self.start_channel.play(self.start_sound)
        
    def update(self):
        """Update sound based on engine state"""
        # Handle engine starting/stopping
        if self.engine.starting and not self.start_channel.get_busy():
            self.play_start_sound()
            
        # If engine is not running or starting, stop all sounds
        if not self.engine.running and not self.engine.starting:
            for channel in self.channels:
                if channel.get_busy():
                    channel.fadeout(500)  # Fade out over 500ms
            return
            
        # Play idle sound if not already playing
        if self.engine.running and not self.idle_playing:
            self.idle_channel.play(self.idle_sound, loops=-1)
            self.idle_playing = True
        
        # Adjust idle sound volume and pitch based on RPM
        if self.idle_playing:
            # Volume decreases as RPM increases (engine noise takes over)
            idle_volume = max(0.1, 0.5 - self.engine.rpm / MAX_RPM * 0.4)
            # Pitch increases with RPM
            idle_pitch = 1.0 + (self.engine.rpm - IDLE_RPM) / (MAX_RPM - IDLE_RPM)
            
            self.idle_channel.set_volume(idle_volume)
            
        # Play cylinder explosion sounds based on firing events
        # Each cylinder fires once per 720 degrees (4-stroke cycle)
        current_angle = self.engine.crank_angle
        
        # Check if we've passed the next firing event
        if len(self.engine.firing_events) > 0:
            next_firing = self.engine.firing_events[self.next_firing_idx]
            next_cyl, next_angle = next_firing
            
            # Check if we've passed this firing angle
            if (current_angle > next_angle and self.last_firing_angle < next_angle) or \
               (current_angle < self.last_firing_angle and next_angle > self.last_firing_angle and next_angle < current_angle + 720):
                
                # Calculate pitch based on RPM
                pitch = 0.5 + 1.5 * self.engine.rpm / MAX_RPM
                
                # Volume based on throttle
                volume = 0.3 + 0.7 * self.engine.throttle
                
                # Play the explosion sound on this cylinder's channel
                channel = self.channels[next_cyl]
                channel.stop()  # Stop any previous sound
                channel.play(self.explosion_sound)
                channel.set_volume(volume)
                
                # Move to next firing event
                self.next_firing_idx = (self.next_firing_idx + 1) % len(self.engine.firing_events)
                
            self.last_firing_angle = current_angle


class EngineSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("V6 Engine Simulator - Nissan GTR VR38DETT")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 20)
        self.title_font = pygame.font.SysFont('Arial', 24, bold=True)
        
        # Create engine
        self.engine = Engine()
        
        # Engine sound manager
        self.sound_manager = RealisticEngineSoundManager(self.engine)
        
        # Engine rendering parameters
        self.center_x = WIDTH // 2
        self.center_y = HEIGHT // 2 - 70
        self.piston_width = 60
        self.piston_height = 40
        self.rod_width = 15
        self.rod_length = 100
        self.crank_radius = 40
        self.v_angle = CYLINDER_ANGLE_V6
        
        # Crankshaft visualization parameters
        self.crank_thickness = 20
        self.crank_length = 300
        self.crank_pin_radius = 15
        
        # Cylinder dimensions
        self.cylinder_width = 70
        self.cylinder_height = 180
        self.cylinder_spacing = 90
        
        # Initialize background surface for static parts
        self.background = pygame.Surface((WIDTH, HEIGHT))
        self.background.fill((0, 0, 0))
        self.draw_static_elements()
        
        # Throttle UI - make it larger and more accessible
        self.throttle_rect = pygame.Rect(WIDTH - 80, HEIGHT // 2 - 150, 40, 300)
        self.throttle_handle_rect = pygame.Rect(WIDTH - 90, HEIGHT // 2, 60, 30)
        self.dragging_throttle = False
        
        # Start/Stop button
        self.start_button_rect = pygame.Rect(WIDTH - 120, HEIGHT // 2 + 200, 100, 40)
        
        # Engine thread
        self.engine_thread = threading.Thread(target=self.engine_loop)
        self.engine_thread.daemon = True
        self.engine_thread.start()
        
    def draw_static_elements(self):
        """Draw static elements on the background surface"""
        # Engine block outline
        block_width = 450
        block_height = 350
        self.block_rect = pygame.Rect(
            self.center_x - block_width // 2,
            self.center_y - block_height // 2,
            block_width,
            block_height
        )
        pygame.draw.rect(self.background, BLOCK_COLOR, self.block_rect)
        pygame.draw.rect(self.background, (100, 100, 100), self.block_rect, 2)
        
        # Engine title
        title_surface = self.title_font.render("Nissan GTR VR38DETT V6 Engine", True, (220, 220, 220))
        self.background.blit(title_surface, (self.center_x - title_surface.get_width() // 2, 20))
        
        # Engine specs
        specs = [
            f"Displacement: {ENGINE_DISPLACEMENT}L",
            f"Cylinders: {CYLINDERS}",
            f"V-Angle: {CYLINDER_ANGLE_V6}°",
            f"Max RPM: {MAX_RPM}",
            f"Redline: {REDLINE_RPM}"
        ]
        
        y_pos = 60
        for spec in specs:
            spec_surface = self.font.render(spec, True, (200, 200, 200))
            self.background.blit(spec_surface, (20, y_pos))
            y_pos += 25
        
        # Throttle label (moved to be more visible)
        throttle_label = self.font.render("Throttle", True, (220, 220, 220))
        self.background.blit(throttle_label, (WIDTH - 100, HEIGHT // 2 - 180))
        
        # Draw throttle instructions
        instructions = [
            "Controls:",
            "- Drag throttle handle",
            "- Up/Down arrow keys",
            "- Click anywhere on throttle"
        ]
        
        y_pos = HEIGHT // 2 + 170
        for instruction in instructions:
            inst_surface = self.font.render(instruction, True, (180, 180, 180))
            self.background.blit(inst_surface, (WIDTH - 200, y_pos))
            y_pos += 25
    
    def draw_engine(self):
        """Draw the engine with all moving parts"""
        # Start with a copy of the background
        self.screen.blit(self.background, (0, 0))
        
        # Draw V6 engine with crankshaft
        crank_x = self.center_x
        crank_y = self.center_y + 120
        
        # Draw crankshaft main journal
        self.draw_crankshaft(crank_x, crank_y)
        
        # Draw cylinders in V configuration
        self.draw_cylinders_and_pistons(crank_x, crank_y)
        
        # Draw RPM gauge
        self.draw_gauge(150, HEIGHT - 100, 100, "RPM", self.engine.rpm, 0, MAX_RPM)
        
        # Draw torque gauge
        self.draw_gauge(350, HEIGHT - 100, 100, "Torque (Nm)", self.engine.torque, 0, 600)
        
        # Draw HP gauge
        self.draw_gauge(550, HEIGHT - 100, 100, "Horsepower", self.engine.horsepower, 0, 600)
        
        # Draw throttle control
        self.draw_throttle()
        
        # Draw Start/Stop button
        self.draw_start_button()
        
        # Draw RPM text
        rpm_text = self.font.render(f"{int(self.engine.rpm)} RPM", True, (255, 255, 255))
        self.screen.blit(rpm_text, (150 - rpm_text.get_width() // 2, HEIGHT - 160))
        
        # Draw torque text
        torque_text = self.font.render(f"{int(self.engine.torque)} Nm", True, (255, 255, 255))
        self.screen.blit(torque_text, (350 - torque_text.get_width() // 2, HEIGHT - 160))
        
        # Draw HP text
        hp_text = self.font.render(f"{int(self.engine.horsepower)} HP", True, (255, 255, 255))
        self.screen.blit(hp_text, (550 - hp_text.get_width() // 2, HEIGHT - 160))
        
        # Draw engine state
        state_text = "RUNNING" if self.engine.running else "STARTING" if self.engine.starting else "OFF"
        state_color = (0, 255, 0) if self.engine.running else (255, 255, 0) if self.engine.starting else (255, 0, 0)
        state_surface = self.font.render(f"Engine: {state_text}", True, state_color)
        self.screen.blit(state_surface, (20, HEIGHT - 50))
    
    def draw_crankshaft(self, crank_x, crank_y):
        """Draw the crankshaft with proper rotation and crank pins"""
        # Main bearing
        pygame.draw.circle(self.screen, CRANKSHAFT_COLOR, (crank_x, crank_y), 25)
        
        # Calculate crank angle
        crank_angle_rad = math.radians(self.engine.crank_angle % 360)
        
        # Draw crankshaft centerline
        pygame.draw.line(self.screen, (70, 70, 70), 
                        (crank_x - self.crank_length//2, crank_y),
                        (crank_x + self.crank_length//2, crank_y), 
                        self.crank_thickness)
        
        # Draw crank throws (pins and webs)
        for i in range(CYLINDERS):
            # Get pin offset angle for this cylinder
            pin_angle = math.radians(self.engine.crank_pin_angles[i])
            
            # Calculate pin position
            pin_angle_abs = (crank_angle_rad + pin_angle) % (2 * math.pi)
            pin_x = crank_x + self.crank_radius * math.sin(pin_angle_abs)
            pin_y = crank_y - self.crank_radius * math.cos(pin_angle_abs)
            
            # Draw crank web (connecting the main journal to the pin)
            pygame.draw.line(self.screen, CRANKSHAFT_COLOR, 
                           (crank_x, crank_y),
                           (pin_x, pin_y), 
                           12)
            
            # Draw crank pin
            pygame.draw.circle(self.screen, (150, 150, 150), (int(pin_x), int(pin_y)), self.crank_pin_radius)
            
    def draw_cylinders_and_pistons(self, crank_x, crank_y):
        """Draw cylinders, pistons and connecting rods"""
        # Calculate V-angle in radians
        v_angle_rad = math.radians(self.v_angle)
        
        # Left bank angle
        left_bank_angle = -v_angle_rad / 2
        # Right bank angle
        right_bank_angle = v_angle_rad / 2
        
        # Get cylinders for each bank (alternating in V6)
        for i in range(CYLINDERS):
            # Determine bank
            if i % 2 == 0:  # Cylinders 0, 2, 4 (1, 3, 5 in 1-based indexing)
                bank_angle = left_bank_angle
                bank_idx = i // 2  # Position within bank
            else:  # Cylinders 1, 3, 5 (2, 4, 6 in 1-based indexing) 
                bank_angle = right_bank_angle
                bank_idx = i // 2  # Position within bank
            
            # Calculate cylinder position
            cyl_offset = (bank_idx - 1) * self.cylinder_spacing
            cyl_x = crank_x + cyl_offset
            cyl_y = crank_y - 100  # Base height of cylinder banks
            
            # Calculate actual cylinder angles with bank angle
            cyl_dir_x = math.sin(bank_angle)
            cyl_dir_y = -math.cos(bank_angle)
            
            # Calculate cylinder bore positions
            cylinder_length = 160
            cylinder_top_x = cyl_x + cylinder_length * cyl_dir_x
            cylinder_top_y = cyl_y + cylinder_length * cyl_dir_y
            
            # Draw cylinder
            cylinder_width = 40
            # Calculate four corners of the cylinder
            cyl_points = [
                (cyl_x - cylinder_width//2 * math.cos(bank_angle + math.pi/2), 
                 cyl_y - cylinder_width//2 * math.sin(bank_angle + math.pi/2)),
                 
                (cyl_x + cylinder_width//2 * math.cos(bank_angle + math.pi/2), 
                 cyl_y + cylinder_width//2 * math.sin(bank_angle + math.pi/2)),
                 
                (cylinder_top_x + cylinder_width//2 * math.cos(bank_angle + math.pi/2), 
                 cylinder_top_y + cylinder_width//2 * math.sin(bank_angle + math.pi/2)),
                 
                (cylinder_top_x - cylinder_width//2 * math.cos(bank_angle + math.pi/2), 
                 cylinder_top_y - cylinder_width//2 * math.sin(bank_angle + math.pi/2))
            ]
            
            # Draw the cylinder with semi-transparency
            cyl_color = (70, 70, 70, 150)
            cyl_s = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            pygame.draw.polygon(cyl_s, cyl_color, cyl_points)
            self.screen.blit(cyl_s, (0, 0))
            
            # Get crank pin position
            pin_angle = math.radians((self.engine.crank_angle + self.engine.crank_pin_angles[i]) % 360)
            pin_x = crank_x + self.crank_radius * math.sin(pin_angle)
            pin_y = crank_y - self.crank_radius * math.cos(pin_angle)
            
            # Calculate piston position
            piston_pos = self.engine.piston_positions[i]
            piston_travel = cylinder_length * 0.8
            
            # Calculate piston center position along cylinder axis
            piston_x = cyl_x + piston_travel * piston_pos * cyl_dir_x
            piston_y = cyl_y + piston_travel * piston_pos * cyl_dir_y
            
            # Draw connecting rod
            rod_color = ROD_COLORS[min(i % 3, len(ROD_COLORS) - 1)]
            pygame.draw.line(self.screen, rod_color, (pin_x, pin_y), (piston_x, piston_y), 8)
            
            # Draw pin joint on connecting rod
            pygame.draw.circle(self.screen, (180, 180, 180), (int(pin_x), int(pin_y)), 8)
            
            # Draw piston
            piston_width = 38
            piston_height = 30
            
            # Get piston corners
            piston_rect = self.get_rotated_rect(
                piston_x, piston_y, 
                piston_width, piston_height, 
                math.degrees(bank_angle)
            )
            
            # Draw piston with opacity based on cylinder number
            piston_color = PISTON_COLORS[min(i % 3, len(PISTON_COLORS) - 1)]
            pygame.draw.polygon(self.screen, piston_color, piston_rect)
            pygame.draw.polygon(self.screen, (50, 50, 50), piston_rect, 2)
            
            # Draw cylinder number
            cylinder_num = FIRING_ORDER[i]
            num_surface = self.font.render(str(cylinder_num), True, (0, 0, 0))
            self.screen.blit(num_surface, (piston_x - num_surface.get_width() // 2, 
                                         piston_y - num_surface.get_height() // 2))
    
    def get_rotated_rect(self, x, y, width, height, angle_deg):
        """Return the four points of a rotated rectangle"""
        angle_rad = math.radians(angle_deg)
        half_w = width / 2
        half_h = height / 2
        
        # Calculate the four corners of the rectangle
        corners = [
            (-half_w, -half_h),  # Top left
            (half_w, -half_h),   # Top right
            (half_w, half_h),    # Bottom right
            (-half_w, half_h)    # Bottom left
        ]
        
        # Rotate and translate the corners
        rotated_corners = []
        for corner_x, corner_y in corners:
            # Rotate
            rot_x = corner_x * math.cos(angle_rad) - corner_y * math.sin(angle_rad)
            rot_y = corner_x * math.sin(angle_rad) + corner_y * math.cos(angle_rad)
            # Translate
            rotated_corners.append((x + rot_x, y + rot_y))
            
        return rotated_corners
    
    def draw_gauge(self, x, y, radius, label, value, min_val, max_val):
        """Draw a circular gauge with the given parameters"""
        # Draw gauge background
        pygame.draw.circle(self.screen, (30, 30, 30), (x, y), radius)
        pygame.draw.circle(self.screen, (200, 200, 200), (x, y), radius, 2)
        
        # Calculate angle for the value
        normalized_value = (value - min_val) / (max_val - min_val) if max_val > min_val else 0
        angle = 180 + normalized_value * 180  # 180 to 360 degrees
        
        # Convert to radians
        angle_rad = math.radians(angle)
        
        # Calculate end point
        end_x = x + (radius - 10) * math.cos(angle_rad)
        end_y = y + (radius - 10) * math.sin(angle_rad)
        
        # Draw needle
        color = (255, 255, 255)
        if value > max_val * 0.9:  # Red if near max
            color = (255, 0, 0)
        elif value > max_val * 0.75:  # Yellow if high
            color = (255, 255, 0)
        
        pygame.draw.line(self.screen, color, (x, y), (end_x, end_y), 3)
        pygame.draw.circle(self.screen, (100, 100, 100), (x, y), 5)
        
        # Draw ticks
        for i in range(11):
            tick_normalized = i / 10
            tick_angle = 180 + tick_normalized * 180
            tick_rad = math.radians(tick_angle)
            
            # Inner and outer tick positions
            inner_x = x + (radius - 15) * math.cos(tick_rad)
            inner_y = y + (radius - 15) * math.sin(tick_rad)
            outer_x = x + radius * math.cos(tick_rad)
            outer_y = y + radius * math.sin(tick_rad)
            
            # Draw tick
            pygame.draw.line(self.screen, (200, 200, 200), (inner_x, inner_y), (outer_x, outer_y), 2)
            
            # Draw tick label
            tick_value = min_val + tick_normalized * (max_val - min_val)
            if i % 2 == 0:  # Only show every other tick label
                label_x = x + (radius - 30) * math.cos(tick_rad)
                label_y = y + (radius - 30) * math.sin(tick_rad)
                label_surface = self.font.render(str(int(tick_value)), True, (150, 150, 150))
                self.screen.blit(label_surface, (label_x - label_surface.get_width() // 2, 
                                                label_y - label_surface.get_height() // 2))
    
    def draw_throttle(self):
        """Draw the throttle control"""
        # Draw throttle background
        pygame.draw.rect(self.screen, (50, 50, 50), self.throttle_rect)
        pygame.draw.rect(self.screen, (200, 200, 200), self.throttle_rect, 2)
        
        # Draw throttle fill
        fill_height = int(self.engine.throttle * self.throttle_rect.height)
        fill_rect = pygame.Rect(
            self.throttle_rect.x,
            self.throttle_rect.y + self.throttle_rect.height - fill_height,
            self.throttle_rect.width,
            fill_height
        )
        
        # Color changes based on throttle position
        throttle_color = (0, 255, 0)  # Green for low throttle
        if self.engine.throttle > 0.6:
            throttle_color = (255, 0, 0)  # Red for high throttle
        elif self.engine.throttle > 0.3:
            throttle_color = (255, 255, 0)  # Yellow for medium throttle
            
        pygame.draw.rect(self.screen, throttle_color, fill_rect)
        
        # Draw throttle handle
        self.throttle_handle_rect.y = self.throttle_rect.y + self.throttle_rect.height - fill_height - self.throttle_handle_rect.height // 2
        pygame.draw.rect(self.screen, (150, 150, 150), self.throttle_handle_rect, 0, 5)
        pygame.draw.rect(self.screen, (200, 200, 200), self.throttle_handle_rect, 2, 5)
        
        # Draw throttle percentage
        throttle_text = self.font.render(f"{int(self.engine.throttle * 100)}%", True, (255, 255, 255))
        self.screen.blit(throttle_text, (self.throttle_rect.x + self.throttle_rect.width // 2 - throttle_text.get_width() // 2, 
                                        self.throttle_rect.y + self.throttle_rect.height + 10))
    
    def draw_start_button(self):
        """Draw the engine start/stop button"""
        # Button color based on engine state
        if self.engine.running:
            button_color = (200, 0, 0)  # Red for stop
            button_text = "STOP"
        else:
            button_color = (0, 200, 0)  # Green for start
            button_text = "START"
            
        # Draw button background
        pygame.draw.rect(self.screen, button_color, self.start_button_rect, 0, 10)
        pygame.draw.rect(self.screen, (200, 200, 200), self.start_button_rect, 2, 10)
        
        # Draw button text
        text_surface = self.font.render(button_text, True, (255, 255, 255))
        text_x = self.start_button_rect.x + (self.start_button_rect.width - text_surface.get_width()) // 2
        text_y = self.start_button_rect.y + (self.start_button_rect.height - text_surface.get_height()) // 2
        self.screen.blit(text_surface, (text_x, text_y))
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.engine.running = False
                pygame.quit()
                return False
                
            elif event.type == MOUSEBUTTONDOWN:
                # Check if clicked on start/stop button
                if self.start_button_rect.collidepoint(event.pos):
                    if self.engine.running:
                        self.engine.stop_engine()
                    else:
                        self.engine.start_engine()
                # Check if clicked on throttle handle or throttle bar
                elif self.throttle_handle_rect.collidepoint(event.pos):
                    self.dragging_throttle = True
                elif self.throttle_rect.collidepoint(event.pos):
                    # Direct click on throttle bar - set throttle directly
                    mouse_y = event.pos[1]
                    throttle_pos = (self.throttle_rect.y + self.throttle_rect.height - mouse_y) / self.throttle_rect.height
                    self.engine.throttle = max(0.0, min(1.0, throttle_pos))
                    # Update handle position
                    self.throttle_handle_rect.y = self.throttle_rect.y + self.throttle_rect.height - int(self.engine.throttle * self.throttle_rect.height) - self.throttle_handle_rect.height // 2
                    
            elif event.type == MOUSEBUTTONUP:
                self.dragging_throttle = False
                
            elif event.type == MOUSEMOTION and self.dragging_throttle:
                # Update throttle based on mouse position
                mouse_y = event.pos[1]
                throttle_pos = (self.throttle_rect.y + self.throttle_rect.height - mouse_y) / self.throttle_rect.height
                self.engine.throttle = max(0.0, min(1.0, throttle_pos))
                
            elif event.type == KEYDOWN:
                if event.key == K_UP:
                    self.engine.throttle = min(1.0, self.engine.throttle + 0.1)
                elif event.key == K_DOWN:
                    self.engine.throttle = max(0.0, self.engine.throttle - 0.1)
                elif event.key == K_SPACE:
                    # Space bar toggles engine state
                    if self.engine.running:
                        self.engine.stop_engine()
                    else:
                        self.engine.start_engine()
                    
        return True
    
    def engine_loop(self):
        """Main engine simulation loop running in a separate thread"""
        last_time = time.time()
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Update engine state
            self.engine.update(dt)
            
            # Small delay to prevent 100% CPU usage
            time.sleep(0.01)
    
    def run(self):
        """Main simulation loop"""
        while True:
            if not self.handle_events():
                break
                
            # Draw engine
            self.draw_engine()
            
            # Update sound
            self.sound_manager.update()
            
            pygame.display.flip()
            self.clock.tick(60)
            
        # Clean up
        pygame.quit()

if __name__ == "__main__":
    simulator = EngineSimulator()
    simulator.run()