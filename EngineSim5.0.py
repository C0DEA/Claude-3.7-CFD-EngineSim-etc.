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

# Color constants based on the reference image
PISTON_COLOR = (255, 255, 255)  # White pistons
CYLINDER_COLOR = (255, 153, 204)  # Pink cylinders
CONNECTING_ROD_COLOR = (102, 204, 255)  # Light blue connecting rods
CRANKSHAFT_COLOR = (200, 200, 200)  # Light gray crankshaft
PIN_COLOR = (0, 0, 0)  # Black pins
VALVE_COLOR = (102, 204, 255)  # Light blue valve circles
SPRING_COLOR = (255, 204, 0)  # Yellow spring circles
BACKGROUND_COLOR = (0, 0, 0)  # Black background

# Engine dimensions
PISTON_WIDTH = 60
PISTON_HEIGHT = 80
CYLINDER_WIDTH = 70
CYLINDER_HEIGHT = 160
ROD_LENGTH = 120  # Fixed rod length
CRANK_RADIUS = 40

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

class Engine:
    def __init__(self):
        self.rpm = IDLE_RPM
        self.target_rpm = IDLE_RPM
        self.throttle = 0.0  # 0.0 to 1.0
        self.crank_angle = 0.0  # degrees
        self.piston_positions = [(0.0, 0.0)] * CYLINDERS  # (position, angle)
        self.torque = 0.0
        self.horsepower = 0.0
        self.running = False  # Engine starts off
        self.starting = False
        self.start_time = 0
        
        # Engine geometry parameters
        self.rod_length = ROD_LENGTH  # Fixed rod length
        self.crank_radius = CRANK_RADIUS
        
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
            # Get crank pin angle for this cylinder
            pin_angle = self.crank_pin_angles[i]
            
            # For V6 engine, determine which bank the cylinder is in
            # Left bank: cylinders 0, 2, 4 (1, 3, 5 in 1-based)
            # Right bank: cylinders 1, 3, 5 (2, 4, 6 in 1-based)
            if i % 2 == 0:
                bank_angle = -CYLINDER_ANGLE_V6 / 2  # Left bank
            else:
                bank_angle = CYLINDER_ANGLE_V6 / 2  # Right bank
            
            # Calculate effective crank angle for this cylinder
            effective_angle = (self.crank_angle + pin_angle) % 360
            effective_rad = math.radians(effective_angle)
            
            # Calculate crank pin position
            crank_x = self.crank_radius * math.cos(effective_rad)
            crank_y = self.crank_radius * math.sin(effective_rad)
            
            # Calculate piston position with fixed rod length
            # Using the rod-crank formula for V-engine
            # We need to project the motion along the cylinder bank angle
            bank_rad = math.radians(bank_angle)
            
            # Component of crank along bank direction
            bank_component = crank_x * math.cos(bank_rad) + crank_y * math.sin(bank_rad)
            
            # Calculate rod angle using law of cosines
            d = math.sqrt(self.crank_radius**2 - bank_component**2)
            h = math.sqrt(self.rod_length**2 - d**2)
            
            # Piston distance from crank center along bank angle
            piston_dist = bank_component + h
            
            # Normalize to 0-1 range
            max_dist = self.crank_radius + self.rod_length
            min_dist = self.rod_length - self.crank_radius
            piston_pos = (piston_dist - min_dist) / (max_dist - min_dist)
            piston_pos = 1 - piston_pos  # Invert so 0 is TDC, 1 is BDC
            
            # Store position and bank angle
            self.piston_positions[i] = (piston_pos, bank_angle)
            
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
        
        # Important: Set enough channels for our needs
        pygame.mixer.set_num_channels(12)  # Make sure we have enough channels
        
        # Create channels for different sounds
        self.explosion_channels = [pygame.mixer.Channel(i) for i in range(8)]  # 8 channels for explosions
        self.channel_index = 0
        
        # Idle sound channel and start sound channel
        self.idle_channel = pygame.mixer.Channel(8)  # Changed from 9 to 8
        self.start_channel = pygame.mixer.Channel(9)  # Changed from 10 to 9
        
        # Attempt to load the explosion sound
        try:
            self.explosion_sound = pygame.mixer.Sound('sound.wav')
            print("Loaded sound.wav successfully")
        except:
            # Create a fallback sound if file not found
            print("Warning: sound.wav not found, using synthesized sound")
            self.explosion_sound = self.create_fallback_explosion_sound()
        
        # Create starting sound
        self.start_sound = self.create_engine_start_sound()
        
        # Track last fire time to manage sound frequency
        self.last_fire_time = 0
        
        # Idle sound
        self.idle_sound = self.create_idle_sound()
        self.idle_playing = False
        
        # For precise timing of explosion sounds
        self.last_crank_angle = 0
        
    def create_fallback_explosion_sound(self):
        """Create a synthesized explosion sound if the WAV file is not found"""
        sample_rate = 44100
        duration = 0.1  # seconds - shorter for faster playback
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate a percussive sound
        t = np.linspace(0, duration, len(buffer), False)
        
        # Create attack with high frequency energy, then decay
        for i, time in enumerate(t):
            if time < 0.01:  # Sharp attack
                buffer[i] = np.random.normal(0, 1) * (time / 0.01)
            else:
                # Exponential decay with some randomness
                decay = np.exp(-(time - 0.01) * 40)
                buffer[i] = np.random.normal(0, 0.5) * decay
        
        # Add some low frequency content
        buffer += 0.7 * np.sin(2 * np.pi * 80 * t) * np.exp(-t * 20)
        
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
        """Update sound based on engine state and RPM"""
        # Handle engine starting/stopping
        if self.engine.starting and not self.start_channel.get_busy():
            self.play_start_sound()
            
        # If engine is not running or starting, stop all sounds
        if not self.engine.running and not self.engine.starting:
            for channel in self.explosion_channels:
                if channel.get_busy():
                    channel.fadeout(500)  # Fade out over 500ms
            if self.idle_playing:
                self.idle_channel.fadeout(500)
                self.idle_playing = False
            return
            
        # Play idle sound if not already playing
        if self.engine.running and not self.idle_playing:
            self.idle_channel.play(self.idle_sound, loops=-1)
            self.idle_playing = True
        
        # Adjust idle sound volume based on RPM
        if self.idle_playing:
            # Volume decreases as RPM increases (engine noise takes over)
            idle_volume = max(0.1, 0.5 - self.engine.rpm / MAX_RPM * 0.4)
            self.idle_channel.set_volume(idle_volume)
        
        # Calculate how often to play explosion sounds based on RPM
        # In a 4-stroke engine, each cylinder fires once every 2 revolutions
        # For a V6, that's 3 firings per revolution
        
        # Calculate time between explosion sounds (in seconds)
        if self.engine.rpm > 0:
            # RPM / 60 = revolutions per second
            # For V6 4-stroke: 3 explosions per revolution (6 cylinders / 2 revolutions)
            explosions_per_second = (self.engine.rpm / 60) * 3
            time_between_explosions = 1.0 / explosions_per_second
            
            # Detect if it's time to play another explosion sound
            current_time = time.time()
            if current_time - self.last_fire_time >= time_between_explosions:
                self.last_fire_time = current_time
                
                # Play explosion sound with volume based on throttle
                volume = min(1.0, 0.3 + 0.7 * self.engine.throttle)
                
                # Use the next available channel
                channel = self.explosion_channels[self.channel_index]
                self.channel_index = (self.channel_index + 1) % len(self.explosion_channels)
                
                # Play the sound
                channel.play(self.explosion_sound)
                channel.set_volume(volume)


class StylizedEngineSimulator:
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
        self.center_y = HEIGHT // 2
        
        # Throttle UI - make it look like the screenshot
        self.throttle_rect = pygame.Rect(WIDTH - 60, HEIGHT // 2 - 150, 30, 300)
        self.throttle_handle_rect = pygame.Rect(WIDTH - 70, HEIGHT // 2, 50, 20)
        self.dragging_throttle = False
        
        # Start/Stop button - green rectangle like in screenshot
        self.start_button_rect = pygame.Rect(WIDTH - 120, HEIGHT - 80, 100, 40)
        
        # Engine thread
        self.engine_thread = threading.Thread(target=self.engine_loop)
        self.engine_thread.daemon = True
        self.engine_thread.start()
        
    def draw_engine(self):
        """Draw the engine with all moving parts"""
        # Fill background
        self.screen.fill(BACKGROUND_COLOR)
        
        # Draw title
        title_surface = self.title_font.render("Nissan GTR VR38DETT V6 Engine", True, (220, 220, 220))
        self.screen.blit(title_surface, (self.center_x - title_surface.get_width() // 2, 20))
        
        # Draw engine specs
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
            self.screen.blit(spec_surface, (20, y_pos))
            y_pos += 25
        
        # Draw crankshaft and engine
        crank_x = self.center_x
        crank_y = self.center_y + 80
        
        # Draw the stylized crankshaft
        self.draw_stylized_crankshaft(crank_x, crank_y)
        
        # Draw each bank of cylinders (completely overlapping like in screenshot)
        self.draw_overlapping_cylinders(crank_x, crank_y)
        
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
        
        # Draw engine state text at bottom left like in screenshot
        state_text = "RUNNING" if self.engine.running else "STARTING" if self.engine.starting else "OFF"
        state_color = (255, 255, 0)  # Yellow like in screenshot
        state_surface = self.font.render(f"Engine: {state_text}", True, state_color)
        self.screen.blit(state_surface, (20, HEIGHT - 30))
        
    def draw_stylized_crankshaft(self, crank_x, crank_y):
        """Draw a stylized crankshaft like in the reference image"""
        # Draw main crankshaft centerpiece
        pygame.draw.circle(self.screen, CRANKSHAFT_COLOR, (crank_x, crank_y), 35)
        pygame.draw.circle(self.screen, (120, 120, 120), (crank_x, crank_y), 20)
        
        # Draw a smaller center circle
        pygame.draw.circle(self.screen, (100, 100, 100), (crank_x, crank_y), 10)
        
    def draw_overlapping_cylinders(self, crank_x, crank_y):
        """Draw the cylinder banks with completely overlapping cylinders"""
        # Calculate V bank angles
        left_bank_angle = -CYLINDER_ANGLE_V6 / 2
        right_bank_angle = CYLINDER_ANGLE_V6 / 2
        
        # Draw left bank (3 overlapping cylinders)
        self.draw_bank_cylinders(crank_x, crank_y, left_bank_angle, [0, 2, 4])
        
        # Draw right bank (3 overlapping cylinders)
        self.draw_bank_cylinders(crank_x, crank_y, right_bank_angle, [1, 3, 5])
        
    def draw_bank_cylinders(self, crank_x, crank_y, bank_angle, cylinder_indices):
        """Draw a bank of overlapping cylinders with transparent interiors"""
        bank_rad = math.radians(bank_angle)
        dir_x = math.sin(bank_rad)
        dir_y = -math.cos(bank_rad)
        
        # Draw cylinders in reverse order (back to front)
        for i, cyl_idx in reversed(list(enumerate(cylinder_indices))):
            # Calculate transparency - more transparent for cylinders at the back
            alpha = 255 - i * 80
            
            # Get piston position for this cylinder
            piston_pos, _ = self.engine.piston_positions[cyl_idx]
            
            # Calculate effective crank angle
            pin_angle = self.engine.crank_pin_angles[cyl_idx]
            effective_angle = (self.engine.crank_angle + pin_angle) % 360
            effective_rad = math.radians(effective_angle)
            
            # Calculate crank pin position
            pin_x = crank_x + CRANK_RADIUS * math.cos(effective_rad)
            pin_y = crank_y - CRANK_RADIUS * math.sin(effective_rad)
            
            # Calculate cylinder dimensions
            cylinder_height = 220
            cylinder_width = 70
            
            # Calculate cylinder position - all cylinders in same bank have same base position
            cylinder_base_x = crank_x
            cylinder_base_y = crank_y
            
            # Calculate cylinder top
            cylinder_top_x = cylinder_base_x + cylinder_height * dir_x
            cylinder_top_y = cylinder_base_y + cylinder_height * dir_y
            
            # Calculate piston position based on piston_pos (0.0 to 1.0)
            piston_travel = cylinder_height * 0.7  # 70% of cylinder height
            piston_x = cylinder_base_x + piston_travel * piston_pos * dir_x
            piston_y = cylinder_base_y + piston_travel * piston_pos * dir_y
            
            # Create surfaces for transparency
            cylinder_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            
            # Get cylinder outline points
            cylinder_rect = self.get_rotated_rect(
                cylinder_base_x + (cylinder_height/2) * dir_x,
                cylinder_base_y + (cylinder_height/2) * dir_y,
                cylinder_width, cylinder_height,
                math.degrees(bank_rad)
            )
            
            # Draw only the cylinder outline (not filled)
            cylinder_color = list(CYLINDER_COLOR) + [alpha]
            pygame.draw.polygon(cylinder_surface, cylinder_color, cylinder_rect, 3)  # Outline only, 3 pixels thick
            
            # Draw connecting rod
            rod_color = list(CONNECTING_ROD_COLOR) + [alpha]
            pygame.draw.line(cylinder_surface, rod_color, (pin_x, pin_y), (piston_x, piston_y), 8)
            
            # Draw piston with alpha
            piston_color = list(PISTON_COLOR) + [alpha]
            piston_rect = self.get_rotated_rect(
                piston_x, piston_y,
                cylinder_width - 10, 50,  # Slightly smaller than cylinder
                math.degrees(bank_rad)
            )
            pygame.draw.polygon(cylinder_surface, piston_color, piston_rect)
            
            # Draw piston pin (black dot)
            pin_color = list(PIN_COLOR) + [alpha]
            pygame.draw.circle(cylinder_surface, pin_color, (int(piston_x), int(piston_y)), 5)
            
            # Draw valves and springs at top of cylinder
            valve_offset = 20
            valve_radius = 10
            spring_radius = 10
            
            # Left valve (blue)
            valve1_x = cylinder_top_x - valve_offset * math.cos(bank_rad + math.pi/2)
            valve1_y = cylinder_top_y - valve_offset * math.sin(bank_rad + math.pi/2)
            valve_color = list(VALVE_COLOR) + [alpha]
            pygame.draw.circle(cylinder_surface, valve_color, (int(valve1_x), int(valve1_y)), valve_radius)
            
            # Left spring (yellow)
            spring1_x = valve1_x + 15 * dir_x
            spring1_y = valve1_y + 15 * dir_y
            spring_color = list(SPRING_COLOR) + [alpha]
            pygame.draw.circle(cylinder_surface, spring_color, (int(spring1_x), int(spring1_y)), spring_radius)
            
            # Right valve (blue)
            valve2_x = cylinder_top_x + valve_offset * math.cos(bank_rad + math.pi/2)
            valve2_y = cylinder_top_y + valve_offset * math.sin(bank_rad + math.pi/2)
            pygame.draw.circle(cylinder_surface, valve_color, (int(valve2_x), int(valve2_y)), valve_radius)
            
            # Right spring (yellow)
            spring2_x = valve2_x + 15 * dir_x
            spring2_y = valve2_y + 15 * dir_y
            pygame.draw.circle(cylinder_surface, spring_color, (int(spring2_x), int(spring2_y)), spring_radius)
            
            # Blit cylinder surface to main screen
            self.screen.blit(cylinder_surface, (0, 0))
            
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
        """Draw the throttle control like in the screenshot"""
        # Draw "Throttle" label
        throttle_label = self.font.render("Throttle", True, (220, 220, 220))
        self.screen.blit(throttle_label, (WIDTH - 90, HEIGHT // 2 - 180))
        
        # Draw throttle background - white rectangle
        pygame.draw.rect(self.screen, (50, 50, 50), self.throttle_rect)
        pygame.draw.rect(self.screen, (255, 255, 255), self.throttle_rect, 1)
        
        # Draw throttle handle - gray rectangle
        self.throttle_handle_rect.y = self.throttle_rect.y + self.throttle_rect.height - int(self.engine.throttle * self.throttle_rect.height) - self.throttle_handle_rect.height // 2
        pygame.draw.rect(self.screen, (150, 150, 150), self.throttle_handle_rect)
        
        # Draw throttle percentage
        throttle_text = self.font.render(f"{int(self.engine.throttle * 100)}%", True, (255, 255, 255))
        self.screen.blit(throttle_text, (WIDTH - 70, self.throttle_rect.y + self.throttle_rect.height + 10))
    
    def draw_start_button(self):
        """Draw the start/stop button like in the screenshot"""
        # Button is bright green when not running, different color when running
        if self.engine.running or self.engine.starting:
            button_color = (200, 0, 0)  # Red for stop
            button_text = "STOP"
        else:
            button_color = (0, 255, 0)  # Bright green for start (like in screenshot)
            button_text = "START"
            
        # Draw button - match the rounded rectangle in screenshot
        pygame.draw.rect(self.screen, button_color, self.start_button_rect, 0, 5)
        
        # Draw button text - white and centered
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
    simulator = StylizedEngineSimulator()
    simulator.run()