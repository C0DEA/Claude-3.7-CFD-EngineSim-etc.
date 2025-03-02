import numpy as np
import pygame
import math
import time
from pygame.locals import *
import threading
import os

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
        self.running = True
        
        # Engine geometry parameters
        self.stroke = 88.4  # mm
        self.bore = 95.5   # mm
        self.rod_length = 158.0  # mm
        self.compression_ratio = 9.0
        
        # Sound parameters
        self.sound_volume = 0.2
        
    def calculate_piston_positions(self):
        """Calculate piston positions based on crank angle and V6 geometry"""
        for i in range(CYLINDERS):
            # Calculate firing order offset
            cylinder_idx = FIRING_ORDER[i] - 1
            # V6 engines have two banks, with cylinders 120 degrees apart
            if cylinder_idx < 3:  # Left bank
                angle_offset = 0
            else:  # Right bank
                angle_offset = CYLINDER_ANGLE_V6
                
            # Each cylinder fires 120 degrees apart in a V6
            firing_offset = (cylinder_idx % 3) * 240
            
            # Calculate piston position using crank angle geometry
            crank_pos = (self.crank_angle + firing_offset) % 720
            crank_rad = math.radians(crank_pos)
            
            # Piston position is based on crankshaft rotation
            # 0 is TDC (Top Dead Center), 1 is BDC (Bottom Dead Center)
            r = self.stroke / 2
            piston_pos = r * math.cos(crank_rad) + math.sqrt(self.rod_length**2 - (r * math.sin(crank_rad))**2)
            
            # Normalize to 0-1 range where 0 is TDC and 1 is BDC
            piston_pos_normalized = 1 - (piston_pos - (self.rod_length - r)) / (2 * r)
            
            # Store position
            self.piston_positions[i] = piston_pos_normalized
            
    def update(self, dt):
        """Update engine state based on throttle and time step"""
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
        self.rpm = max(IDLE_RPM, min(self.rpm, MAX_RPM))
        
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

class EngineSoundManager:
    def __init__(self, engine):
        self.engine = engine
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
        self.channel = pygame.mixer.Channel(0)
        
        # Create sound objects for different RPM ranges
        self.rpm_ranges = [
            (0, 2000),
            (2000, 4000),
            (4000, 6000),
            (6000, MAX_RPM)
        ]
        
        # Create synthesized engine sounds
        self.sound_objects = self.generate_engine_sounds()
        self.current_sound_idx = 0
        self.playing = False
        
    def generate_engine_sounds(self):
        """Generate synthesized engine sounds for different RPM ranges"""
        sample_rate = 44100
        duration = 2.0  # seconds
        sound_objects = []
        
        for i, (rpm_min, rpm_max) in enumerate(self.rpm_ranges):
            # Average RPM for this range
            avg_rpm = (rpm_min + rpm_max) / 2
            
            # Base frequency calculation - higher RPM = higher pitch
            base_freq = 80 + 0.05 * avg_rpm
            
            # Create a buffer for sound data
            buffer = np.zeros(int(sample_rate * duration))
            
            # Add harmonics to create a rich engine sound
            t = np.linspace(0, duration, len(buffer), False)
            
            # Main frequency component (changes with RPM)
            buffer += 0.5 * np.sin(2 * np.pi * base_freq * t)
            
            # Add V6 firing pulses (every 120 degrees of rotation)
            rpm_hz = avg_rpm / 60  # Revolutions per second
            cycle_time = 1 / rpm_hz
            pulse_freq = (rpm_hz * 3)  # For a V6, 3 firing pulses per revolution
            
            # Add pulses
            buffer += 0.3 * np.sin(2 * np.pi * pulse_freq * t)
            
            # Add higher harmonics for mechanical noise
            buffer += 0.15 * np.sin(2 * np.pi * base_freq * 2 * t)
            buffer += 0.1 * np.sin(2 * np.pi * base_freq * 3 * t)
            buffer += 0.05 * np.sin(2 * np.pi * base_freq * 4 * t)
            
            # Add some noise for realism
            buffer += 0.1 * np.random.normal(0, 1, len(buffer))
            
            # Normalize
            buffer = 0.5 * buffer / np.max(np.abs(buffer))
            
            # Convert to 16-bit PCM
            buffer = (buffer * 32767).astype(np.int16)
            
            # Create stereo sound
            stereo_buffer = np.column_stack((buffer, buffer))
            
            # Create a sound object directly from the array
            sound_obj = pygame.mixer.Sound(stereo_buffer)
            sound_obj.set_volume(0.5)
            sound_objects.append(sound_obj)
            
        return sound_objects
    
    def update(self):
        """Update sound based on engine RPM"""
        # Determine which sound file to play based on RPM
        for i, (rpm_min, rpm_max) in enumerate(self.rpm_ranges):
            if rpm_min <= self.engine.rpm < rpm_max:
                sound_idx = i
                break
        else:
            sound_idx = len(self.rpm_ranges) - 1
            
        # Volume based on throttle
        volume = 0.2 + 0.8 * self.engine.throttle
        
        # If we need to change the sound
        if sound_idx != self.current_sound_idx or not self.playing:
            if self.playing:
                self.channel.stop()
            
            self.channel.play(self.sound_objects[sound_idx], loops=-1)
            self.current_sound_idx = sound_idx
            self.playing = True
            
        # Update volume
        self.channel.set_volume(volume)

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
        self.sound_manager = EngineSoundManager(self.engine)
        
        # Engine rendering parameters
        self.center_x = WIDTH // 2
        self.center_y = HEIGHT // 2 - 70
        self.piston_width = 60
        self.piston_height = 40
        self.rod_width = 15
        self.rod_length = 100
        self.crank_radius = 40
        self.v_angle = CYLINDER_ANGLE_V6
        
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
        
        # Draw crankshaft
        crank_x = self.center_x
        crank_y = self.center_y + 120
        pygame.draw.circle(self.screen, CRANKSHAFT_COLOR, (crank_x, crank_y), 25)
        
        # Calculate crank journal position
        crank_journal_angle = math.radians(self.engine.crank_angle % 360)
        journal_x = crank_x + self.crank_radius * math.sin(crank_journal_angle)
        journal_y = crank_y - self.crank_radius * math.cos(crank_journal_angle)
        
        # Draw cylinders and pistons - now with proper cylinder bores
        left_bank_cylinders = [0, 2, 4]  # Cylinders 1, 3, 5
        right_bank_cylinders = [1, 3, 5]  # Cylinders 2, 4, 6
        
        # Calculate bank angles
        left_bank_angle = -self.v_angle / 2
        right_bank_angle = self.v_angle / 2
        
        # Draw left bank cylinders (1, 3, 5)
        self.draw_cylinder_bank(left_bank_cylinders, left_bank_angle, crank_x, crank_y, journal_x, journal_y)
        
        # Draw right bank cylinders (2, 4, 6)
        self.draw_cylinder_bank(right_bank_cylinders, right_bank_angle, crank_x, crank_y, journal_x, journal_y)
        
        # Draw crank journal
        pygame.draw.circle(self.screen, (150, 150, 150), (int(journal_x), int(journal_y)), 15)
        
        # Draw RPM gauge
        self.draw_gauge(150, HEIGHT - 100, 100, "RPM", self.engine.rpm, 0, MAX_RPM)
        
        # Draw torque gauge
        self.draw_gauge(350, HEIGHT - 100, 100, "Torque (Nm)", self.engine.torque, 0, 600)
        
        # Draw HP gauge
        self.draw_gauge(550, HEIGHT - 100, 100, "Horsepower", self.engine.horsepower, 0, 600)
        
        # Draw throttle control
        self.draw_throttle()
        
        # Draw RPM text
        rpm_text = self.font.render(f"{int(self.engine.rpm)} RPM", True, (255, 255, 255))
        self.screen.blit(rpm_text, (150 - rpm_text.get_width() // 2, HEIGHT - 160))
        
        # Draw torque text
        torque_text = self.font.render(f"{int(self.engine.torque)} Nm", True, (255, 255, 255))
        self.screen.blit(torque_text, (350 - torque_text.get_width() // 2, HEIGHT - 160))
        
        # Draw HP text
        hp_text = self.font.render(f"{int(self.engine.horsepower)} HP", True, (255, 255, 255))
        self.screen.blit(hp_text, (550 - hp_text.get_width() // 2, HEIGHT - 160))
    
    def draw_cylinder_bank(self, cylinder_indices, bank_angle, crank_x, crank_y, journal_x, journal_y):
        """Draw a bank of cylinders with pistons and rods"""
        bank_rad = math.radians(bank_angle)
        
        # Calculate the direction vector for the cylinder bank
        bank_dir_x = math.sin(bank_rad)
        bank_dir_y = -math.cos(bank_rad)
        
        for i, bank_idx in enumerate(cylinder_indices):
            # Get the actual cylinder number from firing order
            cylinder_num = FIRING_ORDER[bank_idx]
            
            # Calculate cylinder position along the bank
            cylinder_offset = (i - 1) * self.cylinder_spacing
            
            # Base position of cylinder in bank
            cylinder_base_x = crank_x + cylinder_offset * math.cos(bank_rad)
            cylinder_base_y = crank_y + cylinder_offset * math.sin(bank_rad)
            
            # Calculate top of cylinder
            cylinder_top_x = cylinder_base_x + self.cylinder_height * bank_dir_x
            cylinder_top_y = cylinder_base_y + self.cylinder_height * bank_dir_y
            
            # Draw cylinder bore
            points = [
                (cylinder_base_x - self.cylinder_width//2 * math.cos(bank_rad + math.pi/2), 
                 cylinder_base_y - self.cylinder_width//2 * math.sin(bank_rad + math.pi/2)),
                
                (cylinder_base_x + self.cylinder_width//2 * math.cos(bank_rad + math.pi/2), 
                 cylinder_base_y + self.cylinder_width//2 * math.sin(bank_rad + math.pi/2)),
                
                (cylinder_top_x + self.cylinder_width//2 * math.cos(bank_rad + math.pi/2), 
                 cylinder_top_y + self.cylinder_width//2 * math.sin(bank_rad + math.pi/2)),
                
                (cylinder_top_x - self.cylinder_width//2 * math.cos(bank_rad + math.pi/2), 
                 cylinder_top_y - self.cylinder_width//2 * math.sin(bank_rad + math.pi/2))
            ]
            
            # Draw cylinder
            pygame.draw.polygon(self.screen, CYLINDER_COLOR, points)
            pygame.draw.polygon(self.screen, (100, 100, 100), points, 2)
            
            # Calculate piston position
            piston_travel = self.cylinder_height * 0.8
            piston_pos = self.engine.piston_positions[bank_idx]
            
            # Calculate piston position in cylinder
            piston_x = cylinder_base_x + (piston_travel * piston_pos) * bank_dir_x
            piston_y = cylinder_base_y + (piston_travel * piston_pos) * bank_dir_y
            
            # Calculate transparency based on cylinder number
            alpha = 255 - bank_idx * 30
            
            # Draw connecting rod
            rod_color = list(ROD_COLORS[min(bank_idx % 3, len(ROD_COLORS) - 1)])
            rod_color.append(alpha)
            pygame.draw.line(self.screen, rod_color, (journal_x, journal_y), (piston_x, piston_y), 8)
            
            # Draw piston
            piston_color = list(PISTON_COLORS[min(bank_idx % 3, len(PISTON_COLORS) - 1)])
            piston_color.append(alpha)
            
            # Rectangle points for piston
            piston_rect = self.get_rotated_rect(
                piston_x, piston_y, 
                self.piston_width, self.piston_height, 
                bank_angle
            )
            
            # Draw piston
            pygame.draw.polygon(self.screen, piston_color, piston_rect)
            pygame.draw.polygon(self.screen, (50, 50, 50), piston_rect, 2)
            
            # Draw cylinder number
            if bank_idx < 3:  # Adjust label position based on bank
                label_offset_x = -25
            else:
                label_offset_x = 25
                
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
        normalized_value = (value - min_val) / (max_val - min_val)
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
        """Draw the throttle control - improved and more accessible"""
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
    
    def handle_events(self):
        """Handle pygame events - improved throttle control interaction"""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.engine.running = False
                pygame.quit()
                return False
                
            elif event.type == MOUSEBUTTONDOWN:
                # Check if clicked on throttle handle or throttle bar
                if self.throttle_handle_rect.collidepoint(event.pos):
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
                    
        return True
    
    def engine_loop(self):
        """Main engine simulation loop running in a separate thread"""
        last_time = time.time()
        while self.engine.running:
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