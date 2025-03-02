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
DEFAULT_V_ANGLE = 60  # Default V angle (degrees)
FIRING_ORDER = [1, 2, 3, 4, 5, 6]  # Updated to Nissan GTR firing order
IDLE_RPM = 750
MAX_RPM = 7100
REDLINE_RPM = 6800

# Modified color constants - reverted to original colors
PISTON_COLOR = (255, 255, 255)  # White pistons
CYLINDER_COLOR = (150, 150, 150)  # Gray cylinders
CYLINDER_WALL_COLOR = (255, 153, 204)  # Pink cylinder walls
CONNECTING_ROD_COLOR = (102, 204, 255)  # Light blue connecting rods
CRANKSHAFT_COLOR = (200, 200, 200)  # Light gray crankshaft
PIN_COLOR = (0, 0, 0)  # Black pins
BACKGROUND_COLOR = (0, 0, 0)  # Black background

# Engine dimensions
PISTON_WIDTH = 60
PISTON_HEIGHT = 80
CYLINDER_WIDTH = 70
CYLINDER_HEIGHT = 160
ROD_LENGTH = 120  # Fixed rod length
CRANK_RADIUS = 40
CYLINDER_OFFSET = 80  # Increased distance from crank center to cylinder base

# NEW: Piston BDC adjustment factor to prevent crankshaft collision
BDC_ADJUSTMENT = 0.4  # Increased adjustment to keep pistons further from crankshaft (0-1)

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
        self.slow_mode = False  # Flag for 60RPM mode
        
        # Engine geometry parameters
        self.rod_length = ROD_LENGTH  # Fixed rod length
        self.crank_radius = CRANK_RADIUS
        self.cylinder_offset = CYLINDER_OFFSET  # Distance from crank center to cylinder base
        
        # Variable engine configuration
        self.v_angle = DEFAULT_V_ANGLE  # Can be changed from 0 (inline) to 180 (boxer)
        
        # Use proper Nissan GTR ignition timing with 120° offsets between cylinder pairs
        # For a V6 engine, cylinders fire every 120° of crankshaft rotation
        self.crank_pin_angles = [0, 240, 120, 0, 240, 120]  # Nissan GTR firing pattern
        
        # Calculate firing events (when each cylinder fires in the cycle)
        self.firing_events = []
        
        # In a V6 engine, the firing sequence occurs every 120° of crankshaft rotation
        # For the Nissan GTR VR38DETT, we'll use the proper firing angles
        firing_angles = [0, 120, 240, 360, 480, 600]  # Every 120° in a 720° cycle
        
        for i, cylinder in enumerate(FIRING_ORDER):
            # Use specified firing angles
            firing_angle = firing_angles[i]
            self.firing_events.append((cylinder - 1, firing_angle))
            
        # Sort by firing angle
        self.firing_events.sort(key=lambda x: x[1])
        
        # Track TDC/BDC crossings and stroke positions for sound generation
        self.last_piston_positions = [(0.5, 0.0)] * CYLINDERS
        
        # Track the current stroke for each cylinder (0=intake, 1=compression, 2=power, 3=exhaust)
        self.cylinder_strokes = [0] * CYLINDERS
        
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
        self.slow_mode = False
        
    def set_slow_mode(self, enabled):
        """Set slow mode (60 RPM for visualization)"""
        self.slow_mode = enabled
        
    def set_v_angle(self, angle):
        """Set the V-angle of the engine (0 for inline, 60 for V6, 180 for boxer)"""
        self.v_angle = angle
        
    def calculate_piston_positions(self):
        """Calculate piston positions based on crank angle and engine geometry"""
        for i in range(CYLINDERS):
            # Get crank pin angle for this cylinder
            pin_angle = self.crank_pin_angles[i]
            
            # For variable angle engine, determine which bank the cylinder is in
            # Left bank: cylinders 0, 2, 4 (1, 3, 5 in 1-based)
            # Right bank: cylinders 1, 3, 5 (2, 4, 6 in 1-based)
            if i % 2 == 0:
                bank_angle = -self.v_angle / 2  # Left bank
            else:
                bank_angle = self.v_angle / 2  # Right bank
            
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
            # Fixed normalization to ensure same ATH and bottom for all cylinders
            # These fixed values ensure all pistons have the same stroke range
            max_dist = self.rod_length + self.crank_radius
            min_dist = self.rod_length - self.crank_radius
            piston_pos = (piston_dist - min_dist) / (max_dist - min_dist)
            piston_pos = 1 - piston_pos  # Invert so 0 is TDC, 1 is BDC
            
            # Apply BDC adjustment to prevent pistons from "touching" the crankshaft
            # Only modify when piston is near BDC (position > 0.7)
            if piston_pos > 0.7:
                # Scale the adjustment - more effect as piston approaches BDC
                # This creates a non-linear motion that slows down near BDC
                adjustment = BDC_ADJUSTMENT * ((piston_pos - 0.7) / 0.3)
                piston_pos = piston_pos * (1 - adjustment)
            
            # Store position and bank angle
            self.piston_positions[i] = (piston_pos, bank_angle)
            
    def update(self, dt):
        """Update engine state based on throttle and time step"""
        # Save last positions for TDC crossing detection
        self.last_piston_positions = self.piston_positions.copy()
        
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
            
        # Check if in slow mode (60 RPM)
        if self.slow_mode:
            self.target_rpm = 60
        else:
            # Normal operation - update RPM based on throttle
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
        
        # Smoothly transition to target RPM (faster in slow mode for immediate feedback)
        rpm_diff = self.target_rpm - self.rpm
        transition_rate = 10.0 if self.slow_mode else 3.0
        self.rpm += rpm_diff * min(transition_rate * dt, 1.0)
        
        # Ensure RPM stays within limits (allow below idle when in slow mode)
        min_rpm = 60 if self.slow_mode else (IDLE_RPM if self.running else 0)
        self.rpm = max(min_rpm, min(self.rpm, MAX_RPM))
        
        # Update crank angle based on RPM
        # RPM = revolutions per minute, so RPM/60 = revolutions per second
        # Each revolution is 360 degrees, so RPM/60*360 = degrees per second
        degrees_per_second = self.rpm / 60 * 360
        
        # Store previous crank angle for stroke detection
        prev_crank_angle = self.crank_angle
        
        # Update crank angle for 720° 4-stroke cycle
        self.crank_angle = (self.crank_angle + degrees_per_second * dt) % 720
        
        # Calculate piston positions
        self.calculate_piston_positions()
        
        # Update cylinder strokes based on crank angle
        self.update_cylinder_strokes(prev_crank_angle)
        
        # Calculate torque and horsepower (not applicable in slow mode)
        if not self.slow_mode:
            self.torque = float(torque_function(self.rpm))
            # HP = Torque * RPM / 5252
            self.horsepower = self.torque * self.rpm / 5252
            
            # Adjust for throttle
            throttle_factor = 0.2 + 0.8 * self.throttle
            self.torque *= throttle_factor
            self.horsepower *= throttle_factor
        else:
            # In slow mode, just set nominal values
            self.torque = 60
            self.horsepower = 5
    
    def update_cylinder_strokes(self, prev_crank_angle):
        """Update the stroke phase for each cylinder in the 4-stroke cycle"""
        for i in range(CYLINDERS):
            # Get cylinder's effective crank angle
            effective_angle = (self.crank_angle + self.crank_pin_angles[i]) % 720
            prev_effective_angle = (prev_crank_angle + self.crank_pin_angles[i]) % 720
            
            # Check for stroke transitions (0-180: intake, 180-360: compression, 
            # 360-540: power, 540-720: exhaust)
            
            # Intake stroke begins at TDC (0°)
            if prev_effective_angle > 720 - 5 and effective_angle < 5:
                self.cylinder_strokes[i] = 0  # Intake
            
            # Compression stroke begins at BDC after intake (180°)
            elif prev_effective_angle < 180 and effective_angle >= 180:
                self.cylinder_strokes[i] = 1  # Compression
            
            # Power stroke begins at TDC after compression (360°)
            elif prev_effective_angle < 360 and effective_angle >= 360:
                self.cylinder_strokes[i] = 2  # Power
            
            # Exhaust stroke begins at BDC after power (540°)
            elif prev_effective_angle < 540 and effective_angle >= 540:
                self.cylinder_strokes[i] = 3  # Exhaust
            
    def is_cylinder_at_tdc(self, cylinder_idx):
        """Check if a cylinder is at TDC and just crossed over"""
        current_pos, _ = self.piston_positions[cylinder_idx]
        last_pos, _ = self.last_piston_positions[cylinder_idx]
        
        # Define TDC threshold - position close to 0 is TDC
        TDC_THRESHOLD = 0.1
        
        # Check if piston just crossed into TDC zone
        if current_pos < TDC_THRESHOLD and last_pos > TDC_THRESHOLD:
            return True
        return False
    
    def is_cylinder_at_specific_angle(self, cylinder_idx, target_angle, threshold=5):
        """Check if cylinder is at a specific crank angle (in 720° cycle)"""
        # Get cylinder's effective crank angle
        effective_angle = (self.crank_angle + self.crank_pin_angles[cylinder_idx]) % 720
        
        # Check if within threshold of target angle
        angle_diff = abs((effective_angle - target_angle + 360) % 720 - 360)
        return angle_diff < threshold
    
    def get_cylinder_stroke(self, cylinder_idx):
        """Get the current stroke for a cylinder (0=intake, 1=compression, 2=power, 3=exhaust)"""
        return self.cylinder_strokes[cylinder_idx]
    
    def get_cylinder_stroke_position(self, cylinder_idx):
        """Get position within the current stroke (0.0 to 1.0)"""
        # Get cylinder's effective crank angle
        effective_angle = (self.crank_angle + self.crank_pin_angles[cylinder_idx]) % 720
        
        # Determine which stroke we're in and calculate position
        stroke = self.cylinder_strokes[cylinder_idx]
        
        # Calculate position within stroke (0.0 to 1.0)
        stroke_start_angle = stroke * 180
        position = (effective_angle - stroke_start_angle) / 180
        
        return min(1.0, max(0.0, position))  # Clamp to 0.0-1.0 range


class RealisticGTRSoundManager:
    def __init__(self, engine):
        self.engine = engine
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
        
        # Set enough channels for our needs
        pygame.mixer.set_num_channels(24)  # More channels for proper 4-stroke sounds
        
        # Create channels for different 4-stroke cycle sounds
        self.intake_sound_channels = [pygame.mixer.Channel(i) for i in range(6)]  # 6 cylinders - intake
        self.power_sound_channels = [pygame.mixer.Channel(i+6) for i in range(6)]  # 6 cylinders - power
        self.exhaust_sound_channels = [pygame.mixer.Channel(i+12) for i in range(6)]  # 6 cylinders - exhaust
        
        # Idle, turbo, and start channels
        self.idle_channel = pygame.mixer.Channel(18)
        self.turbo_channel = pygame.mixer.Channel(19)
        self.exhaust_main_channel = pygame.mixer.Channel(20)
        self.start_channel = pygame.mixer.Channel(21)
        
        # Create 4-stroke sounds for GTR VR38DETT
        # Each cylinder needs its own set of sounds for the strokes
        self.intake_sounds = []
        self.power_sounds = []
        self.exhaust_sounds = []
        
        # Create stroke sounds with variations for each cylinder
        for i in range(6):
            # Different variation for each cylinder
            self.intake_sounds.append(self.create_intake_sound(variation=i))
            self.power_sounds.append(self.create_power_sound(variation=i))
            self.exhaust_sounds.append(self.create_exhaust_sound(variation=i))
        
        # Create starting sound
        self.start_sound = self.create_engine_start_sound()
        
        # Create ambient engine sounds
        self.idle_sound = self.create_idle_sound()
        self.turbo_sound = self.create_turbo_sound()
        self.exhaust_main_sound = self.create_exhaust_main_sound()
        
        # Track last fire times for each cylinder and stroke
        self.last_intake_times = [0] * CYLINDERS
        self.last_power_times = [0] * CYLINDERS
        self.last_exhaust_times = [0] * CYLINDERS
        
        # Sound state tracking
        self.idle_playing = False
        self.turbo_playing = False
        self.exhaust_main_playing = False
        
        # GTR specific sound characteristics
        self.gtr_turbo_volume = 0.0  # Dynamically adjusted based on RPM/throttle
        self.gtr_exhaust_volume = 0.7
        
        # Previous cylinder strokes for transition detection
        self.prev_cylinder_strokes = [0] * CYLINDERS
        
    def create_intake_sound(self, variation=0):
        """Create intake stroke sound with turbo induction for GTR"""
        sample_rate = 44100
        duration = 0.3  # Shorter for higher RPMs
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate intake sound with realistic turbo induction
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base intake whoosh with turbo induction (twin-turbo for GTR)
        for i, time in enumerate(t):
            # Air rushing sound - more natural, less harsh
            buffer[i] = 0.3 * np.random.normal(0, 0.6) * np.exp(-time * 5)
            
            # FIXED: Lower turbo frequencies to prevent "alien sound" at high throttle
            # Reduced overall frequency and modulation depth
            turbo_freq = 800 + variation * 50 + 200 * np.sin(2 * np.pi * 4 * time)
            buffer[i] += 0.25 * np.sin(2 * np.pi * turbo_freq * time) * np.exp(-time * 4)
            
            # Add intake resonance
            intake_freq = 100 + variation * 10
            buffer[i] += 0.2 * np.sin(2 * np.pi * intake_freq * time) * np.exp(-time * 3)
        
        # Apply envelope for natural sound
        envelope = np.ones_like(buffer)
        attack = 0.02  # Fast attack
        attack_samples = int(attack * sample_rate)
        fade_out = 0.15  # Slightly longer fade
        fade_samples = int(fade_out * sample_rate)
        envelope[:attack_samples] = np.linspace(0, 1, attack_samples)
        envelope[-fade_samples:] = np.linspace(1, 0, fade_samples)
        buffer = buffer * envelope
        
        # FIXED: Additional filtering to remove harsh frequencies
        # Simple lowpass filtering
        filtered_buffer = np.zeros_like(buffer)
        filter_width = 15
        for i in range(len(buffer)):
            start = max(0, i - filter_width)
            end = min(len(buffer), i + filter_width + 1)
            filtered_buffer[i] = np.mean(buffer[start:end])
        
        # Normalize
        filtered_buffer = 0.6 * filtered_buffer / np.max(np.abs(filtered_buffer))
        
        # Convert to 16-bit PCM
        filtered_buffer = (filtered_buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((filtered_buffer, filtered_buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
    
    def create_power_sound(self, variation=0):
        """Create power stroke sound (combustion) with GTR characteristics"""
        sample_rate = 44100
        duration = 0.35  # Longer for more impact
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate combustion sound with GTR aggressive note
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base frequency varies slightly per cylinder for more realistic engine harmony
        base_freq = 75 + variation * 3  # Reduced variation between cylinders
        
        # REWRITTEN FOR MORE CONSISTENT POWER STROKE SOUND:
        # More focus on mechanical explosion rather than tonal components
        for i, time in enumerate(t):
            # Initial explosion transient - more noise-based, less tonal
            if time < 0.03:
                buffer[i] = 0.9 * np.random.normal(0, 1.0) * (1 - time/0.03)
            
            # Add low frequency rumble - constant, not modulated
            buffer[i] += 0.7 * np.sin(2 * np.pi * base_freq * time) * np.exp(-time * 12)
            
            # Add mid-range mechanical components - fixed frequencies
            buffer[i] += 0.5 * np.sin(2 * np.pi * base_freq * 2 * time) * np.exp(-time * 14)
            buffer[i] += 0.3 * np.sin(2 * np.pi * base_freq * 3 * time) * np.exp(-time * 16)
            buffer[i] += 0.2 * np.sin(2 * np.pi * base_freq * 4 * time) * np.exp(-time * 18)
            
            # Add some metallic components but with faster decay
            buffer[i] += 0.1 * np.sin(2 * np.pi * base_freq * 6 * time) * np.exp(-time * 24)
            buffer[i] += 0.05 * np.sin(2 * np.pi * base_freq * 8 * time) * np.exp(-time * 30)
        
        # Apply exponential decay envelope - fast decay for power stroke
        envelope = np.exp(-t * 8)
        buffer = buffer * envelope
        
        # Add random combustion variations for realism - smaller shorter variations
        for i in range(5):  # Reduced number of random variations
            pos = np.random.randint(0, len(buffer) // 8)  # Only in the beginning
            width = np.random.randint(5, 30)  # Shorter duration
            amp = np.random.random() * 0.05  # Lower amplitude
            buffer[pos:pos+width] += amp * np.random.normal(0, 1.0, min(width, len(buffer)-pos))
        
        # Apply low-pass filtering to smooth out high frequencies
        filtered_buffer = np.zeros_like(buffer)
        filter_width = 8  # Moderate filtering
        for i in range(len(buffer)):
            start = max(0, i - filter_width)
            end = min(len(buffer), i + filter_width + 1)
            filtered_buffer[i] = np.mean(buffer[start:end])
        
        # Normalize
        filtered_buffer = 0.8 * filtered_buffer / np.max(np.abs(filtered_buffer))
        
        # Convert to 16-bit PCM
        filtered_buffer = (filtered_buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((filtered_buffer, filtered_buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
    
    def create_exhaust_sound(self, variation=0):
        """Create exhaust stroke sound with GTR's characteristic note"""
        sample_rate = 44100
        duration = 0.4  # Longer for exhaust resonance
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate exhaust sound with GTR characteristics
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base frequency for exhaust note - varies by cylinder
        base_freq = 65 + variation * 5
        
        # GTR has a distinctive deep, aggressive exhaust note
        for i, time in enumerate(t):
            # Deep exhaust rumble
            buffer[i] = 0.5 * np.sin(2 * np.pi * base_freq * time) * np.exp(-time * 3)
            
            # Add higher harmonics for GTR's aggressive mid-range
            buffer[i] += 0.4 * np.sin(2 * np.pi * base_freq * 2 * time) * np.exp(-time * 4)
            buffer[i] += 0.3 * np.sin(2 * np.pi * base_freq * 3 * time) * np.exp(-time * 5)
            buffer[i] += 0.2 * np.sin(2 * np.pi * base_freq * 4 * time) * np.exp(-time * 6)
            
            # Add characteristic GTR exhaust crackle
            if time > 0.1 and np.random.random() > 0.95:
                buffer[i] += 0.25 * np.random.normal(0, 1.0)
        
        # Apply envelope for natural flow
        envelope = np.ones_like(buffer)
        attack = 0.01  # Quick attack
        attack_samples = int(attack * sample_rate)
        fade_out = 0.2  # Longer fade for resonance
        fade_samples = int(fade_out * sample_rate)
        envelope[:attack_samples] = np.linspace(0, 1, attack_samples)
        envelope[-fade_samples:] = np.linspace(1, 0, fade_samples)
        buffer = buffer * envelope
        
        # Normalize
        buffer = 0.8 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
    
    def create_turbo_sound(self):
        """Create continuous twin-turbo sound for the GTR"""
        sample_rate = 44100
        duration = 1.0  # Loop continuously
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate twin-turbo whistle sound
        t = np.linspace(0, duration, len(buffer), False)
        
        # COMPLETE REWRITE WITH NO MODULATION AT HIGH FREQUENCIES:
        # Create a more mechanical sound without wobbling/warbling effects
        for i, time in enumerate(t):
            # Base air noise - constant, not modulated
            buffer[i] = 0.2 * np.random.normal(0, 0.25)
            
            # Low rumble for turbo spooling - minimal modulation
            base_freq = 120
            buffer[i] += 0.25 * np.sin(2 * np.pi * base_freq * time)
            
            # Mid-range mechanical component - no modulation
            mid_freq = 350
            buffer[i] += 0.15 * np.sin(2 * np.pi * mid_freq * time)
            
            # Higher turbo tone - very minimal modulation
            high_freq = 650
            slow_mod = 1 + 0.05 * np.sin(2 * np.pi * 1 * time)  # Very slow, subtle modulation
            buffer[i] += 0.1 * np.sin(2 * np.pi * high_freq * time * slow_mod)
            
            # Add some metal/mechanical harmonics - no modulation
            buffer[i] += 0.05 * np.sin(2 * np.pi * 220 * time)
            buffer[i] += 0.03 * np.sin(2 * np.pi * 440 * time)
        
        # No overall modulation that could cause wobbling
        
        # Apply smooth envelope for looping
        envelope = np.ones_like(buffer)
        transition = 0.1  # seconds
        transition_samples = int(transition * sample_rate)
        envelope[:transition_samples] = np.linspace(0, 1, transition_samples)
        envelope[-transition_samples:] = np.linspace(1, 0, transition_samples)
        buffer = buffer * envelope
        
        # Apply very aggressive low-pass filtering to remove all high frequencies
        filtered_buffer = np.zeros_like(buffer)
        filter_width = 40  # Very wide filter = very aggressive low-pass
        for i in range(len(buffer)):
            start = max(0, i - filter_width)
            end = min(len(buffer), i + filter_width + 1)
            filtered_buffer[i] = np.mean(buffer[start:end])
        
        # Apply a second filtering pass to completely smooth it out
        final_buffer = np.zeros_like(filtered_buffer)
        filter_width = 20
        for i in range(len(filtered_buffer)):
            start = max(0, i - filter_width)
            end = min(len(filtered_buffer), i + filter_width + 1)
            final_buffer[i] = np.mean(filtered_buffer[start:end])
        
        # Normalize with lower amplitude
        final_buffer = 0.5 * final_buffer / np.max(np.abs(final_buffer))
        
        # Convert to 16-bit PCM
        final_buffer = (final_buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((final_buffer, final_buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
    
    def create_exhaust_main_sound(self):
        """Create main exhaust note sound for overall engine sound"""
        sample_rate = 44100
        duration = 1.0  # Loop continuously
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate GTR exhaust note
        t = np.linspace(0, duration, len(buffer), False)
        
        # GTR's exhaust has a deep, aggressive note
        base_freq = 55  # Hz - fundamental exhaust frequency
        
        # Create rich harmonic series for GTR exhaust
        for i, time in enumerate(t):
            # Fundamental
            buffer[i] = 0.5 * np.sin(2 * np.pi * base_freq * time)
            
            # Rich harmonics for depth
            for harm in range(2, 10):
                amp = 0.5 / harm
                buffer[i] += amp * np.sin(2 * np.pi * base_freq * harm * time)
            
            # Add subtle variation over time
            mod_freq = 3  # Hz
            variation = 0.1 * np.sin(2 * np.pi * mod_freq * time)
            buffer[i] *= (1 + variation)
            
            # Add occasional exhaust pops characteristic of GTR
            if np.random.random() > 0.995:
                buffer[i] += 0.3 * np.random.normal(0, 1.0)
        
        # Apply smooth envelope for looping
        envelope = np.ones_like(buffer)
        transition = 0.1  # seconds
        transition_samples = int(transition * sample_rate)
        envelope[:transition_samples] = np.linspace(0, 1, transition_samples)
        envelope[-transition_samples:] = np.linspace(1, 0, transition_samples)
        buffer = buffer * envelope
        
        # Normalize
        buffer = 0.7 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
    
    def create_engine_start_sound(self):
        """Create GTR-like engine starting sound with 4-stroke characteristics"""
        sample_rate = 44100
        duration = 2.5  # seconds - proper duration for GTR start
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate a starting sound - initial cranking followed by aggressive start
        t = np.linspace(0, duration, len(buffer), False)
        
        # Initial cranking phase
        cranking_freq = 15  # Hz - slow cranking
        for i, time in enumerate(t):
            if time < 1.0:  # Cranking phase
                # Slow cranking motor with mechanical sound
                buffer[i] = 0.5 * np.sin(2 * np.pi * cranking_freq * time)
                
                # Add mechanical starter motor sounds
                buffer[i] += 0.4 * np.random.normal(0, 0.5)
                
                # Simulate individual cylinder compressions
                for cyl in range(6):
                    crank_cyl = (cranking_freq * time * 2 + cyl/6) % 1
                    if crank_cyl < 0.1:
                        comp_factor = 1 - (crank_cyl / 0.1)
                        buffer[i] += 0.2 * comp_factor * np.sin(2 * np.pi * 50 * time)
                
                # Motor sound increases
                buffer[i] *= (0.3 + 0.7 * time)
                
            elif time < 1.5:  # GTR fires up aggressively
                # Transition from cranking to firing
                transition_factor = (time - 1.0) * 2
                
                # First few cylinder firings - uneven as engine starts
                engine_freq = 70 + 250 * transition_factor
                
                # Simulate cylinders firing unevenly during startup
                for cyl in range(6):
                    cyl_offset = cyl / 6
                    cyl_time = time + cyl_offset
                    if cyl_time % 0.2 < 0.05:  # Cylinder firing
                        fire_factor = 1 - (cyl_time % 0.2) / 0.05
                        buffer[i] += 0.7 * fire_factor * np.random.normal(0, 1.0)
                        
                # Add base engine sound with harmonics
                buffer[i] += 0.5 * np.sin(2 * np.pi * engine_freq * time)
                buffer[i] += 0.4 * np.sin(2 * np.pi * engine_freq * 2 * time)
                buffer[i] += 0.3 * np.sin(2 * np.pi * engine_freq * 3 * time)
                
                # Add turbo spool sound as engine starts
                turbo_freq = 1500 * transition_factor
                buffer[i] += 0.2 * transition_factor * np.sin(2 * np.pi * turbo_freq * time)
                
            else:  # Settle to idle with GTR characteristics
                # Engine settles with slight rev
                decay_factor = min(1.0, (time - 1.5) * 2)
                rev_freq = 320 - 150 * decay_factor
                
                # Main engine harmonics
                buffer[i] = 0.6 * np.sin(2 * np.pi * rev_freq * time)
                buffer[i] += 0.4 * np.sin(2 * np.pi * rev_freq * 2 * time)
                buffer[i] += 0.3 * np.sin(2 * np.pi * rev_freq * 3 * time)
                buffer[i] += 0.2 * np.sin(2 * np.pi * rev_freq * 4 * time)
                
                # Add GTR's characteristic exhaust note as it settles
                exhaust_freq = 80
                buffer[i] += 0.5 * (1 - decay_factor) * np.sin(2 * np.pi * exhaust_freq * time)
                
                # Add decreasing mechanical noise
                buffer[i] += 0.2 * np.random.normal(0, max(0.1, 0.6 - 0.5 * decay_factor))
                
                # Add subtle twin-turbo whine
                turbo_freq = 2000
                buffer[i] += 0.15 * decay_factor * np.sin(2 * np.pi * turbo_freq * time)
        
        # Apply volume envelope for entire sound
        envelope = np.ones_like(buffer)
        fade_out = 0.2  # seconds
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
        """Create an idle engine sound with GTR's deep aggressive character"""
        sample_rate = 44100
        duration = 1.0  # seconds
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate an idle sound with GTR characteristics
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base frequency for idle - deeper for GTR V6
        idle_freq = 30  # Hz
        
        # Create a complex waveform with GTR idle character
        for i, time in enumerate(t):
            # Base deep rumble
            buffer[i] = 0.6 * np.sin(2 * np.pi * idle_freq * time)
            
            # Add rich harmonics for GTR character
            buffer[i] += 0.5 * np.sin(2 * np.pi * idle_freq * 2 * time)
            buffer[i] += 0.35 * np.sin(2 * np.pi * idle_freq * 3 * time)
            buffer[i] += 0.25 * np.sin(2 * np.pi * idle_freq * 4 * time)
            buffer[i] += 0.15 * np.sin(2 * np.pi * idle_freq * 6 * time)
            buffer[i] += 0.1 * np.sin(2 * np.pi * idle_freq * 8 * time)
            
            # Add GTR's uneven lope at idle
            lope = 0.1 * np.sin(2 * np.pi * 6 * time) + 0.05 * np.sin(2 * np.pi * 12 * time)
            buffer[i] *= (1.0 + lope)
            
            # Add subtle vibration harmonics
            vibration = 0.05 * np.sin(2 * np.pi * 50 * time) + 0.03 * np.sin(2 * np.pi * 75 * time)
            buffer[i] += vibration
            
            # Add some noise for texture
            buffer[i] += 0.15 * np.random.normal(0, 0.3)
        
        # Apply smooth envelope for looping
        envelope = np.ones_like(buffer)
        transition = 0.1  # seconds
        transition_samples = int(transition * sample_rate)
        envelope[:transition_samples] = np.linspace(0, 1, transition_samples)
        envelope[-transition_samples:] = np.linspace(1, 0, transition_samples)
        buffer = buffer * envelope
        
        # Normalize
        buffer = 0.7 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
    
    def play_start_sound(self):
        """Play the engine start sound"""
        self.start_channel.play(self.start_sound)
    
    def play_intake_sound(self, cylinder_idx, volume=0.7):
        """Play intake stroke sound for a specific cylinder"""
        self.intake_sound_channels[cylinder_idx].play(self.intake_sounds[cylinder_idx])
        self.intake_sound_channels[cylinder_idx].set_volume(volume)
        self.last_intake_times[cylinder_idx] = time.time()
    
    def play_power_sound(self, cylinder_idx, volume=0.8):
        """Play power stroke sound for a specific cylinder"""
        self.power_sound_channels[cylinder_idx].play(self.power_sounds[cylinder_idx])
        self.power_sound_channels[cylinder_idx].set_volume(volume)
        self.last_power_times[cylinder_idx] = time.time()
    
    def play_exhaust_sound(self, cylinder_idx, volume=0.7):
        """Play exhaust stroke sound for a specific cylinder"""
        self.exhaust_sound_channels[cylinder_idx].play(self.exhaust_sounds[cylinder_idx])
        self.exhaust_sound_channels[cylinder_idx].set_volume(volume)
        self.last_exhaust_times[cylinder_idx] = time.time()
    
    def update(self):
        """Update sound based on engine state, RPM, and 4-stroke cycle"""
        # Handle engine starting/stopping
        if self.engine.starting and not self.start_channel.get_busy():
            self.play_start_sound()
            
        # If engine is not running or starting, stop all sounds
        if not self.engine.running and not self.engine.starting:
            # Fade out all sounds
            for channel in self.intake_sound_channels + self.power_sound_channels + self.exhaust_sound_channels:
                if channel.get_busy():
                    channel.fadeout(300)
            
            if self.idle_playing:
                self.idle_channel.fadeout(500)
                self.idle_playing = False
            
            if self.turbo_playing:
                self.turbo_channel.fadeout(300)
                self.turbo_playing = False
                
            if self.exhaust_main_playing:
                self.exhaust_main_channel.fadeout(500)
                self.exhaust_main_playing = False
                
            return
        
        # Play idle sound if engine is running
        if self.engine.running and not self.idle_playing:
            self.idle_channel.play(self.idle_sound, loops=-1)
            self.idle_playing = True
            
            # Also start continuous exhaust note
            self.exhaust_main_channel.play(self.exhaust_main_sound, loops=-1)
            self.exhaust_main_playing = True
        
        # SPECIAL HANDLING FOR HIGH THROTTLE (ABOVE 60%)
        # Determine throttle band with special high throttle band
        low_throttle = self.engine.throttle <= 0.2
        mid_throttle = 0.2 < self.engine.throttle <= 0.6
        high_throttle = self.engine.throttle > 0.6
        
        # Handle turbo sounds
        if self.engine.rpm > 1500:
            # Start turbo sound if not playing
            if not self.turbo_playing:
                self.turbo_channel.play(self.turbo_sound, loops=-1)
                self.turbo_playing = True
            
            # Calculate base volume from RPM
            rpm_factor = min(0.6, (self.engine.rpm - 1500) / (MAX_RPM - 1500))
            
            # Apply throttle-specific volume adjustments with FIXED HIGH THROTTLE VALUES
            if high_throttle:
                # For high throttle, use CONSTANT volume instead of scaling with throttle
                # This prevents the wobbling metal sheet effect by maintaining consistent sound
                turbo_volume = 0.25  # Fixed value
            elif mid_throttle:
                # For mid throttle, use gentle scaling
                throttle_factor = (self.engine.throttle - 0.2) / 0.4  # Normalize to 0-1 range
                turbo_volume = 0.1 + rpm_factor * 0.2 * (0.5 + 0.5 * throttle_factor)
            else:
                turbo_volume = 0.05 + rpm_factor * 0.1
                
            # Apply final volume 
            self.turbo_channel.set_volume(turbo_volume)
        elif self.turbo_playing:
            self.turbo_channel.fadeout(300)
            self.turbo_playing = False
        
        # Adjust ambient sound characteristics
        if self.idle_playing:
            # Idle volume always decreases with RPM
            idle_volume = max(0.1, 0.6 - (self.engine.rpm / MAX_RPM) * 0.5)
            self.idle_channel.set_volume(idle_volume)
            
            # Exhaust volume handling with special high throttle case
            rpm_factor = self.engine.rpm / MAX_RPM
            
            if high_throttle:
                # At high throttle, use FIXED volume to prevent wobbling
                exhaust_volume = 0.6  # Consistent volume
            elif mid_throttle:
                # At mid throttle, scale more gently
                throttle_factor = (self.engine.throttle - 0.2) / 0.4  # Normalize to 0-1
                exhaust_volume = 0.3 + rpm_factor * 0.3 * (0.7 + 0.3 * throttle_factor)
            else:
                exhaust_volume = 0.2 + rpm_factor * 0.3
                
            self.exhaust_main_channel.set_volume(min(0.7, exhaust_volume * self.gtr_exhaust_volume))
        
        # Skip detailed stroke sounds in slow mode for clarity
        if self.engine.slow_mode:
            return
        
        # Get current time for sound spacing
        current_time = time.time()
        
        # Play individual cylinder stroke sounds based on 4-stroke cycle
        for cyl_idx in range(CYLINDERS):
            # Get current stroke for this cylinder
            current_stroke = self.engine.get_cylinder_stroke(cyl_idx)
            
            # Check if stroke changed since last update
            if current_stroke != self.prev_cylinder_strokes[cyl_idx]:
                # Calculate throttle-appropriate volumes with SPECIAL HIGH THROTTLE HANDLING
                rpm_factor = min(0.8, self.engine.rpm / MAX_RPM)
                
                # Intake stroke volumes
                if current_stroke == 0 and current_time - self.last_intake_times[cyl_idx] > 0.1:
                    if high_throttle:
                        # Use FIXED volume at high throttle
                        intake_volume = 0.2
                    elif mid_throttle:
                        throttle_factor = (self.engine.throttle - 0.2) / 0.4
                        intake_volume = 0.2 + 0.2 * rpm_factor * throttle_factor
                    else:
                        intake_volume = 0.2 + 0.2 * rpm_factor * self.engine.throttle
                        
                    self.play_intake_sound(cyl_idx, min(0.4, intake_volume))
                
                # Power stroke volumes
                elif current_stroke == 2 and current_time - self.last_power_times[cyl_idx] > 0.1:
                    if high_throttle:
                        # Use FIXED volume at high throttle
                        power_volume = 0.5
                    elif mid_throttle:
                        throttle_factor = (self.engine.throttle - 0.2) / 0.4
                        power_volume = 0.3 + 0.2 * rpm_factor * throttle_factor
                    else:
                        power_volume = 0.2 + 0.2 * rpm_factor * self.engine.throttle
                        
                    self.play_power_sound(cyl_idx, min(0.6, power_volume))
                
                # Exhaust stroke volumes
                elif current_stroke == 3 and current_time - self.last_exhaust_times[cyl_idx] > 0.1:
                    if high_throttle:
                        # Use FIXED volume at high throttle
                        exhaust_volume = 0.3
                    elif mid_throttle:
                        throttle_factor = (self.engine.throttle - 0.2) / 0.4
                        exhaust_volume = 0.2 + 0.2 * rpm_factor * throttle_factor
                    else:
                        exhaust_volume = 0.2 + 0.2 * rpm_factor * self.engine.throttle
                        
                    self.play_exhaust_sound(cyl_idx, min(0.4, exhaust_volume))
            
            # Update previous stroke state
            self.prev_cylinder_strokes[cyl_idx] = current_stroke


class VariableEngineSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Variable Engine Simulator - Nissan GTR VR38DETT")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 20)
        self.title_font = pygame.font.SysFont('Arial', 24, bold=True)
        
        # FPS counter variables
        self.fps = 0
        self.frame_count = 0
        self.fps_update_time = time.time()
        
        # Create engine
        self.engine = Engine()
        
        # Engine sound manager
        self.sound_manager = RealisticGTRSoundManager(self.engine)
        
        # Engine rendering parameters
        self.center_x = WIDTH // 2
        self.center_y = HEIGHT // 2
        
        # Throttle UI
        self.throttle_rect = pygame.Rect(WIDTH - 60, HEIGHT // 2 - 150, 30, 300)
        self.throttle_handle_rect = pygame.Rect(WIDTH - 70, HEIGHT // 2, 50, 20)
        self.dragging_throttle = False
        
        # Engine Angle Slider
        self.angle_rect = pygame.Rect(WIDTH // 2 - 150, HEIGHT - 50, 300, 20)
        self.angle_handle_rect = pygame.Rect(WIDTH // 2 - 5 + (self.engine.v_angle / 180 * 300), HEIGHT - 60, 10, 40)
        self.dragging_angle = False
        
        # Start/Stop button
        self.start_button_rect = pygame.Rect(WIDTH - 120, HEIGHT - 80, 100, 40)
        
        # 60RPM button
        self.rpm60_button_rect = pygame.Rect(WIDTH - 240, HEIGHT - 80, 100, 40)
        
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
            f"V-Angle: {self.engine.v_angle:.1f}°",
            f"Max RPM: {MAX_RPM}",
            f"Redline: {REDLINE_RPM}"
        ]
        
        y_pos = 60
        for spec in specs:
            spec_surface = self.font.render(spec, True, (200, 200, 200))
            self.screen.blit(spec_surface, (20, y_pos))
            y_pos += 25
        
        # Draw engine configuration text based on angle
        config_text = ""
        if self.engine.v_angle < 10:
            config_text = "Inline Engine"
        elif self.engine.v_angle < 90:
            config_text = "V Engine"
        elif self.engine.v_angle < 170:
            config_text = "Wide V Engine"
        else:
            config_text = "Boxer/Flat Engine"
            
        config_surface = self.font.render(config_text, True, (220, 220, 220))
        self.screen.blit(config_surface, (20, y_pos))
        
        # Draw crankshaft
        crank_x = self.center_x
        crank_y = self.center_y + 80
        
        # Draw the crankshaft and its components
        self.draw_crankshaft(crank_x, crank_y)
        
        # Draw cylinders and pistons together
        self.draw_cylinders_and_pistons(crank_x, crank_y)
        
        # Draw RPM gauge
        self.draw_gauge(150, HEIGHT - 100, 100, "RPM", self.engine.rpm, 0, MAX_RPM)
        
        # Draw torque gauge
        self.draw_gauge(350, HEIGHT - 100, 100, "Torque (Nm)", self.engine.torque, 0, 600)
        
        # Draw HP gauge
        self.draw_gauge(550, HEIGHT - 100, 100, "Horsepower", self.engine.horsepower, 0, 600)
        
        # Draw throttle control
        self.draw_throttle()
        
        # Draw angle slider
        self.draw_angle_slider()
        
        # Draw Start/Stop button
        self.draw_start_button()
        
        # Draw 60RPM button
        self.draw_rpm60_button()
        
        # Draw FPS counter
        self.draw_fps_counter()
        
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
        
        # Draw 4-stroke indicators for debugging (optional)
        if self.engine.running:
            stroke_names = ["Intake", "Compression", "Power", "Exhaust"]
            for i in range(CYLINDERS):
                stroke = self.engine.get_cylinder_stroke(i)
                stroke_text = self.font.render(f"Cyl {i+1}: {stroke_names[stroke]}", True, (180, 180, 180))
                self.screen.blit(stroke_text, (WIDTH - 180, 100 + i * 25))
        
    def draw_fps_counter(self):
        """Draw the FPS counter in the top-right corner"""
        # Calculate and update FPS
        current_time = time.time()
        self.frame_count += 1
        
        if current_time - self.fps_update_time >= 0.5:  # Update FPS every half second
            self.fps = self.frame_count / (current_time - self.fps_update_time)
            self.fps_update_time = current_time
            self.frame_count = 0
        
        # Draw FPS counter
        fps_surface = self.font.render(f"FPS: {int(self.fps)}", True, (255, 255, 255))
        self.screen.blit(fps_surface, (WIDTH - fps_surface.get_width() - 10, 10))
        
    def draw_crankshaft(self, crank_x, crank_y):
        """Draw the crankshaft with rotating components"""
        # Draw main crankshaft centerpiece
        pygame.draw.circle(self.screen, CRANKSHAFT_COLOR, (crank_x, crank_y), 35)
        pygame.draw.circle(self.screen, (120, 120, 120), (crank_x, crank_y), 25)
        
        # Draw a smaller center circle
        pygame.draw.circle(self.screen, (180, 180, 180), (crank_x, crank_y), 10)
    
    def draw_cylinders_and_pistons(self, crank_x, crank_y):
        """Draw cylinders and pistons together with proper visual layering"""
        # We'll draw from back to front for proper layering
        
        # Calculate bank angles
        left_bank_angle = -self.engine.v_angle / 2
        right_bank_angle = self.engine.v_angle / 2
        
        # Define left and right bank cylinders
        left_bank_indices = [0, 2, 4]  # Cylinders 1, 3, 5
        right_bank_indices = [1, 3, 5]  # Cylinders 2, 4, 6
        
        # Draw the cylinder banks and pistons
        self.draw_bank(crank_x, crank_y, left_bank_angle, left_bank_indices)
        self.draw_bank(crank_x, crank_y, right_bank_angle, right_bank_indices)
    
    def draw_bank(self, crank_x, crank_y, bank_angle, cylinder_indices):
        """Draw a complete bank of cylinders with pistons inside"""
        # Direction vector along the bank
        bank_rad = math.radians(bank_angle)
        dir_x = math.sin(bank_rad)
        dir_y = -math.cos(bank_rad)
        
        # Perpendicular direction for width
        perp_x = math.cos(bank_rad + math.pi/2)
        perp_y = math.sin(bank_rad + math.pi/2)
        
        # Draw cylinders in this bank from back to front
        for i, cyl_idx in reversed(list(enumerate(cylinder_indices))):
            # Calculate transparency for depth perception
            alpha = 255 - i * 80
            
            # Get piston position
            piston_pos, _ = self.engine.piston_positions[cyl_idx]
            
            # Get crank pin position
            pin_angle = self.engine.crank_pin_angles[cyl_idx]
            pin_rad = math.radians((self.engine.crank_angle + pin_angle) % 360)
            pin_x = crank_x + self.engine.crank_radius * math.cos(pin_rad)
            pin_y = crank_y - self.engine.crank_radius * math.sin(pin_rad)
            
            # Calculate cylinder base position
            cylinder_base_x = crank_x + self.engine.cylinder_offset * dir_x
            cylinder_base_y = crank_y + self.engine.cylinder_offset * dir_y
            
            # Calculate cylinder dimensions
            cylinder_height = 200
            cylinder_width = 70
            cylinder_spacing = 0
            wall_thickness = 10  # Thicker walls for better visibility
            
            # Calculate position along the bank
            pos_along_bank = i * cylinder_spacing
            cyl_center_x = cylinder_base_x + pos_along_bank * perp_x
            cyl_center_y = cylinder_base_y + pos_along_bank * perp_y
            
            # Calculate top of cylinder
            cyl_top_x = cyl_center_x + cylinder_height * dir_x
            cyl_top_y = cyl_center_y + cylinder_height * dir_y
            
            # Create a surface for this cylinder with transparency
            cylinder_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            
            # COMPLETELY REWRITTEN CYLINDER DRAWING - No outlines, only solid fills
            
            # Calculate outer points (full cylinder with walls)
            half_width = cylinder_width / 2
            outer_points = [
                (cyl_center_x - half_width * perp_x, cyl_center_y - half_width * perp_y),
                (cyl_center_x + half_width * perp_x, cyl_center_y + half_width * perp_y),
                (cyl_top_x + half_width * perp_x, cyl_top_y + half_width * perp_y),
                (cyl_top_x - half_width * perp_x, cyl_top_y - half_width * perp_y)
            ]
            
            # Calculate inner points (cylinder interior only)
            inner_half_width = half_width - wall_thickness
            inner_points = [
                (cyl_center_x - inner_half_width * perp_x, cyl_center_y - inner_half_width * perp_y),
                (cyl_center_x + inner_half_width * perp_x, cyl_center_y + inner_half_width * perp_y),
                (cyl_top_x + inner_half_width * perp_x, cyl_top_y + inner_half_width * perp_y),
                (cyl_top_x - inner_half_width * perp_x, cyl_top_y - inner_half_width * perp_y)
            ]
            
            # Draw outer shape first (cylinder walls) - solid pink
            wall_color = list(CYLINDER_WALL_COLOR) + [alpha]
            pygame.draw.polygon(cylinder_surface, wall_color, outer_points, 0)
            
            # Draw inner shape on top (cylinder interior) - solid gray
            cylinder_color = list(CYLINDER_COLOR) + [alpha]
            pygame.draw.polygon(cylinder_surface, cylinder_color, inner_points, 0)
            
            # Calculate piston position within the cylinder
            piston_travel = cylinder_height * 0.7
            piston_x = cyl_center_x + piston_travel * piston_pos * dir_x
            piston_y = cyl_center_y + piston_travel * piston_pos * dir_y
            
            # Draw connecting rod
            rod_color = list(CONNECTING_ROD_COLOR) + [alpha]
            pygame.draw.line(cylinder_surface, rod_color, (pin_x, pin_y), (piston_x, piston_y), 8)
            
            # Draw piston (smaller than inner cylinder)
            piston_width = cylinder_width - (wall_thickness * 2 + 5)
            piston_height = 40
            
            # Calculate piston points
            piston_points = self.get_rotated_rect(
                piston_x, piston_y,
                piston_width, piston_height,
                math.degrees(bank_rad)
            )
            
            # Draw piston as white filled polygon
            piston_color = list(PISTON_COLOR) + [alpha]
            pygame.draw.polygon(cylinder_surface, piston_color, piston_points, 0)
            
            # Add piston pin as black dot
            pin_color = list(PIN_COLOR) + [alpha]
            pygame.draw.circle(cylinder_surface, pin_color, (int(piston_x), int(piston_y)), 5)
            
            # Blit the surface to the screen
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
    
    def draw_angle_slider(self):
        """Draw slider to adjust engine configuration angle"""
        # Draw slider background
        pygame.draw.rect(self.screen, (50, 50, 50), self.angle_rect)
        pygame.draw.rect(self.screen, (200, 200, 200), self.angle_rect, 2)
        
        # Draw tick marks for important angles
        for angle in [0, 60, 120, 180]:  # Inline, V, wide V, boxer
            x_pos = self.angle_rect.x + (angle / 180) * self.angle_rect.width
            pygame.draw.line(self.screen, (200, 200, 200), 
                           (x_pos, self.angle_rect.y - 5), 
                           (x_pos, self.angle_rect.y + self.angle_rect.height + 5), 
                           2)
            
            # Draw tick labels
            label = f"{angle}°"
            label_surface = self.font.render(label, True, (180, 180, 180))
            self.screen.blit(label_surface, (x_pos - label_surface.get_width() // 2, 
                                            self.angle_rect.y + self.angle_rect.height + 10))
        
        # Draw labels for engine types
        config_labels = [
            ("Inline", 0),
            ("V Engine", 60),
            ("Wide V", 120),
            ("Boxer", 180)
        ]
        
        for label, angle in config_labels:
            x_pos = self.angle_rect.x + (angle / 180) * self.angle_rect.width
            label_surface = self.font.render(label, True, (180, 180, 180))
            self.screen.blit(label_surface, (x_pos - label_surface.get_width() // 2, 
                                            self.angle_rect.y - 25))
        
        # Draw handle
        handle_x = self.angle_rect.x + (self.engine.v_angle / 180) * self.angle_rect.width
        self.angle_handle_rect.x = handle_x - self.angle_handle_rect.width // 2
        pygame.draw.rect(self.screen, (150, 150, 150), self.angle_handle_rect)
        pygame.draw.rect(self.screen, (200, 200, 200), self.angle_handle_rect, 2)
        
        # Draw current angle text
        angle_text = f"Engine Angle: {self.engine.v_angle:.1f}°"
        angle_surface = self.font.render(angle_text, True, (255, 255, 255))
        self.screen.blit(angle_surface, (self.angle_rect.x + self.angle_rect.width // 2 - angle_surface.get_width() // 2, 
                                        self.angle_rect.y - 50))
        
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
        
    def draw_rpm60_button(self):
        """Draw the 60RPM button for slow visualization"""
        # Button is highlighted when active
        if self.engine.slow_mode:
            button_color = (0, 100, 255)  # Blue when active
        else:
            button_color = (70, 70, 100)  # Dark bluish-gray when inactive
            
        # Draw button - rounded rectangle
        pygame.draw.rect(self.screen, button_color, self.rpm60_button_rect, 0, 5)
        pygame.draw.rect(self.screen, (200, 200, 200), self.rpm60_button_rect, 2, 5)
        
        # Draw button text - white and centered
        text_surface = self.font.render("60 RPM", True, (255, 255, 255))
        text_x = self.rpm60_button_rect.x + (self.rpm60_button_rect.width - text_surface.get_width()) // 2
        text_y = self.rpm60_button_rect.y + (self.rpm60_button_rect.height - text_surface.get_height()) // 2
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
                # Check if clicked on 60RPM button
                elif self.rpm60_button_rect.collidepoint(event.pos):
                    # Toggle slow mode
                    self.engine.set_slow_mode(not self.engine.slow_mode)
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
                # Check if clicked on angle slider
                elif self.angle_handle_rect.collidepoint(event.pos):
                    self.dragging_angle = True
                elif self.angle_rect.collidepoint(event.pos):
                    # Direct click on angle bar - set angle directly
                    mouse_x = event.pos[0]
                    angle_pos = (mouse_x - self.angle_rect.x) / self.angle_rect.width
                    new_angle = angle_pos * 180
                    self.engine.set_v_angle(max(0.0, min(180.0, new_angle)))
                    # Update handle position
                    self.angle_handle_rect.x = self.angle_rect.x + (self.engine.v_angle / 180) * self.angle_rect.width - self.angle_handle_rect.width // 2
                    
            elif event.type == MOUSEBUTTONUP:
                self.dragging_throttle = False
                self.dragging_angle = False
                
            elif event.type == MOUSEMOTION:
                if self.dragging_throttle:
                    # Update throttle based on mouse position
                    mouse_y = event.pos[1]
                    throttle_pos = (self.throttle_rect.y + self.throttle_rect.height - mouse_y) / self.throttle_rect.height
                    self.engine.throttle = max(0.0, min(1.0, throttle_pos))
                elif self.dragging_angle:
                    # Update angle based on mouse position
                    mouse_x = event.pos[0]
                    angle_pos = (mouse_x - self.angle_rect.x) / self.angle_rect.width
                    new_angle = angle_pos * 180
                    self.engine.set_v_angle(max(0.0, min(180.0, new_angle)))
                
            elif event.type == KEYDOWN:
                if event.key == K_UP:
                    self.engine.throttle = min(1.0, self.engine.throttle + 0.1)
                elif event.key == K_DOWN:
                    self.engine.throttle = max(0.0, self.engine.throttle - 0.1)
                elif event.key == K_LEFT:
                    # Decrease engine angle
                    self.engine.set_v_angle(max(0.0, self.engine.v_angle - 5.0))
                elif event.key == K_RIGHT:
                    # Increase engine angle
                    self.engine.set_v_angle(min(180.0, self.engine.v_angle + 5.0))
                elif event.key == K_SPACE:
                    # Space bar toggles engine state
                    if self.engine.running:
                        self.engine.stop_engine()
                    else:
                        self.engine.start_engine()
                elif event.key == K_s:
                    # 'S' key toggles slow mode (60 RPM)
                    self.engine.set_slow_mode(not self.engine.slow_mode)
                    
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
    simulator = VariableEngineSimulator()
    simulator.run()