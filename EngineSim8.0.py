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
FIRING_ORDER = [1, 4, 3, 6, 2, 5]  # V6 firing order
IDLE_RPM = 750
MAX_RPM = 7100
REDLINE_RPM = 6800
DIAGNOSTIC_RPM = 60  # Fixed RPM for diagnostic mode

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
CYLINDER_OFFSET = 50  # Distance from crank center to cylinder base

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
        self.diagnostic_mode = False  # Diagnostic mode (fixed 60 RPM)
        
        # Engine geometry parameters
        self.rod_length = ROD_LENGTH  # Fixed rod length
        self.crank_radius = CRANK_RADIUS
        self.cylinder_offset = CYLINDER_OFFSET  # Distance from crank center to cylinder base
        
        # Variable engine configuration
        self.v_angle = DEFAULT_V_ANGLE  # Can be changed from 0 (inline) to 180 (boxer)
        
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
        
    def set_v_angle(self, angle):
        """Set the V-angle of the engine (0 for inline, 60 for V6, 180 for boxer)"""
        self.v_angle = angle
        
    def toggle_diagnostic_mode(self):
        """Toggle diagnostic mode (fixed 60 RPM)"""
        self.diagnostic_mode = not self.diagnostic_mode
        
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
        
        # Check if diagnostic mode is active
        if self.diagnostic_mode:
            # Fixed RPM mode - directly set RPM to diagnostic value
            self.rpm = DIAGNOSTIC_RPM
            self.target_rpm = DIAGNOSTIC_RPM
        else:
            # Normal mode - update RPM based on throttle
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


class RealisticGTRSoundManager:
    def __init__(self, engine):
        self.engine = engine
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
        
        # Important: Set enough channels for our needs
        pygame.mixer.set_num_channels(16)  # More channels for more realistic overlapping
        
        # Create channels for different sounds
        self.explosion_channels = [pygame.mixer.Channel(i) for i in range(12)]  # 12 channels for explosions
        self.channel_index = 0
        
        # Idle sound channel and start sound channel
        self.idle_channel = pygame.mixer.Channel(12)
        self.intake_channel = pygame.mixer.Channel(13)
        self.exhaust_channel = pygame.mixer.Channel(14)
        self.start_channel = pygame.mixer.Channel(15)
        
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
        
        # Create intake and exhaust sounds for more realism
        self.intake_sound = self.create_intake_sound()
        self.exhaust_sound = self.create_exhaust_sound()
        
        # Track last fire time to manage sound frequency
        self.last_fire_time = 0
        self.last_cylinder_fired = -1
        
        # Idle sound
        self.idle_sound = self.create_idle_sound()
        self.idle_playing = False
        self.intake_playing = False
        self.exhaust_playing = False
        
        # For precise timing of explosion sounds
        self.last_crank_angle = 0
        
        # GTR specific sound characteristics
        # - VR38DETT has a distinctive aggressive growl
        # - High-pitched whine from twin turbos
        # - Deep, bassy idle
        self.gtr_exhaust_volume = 0.7
        self.gtr_turbo_whine = 0.4
        
    def create_fallback_explosion_sound(self):
        """Create a synthesized explosion sound if the WAV file is not found"""
        sample_rate = 44100
        duration = 0.1  # seconds - shorter for faster playback
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate a percussive sound with GTR characteristics
        # - More aggressive with higher frequency components
        # - Sharper attack
        t = np.linspace(0, duration, len(buffer), False)
        
        # Create attack with high frequency energy, then decay
        for i, time in enumerate(t):
            if time < 0.005:  # Sharper attack for GTR-like sound
                buffer[i] = np.random.normal(0, 1.5) * (time / 0.005)
            else:
                # Exponential decay with some randomness - more aggressive
                decay = np.exp(-(time - 0.005) * 35)
                buffer[i] = np.random.normal(0, 0.7) * decay
        
        # Add more complex frequency content
        buffer += 0.8 * np.sin(2 * np.pi * 120 * t) * np.exp(-t * 20)  # Higher bass component
        buffer += 0.3 * np.sin(2 * np.pi * 240 * t) * np.exp(-t * 15)  # Mid-range
        buffer += 0.2 * np.sin(2 * np.pi * 500 * t) * np.exp(-t * 30)  # High-end snarl
        
        # Add turbo whine characteristic
        buffer += 0.15 * np.sin(2 * np.pi * 1500 * t) * np.exp(-t * 10)
        
        # Normalize
        buffer = 0.8 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
        
    def create_engine_start_sound(self):
        """Create GTR-like engine starting sound"""
        sample_rate = 44100
        duration = 2.5  # seconds - a bit longer for GTR
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate a starting sound - initial cranking followed by aggressive revving
        t = np.linspace(0, duration, len(buffer), False)
        
        # Initial cranking phase
        cranking_freq = 15  # Hz - slow cranking
        for i, time in enumerate(t):
            if time < 1.0:  # Cranking phase
                # Slow cranking motor
                buffer[i] = 0.5 * np.sin(2 * np.pi * cranking_freq * time)
                # Add more mechanical noise for GTR
                buffer[i] += 0.4 * np.random.normal(0, 0.4)
                # Motor sound increases
                buffer[i] *= (0.3 + 0.7 * time)
            elif time < 1.5:  # GTR fires up aggressively
                rev_factor = (time - 1.0) * 2
                rev_freq = 70 + 250 * rev_factor  # Higher initial rev
                buffer[i] = 0.8 * np.sin(2 * np.pi * rev_freq * time)
                # Add higher frequencies for GTR character
                buffer[i] += 0.5 * np.sin(2 * np.pi * rev_freq * 2 * time)
                buffer[i] += 0.3 * np.sin(2 * np.pi * rev_freq * 3 * time)
                # Add more noise
                buffer[i] += 0.3 * np.random.normal(0, 0.6)
            else:  # Settle to idle with a GTR-like bark
                decay_factor = (time - 1.5) * 2
                rev_freq = 320 - 150 * decay_factor
                buffer[i] = 0.7 * np.sin(2 * np.pi * rev_freq * time)
                buffer[i] += 0.4 * np.sin(2 * np.pi * rev_freq * 2 * time)
                buffer[i] += 0.2 * np.sin(2 * np.pi * rev_freq * 3 * time)
                # Add decreasing noise
                buffer[i] += 0.2 * np.random.normal(0, max(0.1, 0.5 - 0.5 * decay_factor))
        
        # Apply volume envelope
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
    
    def create_intake_sound(self):
        """Create intake sound with turbo whistle characteristic of a GTR"""
        sample_rate = 44100
        duration = 0.5  # seconds
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate intake sound with turbo whistle
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base intake whoosh
        for i, time in enumerate(t):
            # Air rush sound
            buffer[i] = 0.3 * np.random.normal(0, 0.8) * np.exp(-time * 5)
            
            # Add GTR's twin-turbo whistle (high frequency)
            turbo_freq = 2000 + 500 * np.sin(2 * np.pi * 10 * time)
            buffer[i] += 0.4 * np.sin(2 * np.pi * turbo_freq * time) * np.exp(-time * 3)
            
            # Add some whoosh
            whoosh_freq = 300 + 200 * np.sin(2 * np.pi * 5 * time)
            buffer[i] += 0.2 * np.sin(2 * np.pi * whoosh_freq * time) * np.exp(-time * 4)
        
        # Apply volume envelope
        envelope = np.ones_like(buffer)
        attack = 0.05  # seconds
        attack_samples = int(attack * sample_rate)
        fade_out = 0.2  # seconds
        fade_samples = int(fade_out * sample_rate)
        envelope[:attack_samples] = np.linspace(0, 1, attack_samples)
        envelope[-fade_samples:] = np.linspace(1, 0, fade_samples)
        buffer = buffer * envelope
        
        # Normalize
        buffer = 0.7 * buffer / np.max(np.abs(buffer))
        
        # Convert to 16-bit PCM
        buffer = (buffer * 32767).astype(np.int16)
        
        # Create stereo sound
        stereo_buffer = np.column_stack((buffer, buffer))
        
        return pygame.mixer.Sound(stereo_buffer)
        
    def create_exhaust_sound(self):
        """Create exhaust sound with the distinctive GTR growl"""
        sample_rate = 44100
        duration = 0.6  # seconds
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate exhaust sound with GTR characteristics
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base exhaust rumble
        for i, time in enumerate(t):
            # Low frequency rumble
            base_freq = 100
            buffer[i] = 0.6 * np.sin(2 * np.pi * base_freq * time) * np.exp(-time * 4)
            
            # Add GTR's signature mid-range growl
            growl_freq = 250 + 50 * np.sin(2 * np.pi * 15 * time)
            buffer[i] += 0.5 * np.sin(2 * np.pi * growl_freq * time) * np.exp(-time * 5)
            
            # Add some higher frequency crackle typical of GTR
            if np.random.random() > 0.95:
                buffer[i] += 0.2 * np.random.normal(0, 1.0)
        
        # Apply volume envelope
        envelope = np.ones_like(buffer)
        attack = 0.02  # seconds
        attack_samples = int(attack * sample_rate)
        fade_out = 0.3  # seconds
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
        
    def create_idle_sound(self):
        """Create an idle engine sound with GTR characteristics"""
        sample_rate = 44100
        duration = 1.0  # seconds
        
        # Create a buffer for sound data
        buffer = np.zeros(int(sample_rate * duration))
        
        # Generate an idle sound - deep rumble with slight unevenness
        t = np.linspace(0, duration, len(buffer), False)
        
        # Base frequency for idle - deeper for GTR
        idle_freq = 25  # Hz
        
        # Create a complex waveform with harmonics for GTR idle sound
        for i, time in enumerate(t):
            # Base deep rumble
            buffer[i] = 0.6 * np.sin(2 * np.pi * idle_freq * time)
            # Add harmonics with GTR character - more mid/high range
            buffer[i] += 0.4 * np.sin(2 * np.pi * idle_freq * 2 * time)
            buffer[i] += 0.25 * np.sin(2 * np.pi * idle_freq * 3 * time)
            buffer[i] += 0.15 * np.sin(2 * np.pi * idle_freq * 4 * time)
            buffer[i] += 0.1 * np.sin(2 * np.pi * idle_freq * 6 * time)
            # Add some uneven lope character
            lope = 0.05 * np.sin(2 * np.pi * 8 * time)
            buffer[i] *= (1.0 + lope)
            # Add some noise for texture
            buffer[i] += 0.15 * np.random.normal(0, 0.3)
        
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
            if self.intake_playing:
                self.intake_channel.fadeout(300)
                self.intake_playing = False
            if self.exhaust_playing:
                self.exhaust_channel.fadeout(300)
                self.exhaust_playing = False
            return
            
        # Play idle sound if not already playing
        if self.engine.running and not self.idle_playing:
            self.idle_channel.play(self.idle_sound, loops=-1)
            self.idle_playing = True
        
        # Adjust sound characteristics based on RPM and throttle
        if self.idle_playing:
            # Volume and pitch adjustments for GTR-like sound
            # - Idle volume decreases as RPM increases
            # - Turbo whine increases with RPM
            idle_volume = max(0.1, 0.5 - self.engine.rpm / MAX_RPM * 0.4)
            self.idle_channel.set_volume(idle_volume)
            
            # Periodically play intake sound for turbo whoosh at higher RPMs
            if self.engine.rpm > 2000 and self.engine.throttle > 0.3:
                if not self.intake_playing or not self.intake_channel.get_busy():
                    # Volume and pitch increase with RPM and throttle
                    intake_volume = min(0.8, 0.3 + (self.engine.rpm / MAX_RPM) * 0.5)
                    if self.engine.throttle > 0.7:  # Louder on heavy throttle
                        intake_volume *= 1.5
                    
                    self.intake_channel.play(self.intake_sound)
                    self.intake_channel.set_volume(intake_volume)
                    self.intake_playing = True
            
            # Play exhaust sound periodically at higher RPMs for GTR-like burble/crack
            if self.engine.rpm > 1500:
                if not self.exhaust_playing or not self.exhaust_channel.get_busy():
                    # Volume varies with RPM and throttle
                    exhaust_volume = min(1.0, 0.4 + (self.engine.rpm / MAX_RPM) * 0.6)
                    exhaust_volume *= (0.5 + 0.5 * self.engine.throttle)
                    
                    self.exhaust_channel.play(self.exhaust_sound)
                    self.exhaust_channel.set_volume(exhaust_volume * self.gtr_exhaust_volume)
                    self.exhaust_playing = True
        
        # Calculate how often to play cylinder explosion sounds based on RPM
        # In a 4-stroke engine, each cylinder fires once every 2 revolutions
        # For V6, that's 3 firings per revolution
        
        # Calculate time between explosion sounds (in seconds)
        if self.engine.rpm > 0:
            # RPM / 60 = revolutions per second
            # For V6 4-stroke: 3 explosions per revolution (6 cylinders / 2 revolutions)
            explosions_per_second = (self.engine.rpm / 60) * 3
            time_between_explosions = 1.0 / explosions_per_second
            
            # For higher accuracy, also check crank angle to determine firing
            current_angle = self.engine.crank_angle
            
            # Detect if we need to fire based on time and crank angle
            current_time = time.time()
            
            # Find the next cylinder to fire based on crank angle
            for cyl_idx, firing_angle in self.engine.firing_events:
                # Check if we've passed this firing angle since last check
                if ((current_angle > firing_angle and self.last_crank_angle < firing_angle) or
                    (current_angle < self.last_crank_angle and current_angle + 720 > firing_angle > self.last_crank_angle)):
                    
                    # Don't fire the same cylinder twice in quick succession
                    if cyl_idx != self.last_cylinder_fired or current_time - self.last_fire_time > time_between_explosions:
                        # Play explosion sound with GTR character
                        volume = min(1.0, 0.3 + 0.7 * self.engine.throttle)
                        
                        # Enhanced volume based on RPM range (GTR sounds best in mid-high range)
                        if 3000 < self.engine.rpm < 6000:
                            volume *= 1.2
                        
                        # Use the next available channel
                        channel = self.explosion_channels[self.channel_index]
                        self.channel_index = (self.channel_index + 1) % len(self.explosion_channels)
                        
                        # Play the sound
                        channel.play(self.explosion_sound)
                        channel.set_volume(volume)
                        
                        # Update tracking variables
                        self.last_fire_time = current_time
                        self.last_cylinder_fired = cyl_idx
                        break
            
            # Update last crank angle
            self.last_crank_angle = current_angle


class VariableEngineSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Variable Engine Simulator - Nissan GTR VR38DETT")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('Arial', 20)
        self.title_font = pygame.font.SysFont('Arial', 24, bold=True)
        
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
        
        # Diagnostic 60 RPM button (red)
        self.diagnostic_button_rect = pygame.Rect(20, HEIGHT - 80, 140, 40)
        
        # FPS counter
        self.fps_update_interval = 0.5  # Update FPS every 0.5 seconds
        self.fps_last_update = time.time()
        self.frame_count = 0
        self.fps = 0
        
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
        
        # Draw diagnostic button 
        self.draw_diagnostic_button()
        
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
        if self.engine.diagnostic_mode and self.engine.running:
            state_text += " (60 RPM)"
            
        state_color = (255, 255, 0)  # Yellow like in screenshot
        state_surface = self.font.render(f"Engine: {state_text}", True, state_color)
        self.screen.blit(state_surface, (180, HEIGHT - 30))
        
    def draw_crankshaft(self, crank_x, crank_y):
        """Draw the crankshaft with rotating components"""
        # Draw main crankshaft centerpiece with cross-line details
        pygame.draw.circle(self.screen, CRANKSHAFT_COLOR, (crank_x, crank_y), 35)
        pygame.draw.circle(self.screen, (120, 120, 120), (crank_x, crank_y), 25)
        
        # Add cross-line detail like in the reference image
        cross_length = 50  # Length of cross lines
        
        # Horizontal line through center
        pygame.draw.line(self.screen, (255, 102, 153), 
                      (crank_x - cross_length/2, crank_y),
                      (crank_x + cross_length/2, crank_y), 3)
                      
        # Vertical line through center
        pygame.draw.line(self.screen, (255, 102, 153), 
                      (crank_x, crank_y - cross_length/2),
                      (crank_x, crank_y + cross_length/2), 3)
        
        # Center circle
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
            
            # Get corresponding crank pin position
            pin_angle = self.engine.crank_pin_angles[cyl_idx]
            pin_rad = math.radians((self.engine.crank_angle + pin_angle) % 360)
            pin_x = crank_x + self.engine.crank_radius * math.cos(pin_rad)
            pin_y = crank_y - self.engine.crank_radius * math.sin(pin_rad)
            
            # Calculate cylinder base position with offset from crankshaft
            cylinder_base_x = crank_x + self.engine.cylinder_offset * dir_x
            cylinder_base_y = crank_y + self.engine.cylinder_offset * dir_y
            
            # Calculate cylinder dimensions
            cylinder_height = 200
            cylinder_width = 70
            cylinder_spacing = 30  # Offset between cylinders in the same bank
            
            # Calculate position along the bank
            pos_along_bank = i * cylinder_spacing
            cyl_center_x = cylinder_base_x + pos_along_bank * perp_x
            cyl_center_y = cylinder_base_y + pos_along_bank * perp_y
            
            # Calculate top of cylinder
            cyl_top_x = cyl_center_x + cylinder_height * dir_x
            cyl_top_y = cyl_center_y + cylinder_height * dir_y
            
            # Create a surface for this cylinder with transparency
            cylinder_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            
            # Calculate the four corners of the cylinder as an outline (not filled)
            half_width = cylinder_width / 2
            
            # Calculate the four corners of the cylinder
            cylinder_points = [
                (cyl_center_x - half_width * perp_x, cyl_center_y - half_width * perp_y),
                (cyl_center_x + half_width * perp_x, cyl_center_y + half_width * perp_y),
                (cyl_top_x + half_width * perp_x, cyl_top_y + half_width * perp_y),
                (cyl_top_x - half_width * perp_x, cyl_top_y - half_width * perp_y)
            ]
            
            # Draw connecting rod first (so it appears behind piston)
            rod_color = list(CONNECTING_ROD_COLOR) + [alpha]
            
            # Calculate piston position within the cylinder
            piston_travel = cylinder_height * 0.8
            piston_x = cyl_center_x + piston_travel * piston_pos * dir_x
            piston_y = cyl_center_y + piston_travel * piston_pos * dir_y
            
            # Draw connecting rod from crank pin to piston
            pygame.draw.line(cylinder_surface, rod_color, (pin_x, pin_y), (piston_x, piston_y), 8)
            
            # Draw piston (white rectangle)
            piston_width = cylinder_width - 10  # Slightly smaller than cylinder
            piston_height = 40
            
            # Calculate piston points
            piston_points = self.get_rotated_rect(
                piston_x, piston_y,
                piston_width, piston_height,
                math.degrees(bank_rad)
            )
            
            # Draw piston
            piston_color = list(PISTON_COLOR) + [alpha]
            pygame.draw.polygon(cylinder_surface, piston_color, piston_points, 0)
            
            # Add piston pin (black dot)
            pin_color = list(PIN_COLOR) + [alpha]
            pygame.draw.circle(cylinder_surface, pin_color, (int(piston_x), int(piston_y)), 5)
            
            # Now draw cylinder outline (pink)
            cylinder_color = list(CYLINDER_COLOR) + [alpha]
            pygame.draw.polygon(cylinder_surface, cylinder_color, cylinder_points, 2)  # 2-pixel width outline
            
            # Draw valve and spring details consistently positioned at exact top corners of cylinder
            valve_radius = 10
            spring_radius = 10
            
            # Position valves exactly at the top corners of cylinder
            # Calculate valve positions
            valve1_x = cyl_top_x - half_width * 0.6 * perp_x
            valve1_y = cyl_top_y - half_width * 0.6 * perp_y
            
            valve2_x = cyl_top_x + half_width * 0.6 * perp_x
            valve2_y = cyl_top_y + half_width * 0.6 * perp_y
            
            # Draw valves (blue circles)
            valve_color = list(VALVE_COLOR) + [alpha]
            pygame.draw.circle(cylinder_surface, valve_color, (int(valve1_x), int(valve1_y)), valve_radius)
            pygame.draw.circle(cylinder_surface, valve_color, (int(valve2_x), int(valve2_y)), valve_radius)
            
            # Position springs slightly behind valves
            spring_offset = 12
            spring1_x = valve1_x + spring_offset * dir_x
            spring1_y = valve1_y + spring_offset * dir_y
            
            spring2_x = valve2_x + spring_offset * dir_x
            spring2_y = valve2_y + spring_offset * dir_y
            
            # Draw springs (yellow circles)
            spring_color = list(SPRING_COLOR) + [alpha]
            pygame.draw.circle(cylinder_surface, spring_color, (int(spring1_x), int(spring1_y)), spring_radius)
            pygame.draw.circle(cylinder_surface, spring_color, (int(spring2_x), int(spring2_y)), spring_radius)
            
            # Blit the cylinder surface to the screen
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
    
    def draw_fps_counter(self):
        """Draw FPS counter in the top right corner"""
        # Update frame count
        self.frame_count += 1
        
        # Calculate FPS every update interval
        current_time = time.time()
        if current_time - self.fps_last_update > self.fps_update_interval:
            self.fps = self.frame_count / (current_time - self.fps_last_update)
            self.frame_count = 0
            self.fps_last_update = current_time
        
        # Draw FPS text
        fps_text = f"FPS: {int(self.fps)}"
        fps_surface = self.font.render(fps_text, True, (255, 255, 255))
        self.screen.blit(fps_surface, (WIDTH - fps_surface.get_width() - 10, 10))
        
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
        
    def draw_diagnostic_button(self):
        """Draw the 60 RPM diagnostic button"""
        # Always red, but darker when inactive
        if self.engine.diagnostic_mode:
            button_color = (255, 0, 0)  # Bright red when active
            text_color = (255, 255, 255)  # White text
        else:
            button_color = (150, 0, 0)  # Darker red when inactive
            text_color = (220, 220, 220)  # Light gray text
            
        # Draw button
        pygame.draw.rect(self.screen, button_color, self.diagnostic_button_rect, 0, 5)
        pygame.draw.rect(self.screen, (200, 200, 200), self.diagnostic_button_rect, 2, 5)
        
        # Draw button text
        text_surface = self.font.render("60 RPM MODE", True, text_color)
        text_x = self.diagnostic_button_rect.x + (self.diagnostic_button_rect.width - text_surface.get_width()) // 2
        text_y = self.diagnostic_button_rect.y + (self.diagnostic_button_rect.height - text_surface.get_height()) // 2
        self.screen.blit(text_surface, (text_x, text_y))
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.engine.running = False
                pygame.quit()
                return False
                
            elif event.type == MOUSEBUTTONDOWN:
                # Check if clicked on diagnostic button
                if self.diagnostic_button_rect.collidepoint(event.pos):
                    self.engine.toggle_diagnostic_mode()
                    
                # Check if clicked on start/stop button
                elif self.start_button_rect.collidepoint(event.pos):
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
                elif event.key == K_d:
                    # 'D' key toggles diagnostic mode
                    self.engine.toggle_diagnostic_mode()
                    
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
            self.clock.tick(60)  # Limit to 60 FPS
            
        # Clean up
        pygame.quit()

if __name__ == "__main__":
    simulator = VariableEngineSimulator()
    simulator.run()