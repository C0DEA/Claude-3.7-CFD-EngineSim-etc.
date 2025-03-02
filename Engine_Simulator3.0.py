import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import RadioButtons, Slider, Button
import pygame
import random
import time
import math
import os
from scipy.integrate import solve_ivp
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from enum import Enum, auto

# Constants
R_AIR = 287.058  # J/(kg·K) - Specific gas constant for air
GAMMA = 1.4  # Ratio of specific heats for air
AIR_FUEL_RATIO_STOICH = 14.7  # Stoichiometric air-fuel ratio for gasoline

# Engine Configuration Classes
class EngineLayout(Enum):
    INLINE = auto()
    V60 = auto()
    V90 = auto()
    BOXER = auto()
    FLAT = auto()

class CrankshaftType(Enum):
    FLATPLANE = auto()
    CROSSPLANE = auto()

@dataclass
class EngineConfig:
    # Basic configuration
    num_cylinders: int = 4
    layout: EngineLayout = EngineLayout.INLINE
    
    # Cylinder dimensions
    bore: float = 86.0  # mm
    stroke: float = 86.0  # mm
    
    # Performance parameters
    compression_ratio: float = 10.0
    ignition_timing: float = 15.0  # degrees BTDC (Before Top Dead Center)
    air_fuel_ratio: float = 14.7  # Air-fuel ratio
    
    # Crankshaft configuration
    crankshaft_type: CrankshaftType = CrankshaftType.FLATPLANE
    
    # Valve timing (in crank degrees)
    intake_valve_open: float = 10.0  # BTDC
    intake_valve_close: float = 40.0  # ABDC (After Bottom Dead Center)
    exhaust_valve_open: float = 40.0  # BBDC (Before Bottom Dead Center)
    exhaust_valve_close: float = 10.0  # ATDC (After Top Dead Center)
    
    # Forced induction
    forced_induction_pressure: float = 1.0  # bar (1.0 = atmospheric)
    
    # Reliability factors (0-100, higher is better)
    reliability: float = 90.0

    def __post_init__(self):
        # Validate configuration
        if self.num_cylinders < 1 or self.num_cylinders > 16:
            raise ValueError("Number of cylinders must be between 1 and 16")
        
        if self.compression_ratio < 7.0 or self.compression_ratio > 15.0:
            raise ValueError("Compression ratio typically ranges from 7.0 to 15.0")
        
        if self.air_fuel_ratio < 10.0 or self.air_fuel_ratio > 20.0:
            raise ValueError("Air-fuel ratio typically ranges from 10.0 to 20.0")

        # Calculate derived properties
        self.displacement_per_cylinder = (np.pi * (self.bore/2)**2 * self.stroke) / 1000  # cc
        self.total_displacement = self.displacement_per_cylinder * self.num_cylinders  # cc
    
        # Configure firing order based on engine layout and cylinder count
        self.firing_order = self._determine_firing_order()
        
        # Calculate rod to stroke ratio (typical value is around 1.75)
        self.rod_length = self.stroke * 1.75  # mm
    
    def _determine_firing_order(self) -> List[int]:
        """Determine the firing order based on engine layout and cylinder count."""
        if self.layout == EngineLayout.INLINE:
            if self.num_cylinders == 4:
                return [1, 3, 4, 2]
            elif self.num_cylinders == 6:
                return [1, 5, 3, 6, 2, 4]
            elif self.num_cylinders == 8:
                return [1, 8, 4, 3, 6, 5, 7, 2]
        elif self.layout in [EngineLayout.V60, EngineLayout.V90]:
            if self.num_cylinders == 6:
                return [1, 4, 2, 5, 3, 6]
            elif self.num_cylinders == 8:
                if self.crankshaft_type == CrankshaftType.FLATPLANE:
                    return [1, 5, 4, 8, 3, 7, 2, 6]
                else:  # CROSSPLANE
                    return [1, 3, 7, 2, 6, 5, 4, 8]
            elif self.num_cylinders == 10:
                return [1, 6, 5, 10, 2, 7, 3, 8, 4, 9]
            elif self.num_cylinders == 12:
                return [1, 7, 4, 10, 2, 8, 6, 12, 3, 9, 5, 11]
        elif self.layout in [EngineLayout.BOXER, EngineLayout.FLAT]:
            if self.num_cylinders == 4:
                return [1, 3, 2, 4]
            elif self.num_cylinders == 6:
                return [1, 5, 3, 6, 2, 4]
            
        # Default firing order for other configurations
        return list(range(1, self.num_cylinders + 1))

    def get_cylinder_angle(self, cylinder_idx: int) -> float:
        """Get the angular offset for a cylinder based on the engine layout."""
        base_angle = 720 / self.num_cylinders  # Angle between cylinders in a 4-stroke cycle
        
        # Determine firing order position
        position = self.firing_order.index(cylinder_idx + 1)
        
        # Apply offset based on position in firing order
        if self.layout == EngineLayout.INLINE:
            return position * base_angle
        elif self.layout == EngineLayout.V60:
            bank = position % 2  # 0 for left bank, 1 for right bank
            row = position // 2
            return row * base_angle + (bank * 60)  # 60 degree offset between banks
        elif self.layout == EngineLayout.V90:
            bank = position % 2  # 0 for left bank, 1 for right bank
            row = position // 2
            return row * base_angle + (bank * 90)  # 90 degree offset between banks
        elif self.layout == EngineLayout.BOXER:
            bank = position % 2  # 0 for left bank, 1 for right bank
            row = position // 2
            return row * base_angle + (bank * 180)  # 180 degree offset between banks
        elif self.layout == EngineLayout.FLAT:
            bank = position % 2  # 0 for left bank, 1 for right bank
            row = position // 2
            return row * base_angle + (bank * 180)  # 180 degree offset between banks
        
        # Default
        return position * base_angle


class CylinderState:
    """Represents the state of a single cylinder."""
    def __init__(self, config: EngineConfig, cylinder_idx: int):
        self.config = config
        self.cylinder_idx = cylinder_idx
        self.angle_offset = config.get_cylinder_angle(cylinder_idx)
        
        # Cylinder state
        self.pressure = 1.0  # bar
        self.temperature = 298.0  # K
        self.volume = 0.0  # cc
        self.mass_air = 0.0  # g
        self.mass_fuel = 0.0  # g
        
        # Valve states
        self.intake_valve_open = False
        self.exhaust_valve_open = False
        
        # Combustion state
        self.is_combustion = False
        self.combustion_progress = 0.0
        
        # Knock detection
        self.knock_intensity = 0.0
        self.knock_detected = False
        
        # Mechanical state
        self.piston_position = 0.0  # 0 (BDC) to 1 (TDC)
        
        # Cycle metrics
        self.power_output = 0.0  # W
        self.torque = 0.0  # Nm
        
        # Reliability metrics
        self.wear = 0.0  # 0 to 100
        self.is_broken = False
        
    def update(self, crank_angle: float, rpm: float) -> None:
        """Update cylinder state based on crankshaft angle."""
        # Calculate current angle of this cylinder
        current_angle = (crank_angle + self.angle_offset) % 720
        
        # Calculate piston position using crankshaft geometry
        self.piston_position = self._calculate_piston_position(current_angle)
        
        # Update valve states
        self._update_valve_states(current_angle)
        
        # Update cylinder volume
        self._calculate_volume()
        
        # Determine cycle phase and execute appropriate calculations
        if current_angle >= 0 and current_angle < 180:
            # Intake stroke
            self._intake_stroke(current_angle, rpm)
        elif current_angle >= 180 and current_angle < 360:
            # Compression stroke
            self._compression_stroke(current_angle)
        elif current_angle >= 360 and current_angle < 540:
            # Power stroke (combustion)
            self._power_stroke(current_angle, rpm)
        elif current_angle >= 540 and current_angle < 720:
            # Exhaust stroke
            self._exhaust_stroke(current_angle)
            
        # Check for knock
        self._check_knock()
        
        # Update reliability/wear
        self._update_reliability(rpm)
    
    def _calculate_piston_position(self, angle: float) -> float:
        """Calculate normalized piston position (0=BDC, 1=TDC) using crank slider geometry."""
        # Convert angle to radians
        rad_angle = math.radians(angle)
        
        # Use the crank-slider equation to calculate piston position
        crank_radius = self.config.stroke / 2
        rod_length = self.config.rod_length
        
        # Normalized position (0 = TDC, 1 = BDC)
        pos = crank_radius * math.cos(rad_angle) + math.sqrt(rod_length**2 - (crank_radius * math.sin(rad_angle))**2)
        pos = (pos - (rod_length - crank_radius)) / (2 * crank_radius)
        
        # Convert to our convention (0 = BDC, 1 = TDC)
        return 1 - pos
        
    def _update_valve_states(self, angle: float) -> None:
        """Update intake and exhaust valve states based on timing."""
        # Intake valve
        if angle >= (720 - self.config.intake_valve_open) or angle < (180 + self.config.intake_valve_close):
            self.intake_valve_open = True
        else:
            self.intake_valve_open = False
            
        # Exhaust valve
        if angle >= (540 - self.config.exhaust_valve_open) and angle < (720 + self.config.exhaust_valve_close):
            self.exhaust_valve_open = True
        else:
            self.exhaust_valve_open = False
    
    def _calculate_volume(self) -> None:
        """Calculate the current volume of the cylinder based on piston position."""
        # Get cylinder bore and stroke
        bore = self.config.bore / 10  # convert mm to cm
        stroke = self.config.stroke / 10  # convert mm to cm
        
        # Clearance volume (at TDC)
        displacement = (np.pi / 4) * bore**2 * stroke  # cc
        clearance_volume = displacement / (self.config.compression_ratio - 1)
        
        # Calculate current volume based on piston position (0=BDC, 1=TDC)
        current_volume = clearance_volume + displacement * (1 - self.piston_position)
        self.volume = current_volume  # cc
    
    def _intake_stroke(self, angle: float, rpm: float) -> None:
        """Simulate intake stroke."""
        if self.intake_valve_open:
            # Calculate mass flow rate based on pressure difference and valve opening
            pressure_ratio = self.config.forced_induction_pressure / self.pressure
            
            # Simple model for mass flow rate
            flow_coefficient = 0.7  # Discharge coefficient
            valve_area = np.pi * 1.5**2  # Assume 30mm valve diameter
            
            # Density of intake air
            rho_intake = 1.2 * self.config.forced_induction_pressure  # kg/m^3
            
            # Flow velocity based on pressure difference
            if pressure_ratio > 1:
                flow_velocity = 20 * math.sqrt(pressure_ratio - 1)  # m/s
            else:
                flow_velocity = 0
                
            # Mass flow rate
            mass_flow = flow_coefficient * valve_area * rho_intake * flow_velocity / 10000  # g/s
            
            # Time step based on rpm
            dt = 1 / (rpm / 60) / 720  # seconds per degree
            
            # Mass of air inducted in this step
            dmass = mass_flow * dt
            
            # Update cylinder state
            self.mass_air += dmass
            
            # Calculate fuel mass based on air-fuel ratio
            self.mass_fuel = self.mass_air / self.config.air_fuel_ratio
            
            # Update pressure and temperature
            # Assuming isothermal process during intake
            self.pressure = self.config.forced_induction_pressure
            self.temperature = 298.0  # K (assume intake temperature is constant)
    
    def _compression_stroke(self, angle: float) -> None:
        """Simulate compression stroke."""
        if angle == 180:
            # Start of compression stroke
            self.initial_compression_volume = self.volume
            self.initial_compression_pressure = self.pressure
            self.initial_compression_temperature = self.temperature
        
        # Polytropic compression process: PV^n = constant
        # For compression, n is typically between 1.3 and 1.35
        n = 1.32
        
        if self.initial_compression_volume > 0:
            # Calculate new pressure using polytropic process
            self.pressure = self.initial_compression_pressure * (self.initial_compression_volume / self.volume)**n
            
            # Calculate new temperature using ideal gas law
            self.temperature = self.initial_compression_temperature * (self.initial_compression_volume / self.volume)**(n-1)
        
        # Check for auto-ignition conditions (knock precursor)
        threshold_temperature = 950  # K
        if self.temperature > threshold_temperature and angle < (360 - self.config.ignition_timing):
            # Increasing knock likelihood if temperature gets too high before ignition
            self.knock_intensity += 0.1 * (self.temperature - threshold_temperature) / 100
    
    def _power_stroke(self, angle: float, rpm: float) -> None:
        """Simulate power (combustion) stroke."""
        # Ignition timing
        ignition_angle = 360 - self.config.ignition_timing
        
        if angle >= ignition_angle and not self.is_combustion:
            # Initiate combustion
            self.is_combustion = True
            self.combustion_progress = 0.0
            self.combustion_start_pressure = self.pressure
            self.combustion_start_temperature = self.temperature
            self.combustion_start_volume = self.volume
        
        if self.is_combustion:
            # Wiebe function for combustion progress
            # a and m are Wiebe function parameters
            a = 5.0
            m = 2.0
            
            # Combustion duration in crank angle degrees
            combustion_duration = 60.0  # typical for gasoline
            
            # Normalized combustion progress
            if angle - ignition_angle < combustion_duration:
                x = (angle - ignition_angle) / combustion_duration
                self.combustion_progress = 1 - math.exp(-a * x**(m+1))
            else:
                self.combustion_progress = 1.0
            
            # Heat release during combustion
            # Lower heating value of gasoline in J/g
            lhv = 44000
            
            # Total energy available from fuel
            available_energy = self.mass_fuel * lhv
            
            # Energy released in this step
            if angle == ignition_angle:
                energy_released = 0
            else:
                energy_released = available_energy * (self.combustion_progress - self.combustion_progress_prev 
                                                    if hasattr(self, 'combustion_progress_prev') else self.combustion_progress)
            
            self.combustion_progress_prev = self.combustion_progress
            
            # Pressure increase due to combustion (simplified model)
            # Assume constant volume combustion initially
            if self.volume > 0:
                # dp = dQ/V * (gamma-1)
                dp = energy_released / self.volume * (GAMMA - 1) / 1000  # bar
                self.pressure += dp
                
                # Temperature increase
                self.temperature = self.pressure * self.volume / (self.mass_air * R_AIR / 1000)
            
            # Calculate power output
            if angle > ignition_angle:
                # Calculate P-dV work
                dt = 1 / (rpm / 60) / 720  # seconds per degree
                power = dp * self.volume / dt  # W
                self.power_output = power
                
                # Calculate torque
                if rpm > 0:
                    self.torque = power / (rpm * 2 * np.pi / 60)  # Nm
        
        if angle >= ignition_angle + combustion_duration:
            # Expansion after combustion (polytropic process)
            n = 1.3
            self.pressure = self.combustion_start_pressure * (self.combustion_start_volume / self.volume)**n
            
            # End combustion
            if angle >= 540:
                self.is_combustion = False
    
    def _exhaust_stroke(self, angle: float) -> None:
        """Simulate exhaust stroke."""
        if self.exhaust_valve_open:
            # Exhaust pressure typically slightly above atmospheric
            target_pressure = 1.1  # bar
            
            # Simple model for pressure equalization
            # Pressure exponentially approaches target pressure
            rate = 0.1
            self.pressure = self.pressure * (1 - rate) + target_pressure * rate
            
            # Temperature also drops
            self.temperature = self.temperature * (1 - rate) + 400 * rate  # K
            
            # Reset cylinder charge at end of exhaust stroke
            if angle >= 710:
                self.mass_air = 0
                self.mass_fuel = 0
    
    def _check_knock(self) -> None:
        """Check for engine knock."""
        # Factors that increase knock:
        # 1. High compression ratio
        # 2. Advanced ignition timing
        # 3. Low octane fuel / poor air-fuel mixture
        # 4. High engine load/temperature
        
        # Simulate knock threshold based on engine parameters
        knock_threshold = 1.0
        
        # Adjust threshold based on compression ratio
        # Higher compression ratio = lower threshold (easier to knock)
        compression_factor = 1.0 - (self.config.compression_ratio - 8) / 10
        knock_threshold *= max(0.5, compression_factor)
        
        # Adjust threshold based on ignition timing
        # More advanced timing = lower threshold
        timing_factor = 1.0 - (self.config.ignition_timing - 10) / 20
        knock_threshold *= max(0.5, timing_factor)
        
        # Adjust threshold based on air-fuel ratio
        # Lean mixtures (higher AFR) are more prone to knock
        afr_factor = 1.0 - (self.config.air_fuel_ratio - AIR_FUEL_RATIO_STOICH) / 5
        knock_threshold *= max(0.7, afr_factor)
        
        # Check if knock intensity exceeds threshold
        self.knock_detected = self.knock_intensity > knock_threshold
        
        # Decay knock intensity over time
        self.knock_intensity *= 0.95
    
    def _update_reliability(self, rpm: float) -> None:
        """Update cylinder reliability/wear."""
        # Base wear rate
        base_wear_rate = 0.0001
        
        # Factors affecting wear
        # 1. RPM factor - higher RPM = more wear
        rpm_factor = (rpm / 3000)**2
        
        # 2. Knock factor - knocking causes significant wear
        knock_factor = 1.0 + (10.0 * self.knock_intensity if self.knock_detected else 0)
        
        # 3. Temperature factor - higher temperatures accelerate wear
        temp_factor = (self.temperature / 800)**2
        
        # Calculate wear increment
        wear_increment = base_wear_rate * rpm_factor * knock_factor * temp_factor
        
        # Apply reliability rating (higher reliability = slower wear)
        reliability_factor = (100 - self.config.reliability) / 100 + 0.1
        wear_increment *= reliability_factor
        
        # Update wear
        self.wear += wear_increment
        
        # Check if engine has failed
        # Random failure chance increases with wear
        failure_probability = (self.wear / 100)**3
        if random.random() < failure_probability:
            self.is_broken = True


class EngineSimulator:
    def __init__(self, config: EngineConfig):
        self.config = config
        self.cylinders = [CylinderState(config, i) for i in range(config.num_cylinders)]
        
        # Engine state
        self.crank_angle = 0.0  # degrees
        self.rpm = 1000.0  # initial RPM
        self.throttle_position = 0.2  # 0.0 to 1.0
        self.target_rpm = 1000.0
        
        # Engine metrics
        self.total_power = 0.0  # W
        self.total_torque = 0.0  # Nm
        self.engine_load = 0.0  # 0.0 to 1.0
        
        # Performance metrics
        self.fuel_consumption = 0.0  # g/s
        self.thermal_efficiency = 0.0  # percentage
        
        # Engine temperature (°C)
        self.coolant_temp = 85.0
        
        # Timing chain/belt wear (0 to 100)
        self.timing_chain_wear = 0.0
        
        # Simulation time step
        self.time_step = 1.0  # crank angle degrees per step
        
        # Load/resistance model
        self.load_coefficient = 0.001  # Load coefficient for RPM decay
        
        # Initialize audio
        pygame.mixer.init()
        
        # Load sound effects
        self._init_sound_effects()
        
        # Combustion tracking for sound
        self.last_combustion_angles = [0.0] * config.num_cylinders
        self.combustion_started = [False] * config.num_cylinders
        
        # Engine status
        self.is_running = False
        
    def _init_sound_effects(self):
        """Initialize engine sound effects."""
        try:
            # Create sounds programmatically since we can't include actual files
            pygame.mixer.set_num_channels(16)  # Set enough channels for all cylinders
            
            # Generate a more realistic engine sound
            sample_rate = 44100
            duration = 0.2  # seconds
            
            # Create a more complex sound for combustion
            buffer = np.zeros((int(duration * sample_rate), 2), dtype=np.int16)
            max_amplitude = 32767  # Max amplitude for 16-bit audio
            
            # Combine multiple frequencies to make a more realistic engine sound
            base_freq = 100
            for i in range(int(duration * sample_rate)):
                t = i / sample_rate
                # Combine fundamental frequency with harmonics for richer sound
                value = int(max_amplitude * 0.4 * np.sin(2 * np.pi * base_freq * t) * np.exp(-i / (duration * sample_rate * 0.3)))
                value += int(max_amplitude * 0.2 * np.sin(2 * np.pi * base_freq * 2 * t) * np.exp(-i / (duration * sample_rate * 0.2)))
                value += int(max_amplitude * 0.1 * np.sin(2 * np.pi * base_freq * 3 * t) * np.exp(-i / (duration * sample_rate * 0.1)))
                value += int(max_amplitude * 0.05 * np.random.rand() * np.exp(-i / (duration * sample_rate * 0.05)))  # Add some noise
                
                # Clip to prevent overflow
                value = max(-32768, min(32767, value))
                
                buffer[i, 0] = value  # Left channel
                buffer[i, 1] = value  # Right channel
            
            # Create a Sound object from the buffer
            self.combustion_sound = pygame.mixer.Sound(buffer)
            
            # Create an idle sound (a continuous low rumble)
            idle_duration = 1.0  # seconds (longer for looping)
            idle_buffer = np.zeros((int(idle_duration * sample_rate), 2), dtype=np.int16)
            
            idle_freq = 60  # Lower frequency for idle
            for i in range(int(idle_duration * sample_rate)):
                t = i / sample_rate
                # Create a rumbling sound with multiple low frequencies
                value = int(max_amplitude * 0.2 * np.sin(2 * np.pi * idle_freq * t))
                value += int(max_amplitude * 0.1 * np.sin(2 * np.pi * (idle_freq + 5) * t))
                value += int(max_amplitude * 0.05 * np.sin(2 * np.pi * (idle_freq + 10) * t))
                value += int(max_amplitude * 0.05 * np.random.rand())  # Add some noise
                
                # Add some amplitude modulation to create a pulsing effect
                mod = 0.7 + 0.3 * np.sin(2 * np.pi * 8 * t)  # 8 Hz modulation
                value = int(value * mod)
                
                # Clip to prevent overflow
                value = max(-32768, min(32767, value))
                
                idle_buffer[i, 0] = value  # Left channel
                idle_buffer[i, 1] = value  # Right channel
            
            self.idle_sound = pygame.mixer.Sound(idle_buffer)
            
            # Flag to track if idle sound is playing
            self.idle_sound_playing = False
            
        except Exception as e:
            print(f"Error initializing sound: {e}")
            # Create dummy sounds to prevent errors
            buffer = np.zeros((1000, 2), dtype=np.int16)
            self.combustion_sound = pygame.mixer.Sound(buffer)
            self.idle_sound = pygame.mixer.Sound(buffer)
    
    def update(self) -> None:
        """Update engine state by one time step."""
        if not self.is_running:
            return
            
        # Check if any cylinder is broken
        if any(cyl.is_broken for cyl in self.cylinders):
            self.is_running = False
            print("ENGINE FAILURE DETECTED!")
            return
        
        # Update crank angle
        self.crank_angle = (self.crank_angle + self.time_step) % 720
        
        # Reset metrics for this cycle
        cycle_power = 0.0
        cycle_torque = 0.0
        
        # Handle idle sound (continuous engine background noise)
        if pygame.mixer.get_init() and not self.idle_sound_playing:
            idle_channel = pygame.mixer.Channel(0)
            self.idle_sound.set_volume(0.3)
            idle_channel.play(self.idle_sound, loops=-1)  # Loop continuously
            self.idle_sound_playing = True
            
        # Update the idle sound volume based on RPM
        if pygame.mixer.get_init() and self.idle_sound_playing:
            # Adjust volume based on RPM (higher RPM = louder)
            idle_volume = 0.3 + 0.5 * min(1.0, self.rpm / 6000)
            pygame.mixer.Channel(0).set_volume(idle_volume)
        
        # Update each cylinder and check for combustion events
        for i, cylinder in enumerate(self.cylinders):
            # Store previous combustion state
            was_combustion = cylinder.is_combustion
            
            # Update cylinder state
            cylinder.update(self.crank_angle, self.rpm)
            
            # Check for start of combustion (for sound)
            if cylinder.is_combustion and not was_combustion:
                # Play combustion sound
                if pygame.mixer.get_init():
                    # Adjust volume based on throttle and RPM
                    volume = min(1.0, 0.3 + 0.7 * self.throttle_position * (self.rpm / 6000))
                    self.combustion_sound.set_volume(volume)
                    
                    # Play on a separate channel for each cylinder
                    # Add small variations to pitch to make it sound more natural
                    pitch_variation = 0.9 + 0.2 * random.random()
                    self.combustion_sound.set_volume(volume)
                    pygame.mixer.Channel(i % 8 + 1).play(self.combustion_sound)
            
            # Add to cycle metrics
            cycle_power += cylinder.power_output
            cycle_torque += cylinder.torque
        
        # Calculate total power and torque
        self.total_power = cycle_power
        self.total_torque = cycle_torque
        
        # Calculate engine load
        self.engine_load = self.throttle_position
        
        # Update RPM based on torque, load, and inertia
        # Simplified model: RPM changes based on net torque and a load factor
        # If throttle position provides more power than load requires, RPM increases
        target_rpm_delta = (self.target_rpm - self.rpm) * 0.05
        rpm_delta = target_rpm_delta + self.total_torque * 0.1 - self.rpm * self.load_coefficient
        self.rpm = max(500, self.rpm + rpm_delta)
        
        # Calculate fuel consumption
        # Base consumption rate in g/s at 1000 RPM and 100% throttle
        base_consumption = self.config.total_displacement / 1000 * 0.5
        
        # Scale by RPM and throttle
        self.fuel_consumption = base_consumption * (self.rpm / 1000) * self.throttle_position
        
        # Calculate thermal efficiency (simplified)
        # Typical gasoline engine: 20-35% efficiency
        base_efficiency = 0.25  # 25%
        
        # Efficiency affected by RPM (best at mid-range)
        rpm_factor = 1.0 - abs((self.rpm - 3000) / 4000)
        
        # Efficiency affected by load (best at high load)
        load_factor = 0.5 + 0.5 * self.engine_load
        
        # Efficiency affected by knock (reduces efficiency)
        knock_factor = 1.0 - 0.2 * sum(cyl.knock_intensity for cyl in self.cylinders) / len(self.cylinders)
        
        self.thermal_efficiency = base_efficiency * rpm_factor * load_factor * knock_factor * 100  # percentage
        
        # Update coolant temperature
        target_temp = 85 + 15 * self.engine_load
        self.coolant_temp += (target_temp - self.coolant_temp) * 0.01
        
        # Update timing chain/belt wear
        base_wear_rate = 0.0001
        self.timing_chain_wear += base_wear_rate * (self.rpm / 1000) * (1 + self.engine_load)
        
    def set_throttle(self, position: float) -> None:
        """Set throttle position (0.0 to 1.0)."""
        self.throttle_position = max(0.0, min(1.0, position))
        self.target_rpm = 1000 + self.throttle_position * 6000
    
    def start_engine(self) -> None:
        """Start the engine."""
        if not self.is_running and not any(cyl.is_broken for cyl in self.cylinders):
            self.is_running = True
            self.rpm = 1000.0
            print("Engine started")
            
            # Play startup sound sequence
            if pygame.mixer.get_init():
                try:
                    # Generate a starter motor sound
                    sample_rate = 44100
                    duration = 1.0  # seconds
                    
                    # Create a starter motor sound
                    buffer = np.zeros((int(duration * sample_rate), 2), dtype=np.int16)
                    max_amplitude = 32767  # Max amplitude for 16-bit audio
                    
                    # Generate a whining starter motor sound
                    starter_freq = 300
                    for i in range(int(duration * sample_rate)):
                        t = i / sample_rate
                        # Frequency rises as the starter motor spins up
                        freq = starter_freq + 300 * t
                        value = int(max_amplitude * 0.4 * np.sin(2 * np.pi * freq * t))
                        value += int(max_amplitude * 0.1 * np.random.rand())  # Add some noise
                        
                        # Clip to prevent overflow
                        value = max(-32768, min(32767, value))
                        
                        buffer[i, 0] = value  # Left channel
                        buffer[i, 1] = value  # Right channel
                    
                    starter_sound = pygame.mixer.Sound(buffer)
                    
                    # Play the starter sound
                    starter_sound.set_volume(0.5)
                    pygame.mixer.Channel(15).play(starter_sound)
                    
                except Exception as e:
                    print(f"Error playing starter sound: {e}")
    
    def stop_engine(self) -> None:
        """Stop the engine."""
        if self.is_running:
            self.is_running = False
            self.rpm = 0.0
            self.idle_sound_playing = False
            print("Engine stopped")
            
            # Stop all sounds
            if pygame.mixer.get_init():
                for i in range(16):
                    pygame.mixer.Channel(i).stop()
    
    def get_metrics(self) -> Dict:
        """Get current engine metrics."""
        return {
            "rpm": self.rpm,
            "power_hp": self.total_power / 745.7,  # Convert W to hp
            "torque_nm": self.total_torque,
            "fuel_consumption_gps": self.fuel_consumption,
            "thermal_efficiency": self.thermal_efficiency,
            "coolant_temp": self.coolant_temp,
            "timing_wear": self.timing_chain_wear,
            "engine_load": self.engine_load,
            "is_running": self.is_running,
            "cylinder_pressures": [cyl.pressure for cyl in self.cylinders],
            "knock_detected": any(cyl.knock_detected for cyl in self.cylinders),
            "cylinder_wear": [cyl.wear for cyl in self.cylinders]
        }


class EngineVisualizer:
    def __init__(self, simulator: EngineSimulator, fig_size=(12, 9)):
        self.simulator = simulator
        
        # Set up the figure and subplots
        self.fig = plt.figure(figsize=fig_size)
        self.fig.suptitle("2D Engine Simulator", fontsize=16)
        
        # Engine animation (top left)
        self.ax_engine = plt.subplot2grid((3, 3), (0, 0), rowspan=2, colspan=2)
        self.ax_engine.set_aspect('equal')
        self.ax_engine.set_xlim(-150, 150)
        self.ax_engine.set_ylim(-150, 150)
        self.ax_engine.set_title("Engine Animation")
        self.ax_engine.set_axis_off()
        self.ax_engine.set_facecolor('black')  # Dark background for better contrast
        
        # Pressure plot (top right)
        self.ax_pressure = plt.subplot2grid((3, 3), (0, 2))
        self.ax_pressure.set_title("Cylinder Pressure")
        self.ax_pressure.set_xlabel("Crank Angle (°)")
        self.ax_pressure.set_ylabel("Pressure (bar)")
        self.ax_pressure.set_xlim(0, 720)
        self.ax_pressure.set_ylim(0, 100)
        
        # Metrics (middle right)
        self.ax_metrics = plt.subplot2grid((3, 3), (1, 2))
        self.ax_metrics.set_title("Engine Metrics")
        self.ax_metrics.set_axis_off()
        
        # PV diagram (bottom left)
        self.ax_pv = plt.subplot2grid((3, 3), (2, 0))
        self.ax_pv.set_title("P-V Diagram")
        self.ax_pv.set_xlabel("Volume (cc)")
        self.ax_pv.set_ylabel("Pressure (bar)")
        self.ax_pv.set_yscale('log')
        
        # Performance graph (bottom middle)
        self.ax_perf = plt.subplot2grid((3, 3), (2, 1))
        self.ax_perf.set_title("Performance Curves")
        self.ax_perf.set_xlabel("RPM")
        self.ax_perf.set_ylabel("Power (hp) / Torque (Nm)")
        
        # Reliability stats (bottom right)
        self.ax_reliability = plt.subplot2grid((3, 3), (2, 2))
        self.ax_reliability.set_title("Cylinder Wear")
        self.ax_reliability.set_xlim(0, 100)
        self.ax_reliability.set_ylim(0, self.simulator.config.num_cylinders)
        self.ax_reliability.set_xlabel("Wear (%)")
        self.ax_reliability.set_yticks(range(self.simulator.config.num_cylinders))
        self.ax_reliability.set_yticklabels([f"Cyl {i+1}" for i in range(self.simulator.config.num_cylinders)])
        
        # Data for plots
        self.crank_angles = []
        self.pressures = [[] for _ in range(simulator.config.num_cylinders)]
        self.volumes = [[] for _ in range(simulator.config.num_cylinders)]
        self.rpm_history = []
        self.power_history = []
        self.torque_history = []
        
        # Engine layout visualization components
        self.pistons = []
        self.cylinders = []
        self.crankshaft = None
        self.connecting_rods = []
        
        # Cylinder colors for better visualization
        self.cylinder_colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c', '#d35400', '#34495e']
        self.piston_colors = ['#5dade2', '#ec7063', '#58d68d', '#f5b041', '#af7ac5', '#48c9b0', '#dc7633', '#5d6d7e']
        
        # Create engine components based on layout
        self._create_engine_components()
        
        # Metrics text
        self.metrics_text = self.ax_metrics.text(0.5, 0.5, "", ha='center', va='center', fontsize=10)
        
        # Tight layout
        plt.tight_layout(rect=[0, 0, 1, 0.95])
        
    def _create_engine_components(self):
        """Create engine visualization components based on layout."""
        config = self.simulator.config
        num_cylinders = config.num_cylinders
        
        # Clear any existing components
        self.ax_engine.clear()
        self.ax_engine.set_aspect('equal')
        self.ax_engine.set_xlim(-150, 150)
        self.ax_engine.set_ylim(-150, 150)
        self.ax_engine.set_title("Engine Animation")
        self.ax_engine.set_axis_off()
        self.ax_engine.set_facecolor('black')  # Dark background for better contrast
        
        # Reset component lists
        self.pistons = []
        self.cylinders = []
        self.connecting_rods = []
        
        # Position cylinders based on engine layout
        cylinder_positions = []
        cylinder_angles = []
        
        if config.layout == EngineLayout.INLINE:
            # Inline engine - cylinders in a row
            spacing = min(25, 150 / max(num_cylinders, 1))
            for i in range(num_cylinders):
                x = i * spacing - (num_cylinders-1) * spacing / 2
                y = 60  # Fixed height for all cylinders
                cylinder_positions.append((x, y))
                cylinder_angles.append(0)  # All vertical
            
        elif config.layout in [EngineLayout.V60, EngineLayout.V90]:
            # V engine - cylinders in two banks
            v_angle = 60 if config.layout == EngineLayout.V60 else 90
            spacing = min(25, 150 / max(num_cylinders//2, 1))
            
            # Calculate positions for each bank
            for i in range(num_cylinders):
                bank = i % 2  # 0 = left bank, 1 = right bank
                position = i // 2  # Position within bank
                
                # Calculate x position (shared between banks in same position)
                x = position * spacing - (num_cylinders//2-0.5) * spacing / 2
                
                # Calculate y position and angle based on bank
                if bank == 0:  # Left bank
                    y = 40
                    angle = v_angle / 2
                else:  # Right bank
                    y = 40
                    angle = -v_angle / 2
                
                cylinder_positions.append((x, y))
                cylinder_angles.append(angle)
            
        elif config.layout in [EngineLayout.BOXER, EngineLayout.FLAT]:
            # Boxer/Flat engine - cylinders on opposite sides
            spacing = min(25, 150 / max(num_cylinders//2, 1))
            
            # Calculate positions for each bank
            for i in range(num_cylinders):
                bank = i % 2  # 0 = left bank, 1 = right bank
                position = i // 2  # Position within bank
                
                # Calculate x position (shared between banks in same position)
                x = position * spacing - (num_cylinders//2-0.5) * spacing / 2
                
                # Calculate y position and angle based on bank
                if bank == 0:  # Left bank
                    y = 30
                    angle = 90  # Horizontal, pointing left
                else:  # Right bank
                    y = -30
                    angle = -90  # Horizontal, pointing right
                
                cylinder_positions.append((x, y))
                cylinder_angles.append(angle)
        
        # Create crankshaft
        crankshaft_center = (0, 0)
        self.crankshaft = patches.Circle(crankshaft_center, 10, facecolor='#95a5a6', edgecolor='#7f8c8d')
        self.ax_engine.add_patch(self.crankshaft)
        
        # Create cylinders and pistons with proper orientation and colors
        for i in range(num_cylinders):
            x, y = cylinder_positions[i]
            angle = cylinder_angles[i]
            
            # Assign color based on cylinder index (cycle through colors)
            cylinder_color = self.cylinder_colors[i % len(self.cylinder_colors)]
            piston_color = self.piston_colors[i % len(self.piston_colors)]
            
            # Convert angle to radians for calculations
            angle_rad = math.radians(angle)
            
            # Calculate cylinder dimensions based on angle
            # For non-vertical cylinders, adjust width and height
            if angle == 0:  # Vertical cylinder
                width = 20
                height = 60
                x_offset = -width/2
                y_offset = 0
            elif abs(angle) == 90:  # Horizontal cylinder
                width = 60
                height = 20
                x_offset = 0 if angle > 0 else -width  # Left or right orientation
                y_offset = -height/2
            else:  # Angled cylinder
                width = 20
                height = 60
                # Adjust position so cylinder base is at the right spot
                x_offset = -width/2
                y_offset = 0
                
            # Create the cylinder with a cool translucent look
            cylinder = patches.Rectangle((x+x_offset, y+y_offset), width, height, 
                                         angle=angle, fill=True, 
                                         facecolor=cylinder_color, edgecolor='black',
                                         alpha=0.7, linewidth=2)
            self.ax_engine.add_patch(cylinder)
            self.cylinders.append((cylinder, (x, y), angle))
            
            # Calculate piston dimensions and position
            if angle == 0:  # Vertical cylinders
                piston_width = 16
                piston_height = 10
                piston_x = x - piston_width/2
                piston_y = y + 30
            elif abs(angle) == 90:  # Horizontal cylinders
                piston_width = 10
                piston_height = 16
                if angle == 90:  # Left facing
                    piston_x = x - 30 - piston_width
                    piston_y = y - piston_height/2
                else:  # Right facing
                    piston_x = x + 30
                    piston_y = y - piston_height/2
            else:  # Angled cylinders
                piston_width = 16
                piston_height = 10
                # Position will be updated during animation
                piston_x = x - piston_width/2
                piston_y = y + 30
                
            # Create the piston with color
            piston = patches.Rectangle((piston_x, piston_y), piston_width, piston_height,
                                     angle=angle, facecolor=piston_color, edgecolor='black')
            self.ax_engine.add_patch(piston)
            self.pistons.append((piston, (x, y), angle))
            
            # Create connecting rod (position will be updated during animation)
            rod = patches.Rectangle((0, 0), 5, 40, facecolor='#f1c40f', edgecolor='#e67e22')
            self.ax_engine.add_patch(rod)
            self.connecting_rods.append(rod)
            
        # Add decoration to make it look more like an engine block
        if config.layout in [EngineLayout.V60, EngineLayout.V90]:
            # For V-engines, add a block beneath the V
            block_width = min(150, num_cylinders * spacing + 20)
            block_height = 30
            block = patches.Rectangle((-block_width/2, -15), block_width, block_height, 
                                      facecolor='#7f8c8d', edgecolor='black')
            self.ax_engine.add_patch(block)
            
            # Add cooling fins or other details
            fin_height = 5
            fin_spacing = block_width / 8
            for i in range(7):
                fin_x = -block_width/2 + (i+1) * fin_spacing
                fin = patches.Rectangle((fin_x, -15-fin_height), 2, fin_height, 
                                        facecolor='#95a5a6', edgecolor=None)
                self.ax_engine.add_patch(fin)
                
        elif config.layout == EngineLayout.INLINE:
            # For inline engines, add a block beneath the cylinders
            block_width = min(150, num_cylinders * spacing + 20)
            block_height = 20
            block = patches.Rectangle((-block_width/2, 15), block_width, block_height, 
                                      facecolor='#7f8c8d', edgecolor='black')
            self.ax_engine.add_patch(block)
            
        elif config.layout in [EngineLayout.BOXER, EngineLayout.FLAT]:
            # For boxer/flat engines, add a central section
            block_width = 40
            block_height = 40
            block = patches.Rectangle((-block_width/2, -block_height/2), block_width, block_height, 
                                      facecolor='#7f8c8d', edgecolor='black')
            self.ax_engine.add_patch(block)
    
    def update_visualization(self):
        """Update all visualization components."""
        # Get current metrics
        metrics = self.simulator.get_metrics()
        
        # Update engine component positions
        self._update_engine_animation()
        
        # Update pressure plot data
        current_cylinder = 0  # Focus on first cylinder for pressure plot
        self.crank_angles.append(self.simulator.crank_angle)
        
        for i, cylinder in enumerate(self.simulator.cylinders):
            self.pressures[i].append(cylinder.pressure)
            self.volumes[i].append(cylinder.volume)
        
        # Keep only last 720 degrees of data
        if len(self.crank_angles) > 720:
            self.crank_angles = self.crank_angles[-720:]
            for i in range(len(self.pressures)):
                self.pressures[i] = self.pressures[i][-720:]
                self.volumes[i] = self.volumes[i][-720:]
        
        # Update pressure plot
        self.ax_pressure.clear()
        self.ax_pressure.set_title("Cylinder Pressure")
        self.ax_pressure.set_xlabel("Crank Angle (°)")
        self.ax_pressure.set_ylabel("Pressure (bar)")
        self.ax_pressure.set_xlim(0, 720)
        self.ax_pressure.set_ylim(0, max(max(p) for p in self.pressures) * 1.1 if self.pressures[0] else 100)
        
        for i, pressure in enumerate(self.pressures):
            if i == current_cylinder:
                self.ax_pressure.plot(self.crank_angles, pressure, 'r-', linewidth=2)
            else:
                self.ax_pressure.plot(self.crank_angles, pressure, 'b-', alpha=0.2)
                
        # Mark TDC positions
        self.ax_pressure.axvline(x=0, color='gray', linestyle='--', alpha=0.5)
        self.ax_pressure.axvline(x=360, color='gray', linestyle='--', alpha=0.5)
        
        # Update PV diagram
        self.ax_pv.clear()
        self.ax_pv.set_title("P-V Diagram")
        self.ax_pv.set_xlabel("Volume (cc)")
        self.ax_pv.set_ylabel("Pressure (bar)")
        self.ax_pv.set_yscale('log')
        
        if self.volumes[current_cylinder]:
            self.ax_pv.plot(self.volumes[current_cylinder], self.pressures[current_cylinder], 'g-')
        
        # Update performance history
        self.rpm_history.append(metrics["rpm"])
        self.power_history.append(metrics["power_hp"])
        self.torque_history.append(metrics["torque_nm"])
        
        if len(self.rpm_history) > 100:
            self.rpm_history = self.rpm_history[-100:]
            self.power_history = self.power_history[-100:]
            self.torque_history = self.torque_history[-100:]
        
        # Update performance plot
        self.ax_perf.clear()
        self.ax_perf.set_title("Performance")
        self.ax_perf.set_xlabel("RPM")
        self.ax_perf.set_ylabel("Power (hp) / Torque (Nm)")
        
        if self.rpm_history:
            self.ax_perf.plot(self.rpm_history, self.power_history, 'r-', label='Power (hp)')
            self.ax_perf.plot(self.rpm_history, self.torque_history, 'b-', label='Torque (Nm)')
            self.ax_perf.legend()
        
        # Update metrics text
        metrics_str = f"Engine Status: {'Running' if metrics['is_running'] else 'Stopped'}\n"
        metrics_str += f"RPM: {metrics['rpm']:.0f}\n"
        metrics_str += f"Power: {metrics['power_hp']:.1f} hp\n"
        metrics_str += f"Torque: {metrics['torque_nm']:.1f} Nm\n"
        metrics_str += f"Engine Load: {metrics['engine_load']*100:.1f}%\n"
        metrics_str += f"Fuel Consumption: {metrics['fuel_consumption_gps']*3600/1000:.2f} kg/h\n"
        metrics_str += f"Thermal Efficiency: {metrics['thermal_efficiency']:.1f}%\n"
        metrics_str += f"Coolant Temp: {metrics['coolant_temp']:.1f}°C\n"
        metrics_str += f"Knock: {'YES' if metrics['knock_detected'] else 'No'}\n"
        
        self.metrics_text.set_text(metrics_str)
        
        # Update reliability bars
        self.ax_reliability.clear()
        self.ax_reliability.set_title("Cylinder Wear")
        self.ax_reliability.set_xlim(0, 100)
        self.ax_reliability.set_ylim(0, self.simulator.config.num_cylinders)
        self.ax_reliability.set_xlabel("Wear (%)")
        self.ax_reliability.set_yticks(range(self.simulator.config.num_cylinders))
        self.ax_reliability.set_yticklabels([f"Cyl {i+1}" for i in range(self.simulator.config.num_cylinders)])
        
        # Add horizontal bars for each cylinder's wear
        for i, wear in enumerate(metrics["cylinder_wear"]):
            color = 'green' if wear < 30 else 'yellow' if wear < 70 else 'red'
            self.ax_reliability.barh(i, wear, color=color)
            
        # Update figure
        self.fig.canvas.draw()
    
    def _update_engine_animation(self):
        """Update the engine animation components."""
        crank_angle = math.radians(self.simulator.crank_angle % 360)
        stroke = self.simulator.config.stroke / 2  # Radius of crankshaft rotation
        
        # Update piston and connecting rod positions for each cylinder
        for i, cylinder in enumerate(self.simulator.cylinders):
            if i >= len(self.pistons):
                continue  # Skip if we don't have a matching piston
                
            # Get cylinder position and angle
            piston, (cyl_x, cyl_y), angle = self.pistons[i]
            
            # Calculate local crank angle for this cylinder
            local_angle = self.simulator.crank_angle + cylinder.angle_offset
            local_angle_rad = math.radians(local_angle % 360)
            
            # Calculate piston position using the simulator's method
            piston_pos = cylinder.piston_position
            
            # Get crankpin position relative to center
            crankpin_x = stroke * math.cos(local_angle_rad)
            crankpin_y = stroke * math.sin(local_angle_rad)
            
            # Handle different cylinder orientations
            angle_rad = math.radians(angle)
            
            if angle == 0:  # Vertical cylinders
                # Piston moves up and down
                piston_y = cyl_y + 30 * (1 - piston_pos * 2)  # Scale to get proper motion range
                piston.set_y(piston_y)
                
                # Rod coordinates for vertical cylinder
                rod_angle = math.atan2(piston_y + 5 - crankpin_y, cyl_x - crankpin_x)
                rod_length = math.sqrt((piston_y + 5 - crankpin_y)**2 + (cyl_x - crankpin_x)**2)
                
            elif angle == 90:  # Horizontal left-facing cylinders
                # Piston moves left and right
                piston_x = cyl_x - 30 - 10 - (30 * piston_pos * 2)
                piston.set_x(piston_x)
                
                # Rod coordinates for horizontal cylinder
                rod_angle = math.atan2(cyl_y - crankpin_y, piston_x + 10 - crankpin_x)
                rod_length = math.sqrt((cyl_y - crankpin_y)**2 + (piston_x + 10 - crankpin_x)**2)
                
            elif angle == -90:  # Horizontal right-facing cylinders
                # Piston moves left and right
                piston_x = cyl_x + 30 + (30 * piston_pos * 2)
                piston.set_x(piston_x)
                
                # Rod coordinates for horizontal cylinder
                rod_angle = math.atan2(cyl_y - crankpin_y, piston_x - crankpin_x)
                rod_length = math.sqrt((cyl_y - crankpin_y)**2 + (piston_x - crankpin_x)**2)
                
            else:  # Angled cylinders (V-engines)
                # Calculate position along the cylinder axis
                offset_distance = 30 * (1 - piston_pos * 2)
                
                # Calculate x and y components of the offset
                dx = offset_distance * math.sin(angle_rad)
                dy = offset_distance * math.cos(angle_rad)
                
                # Update piston position - offset from cylinder base
                if angle > 0:  # Left bank
                    piston_x = cyl_x - 8 - dx
                    piston_y = cyl_y + dy
                else:  # Right bank
                    piston_x = cyl_x - 8 + dx
                    piston_y = cyl_y + dy
                
                piston.set_x(piston_x)
                piston.set_y(piston_y)
                
                # Rod coordinates for angled cylinder
                piston_center_x = piston_x + 8  # Half of piston width
                piston_center_y = piston_y + 5  # Half of piston height
                
                rod_angle = math.atan2(piston_center_y - crankpin_y, piston_center_x - crankpin_x)
                rod_length = math.sqrt((piston_center_y - crankpin_y)**2 + (piston_center_x - crankpin_x)**2)
            
            # Update connecting rod
            # Set properties for the connecting rod
            self.connecting_rods[i].set_xy((0, -2.5))
            self.connecting_rods[i].set_width(rod_length)
            self.connecting_rods[i].set_height(5)
            self.connecting_rods[i].set_angle(math.degrees(rod_angle))
            self.connecting_rods[i].set_xy((crankpin_x, crankpin_y))
            
            # Change color of cylinder during combustion phase
            if cylinder.is_combustion:
                # Combustion color (orange-red glow)
                glow_intensity = cylinder.combustion_progress
                # Mix between normal color and combustion color
                cylinder_patch = self.cylinders[i][0]
                # Get the base color from our color array
                base_color = self.cylinder_colors[i % len(self.cylinder_colors)]
                # Parse the hex color
                base_r = int(base_color[1:3], 16) / 255.0
                base_g = int(base_color[3:5], 16) / 255.0
                base_b = int(base_color[5:7], 16) / 255.0
                
                # Create a fiery glow by mixing with red/orange
                r = min(1.0, base_r + glow_intensity * 0.7)
                g = min(1.0, base_g + glow_intensity * 0.4)
                b = max(0.0, base_b - glow_intensity * 0.2)
                
                cylinder_patch.set_facecolor((r, g, b, 0.7))
            else:
                # Reset to normal color
                cylinder_patch = self.cylinders[i][0]
                base_color = self.cylinder_colors[i % len(self.cylinder_colors)]
                cylinder_patch.set_facecolor(base_color)
        
        # Rotate crankshaft
        self.crankshaft.set_transform(self.ax_engine.transData + 
                                      plt.matplotlib.transforms.Affine2D().rotate(crank_angle))


class EngineSimulatorUI:
    def __init__(self):
        # Create initial engine config
        self.config = EngineConfig()
        
        # Create simulator
        self.simulator = EngineSimulator(self.config)
        
        # Create visualizer
        self.visualizer = EngineVisualizer(self.simulator)
        
        # Set up UI elements
        self._setup_ui()
        
        # Animation
        self.animation = None
        
    def _setup_ui(self):
        """Set up UI controls."""
        # Add toolbar at bottom
        self.toolbar_ax = plt.axes([0.1, 0.01, 0.8, 0.03])
        self.throttle_slider = Slider(
            self.toolbar_ax, 'Throttle', 0.0, 1.0, valinit=0.2
        )
        self.throttle_slider.on_changed(self._on_throttle_change)
        
        # Add buttons
        self.button_ax_start = plt.axes([0.1, 0.06, 0.2, 0.03])
        self.button_ax_stop = plt.axes([0.35, 0.06, 0.2, 0.03])
        self.button_ax_config = plt.axes([0.6, 0.06, 0.25, 0.03])
        
        self.start_button = Button(self.button_ax_start, 'Start Engine')
        self.stop_button = Button(self.button_ax_stop, 'Stop Engine')
        self.config_button = Button(self.button_ax_config, 'Engine Configuration')
        
        self.start_button.on_clicked(self._on_start)
        self.stop_button.on_clicked(self._on_stop)
        self.config_button.on_clicked(self._show_config_dialog)
    
    def _on_throttle_change(self, val):
        """Handle throttle slider change."""
        self.simulator.set_throttle(val)
    
    def _on_start(self, event):
        """Handle start button click."""
        self.simulator.start_engine()
    
    def _on_stop(self, event):
        """Handle stop button click."""
        self.simulator.stop_engine()
    
    def _show_config_dialog(self, event):
        """Show engine configuration dialog."""
        # Stop animation temporarily
        if self.animation is not None:
            self.animation.event_source.stop()
        
        # Create a dialog figure
        fig_dialog = plt.figure(figsize=(6, 8))
        fig_dialog.suptitle("Engine Configuration", fontsize=16)
        
        # Dictionary to store new values
        new_config = {k: v for k, v in self.config.__dict__.items()}
        
        # Set up widgets
        axcolor = 'lightgoldenrodyellow'
        
        # Engine layout dropdown - Use a proper dropdown with good spacing
        ax_layout = plt.axes([0.25, 0.9, 0.65, 0.03])
        layout_options = [e.name for e in EngineLayout]
        
        # Use a button-based layout selector instead of radio buttons
        layout_idx = layout_options.index(self.config.layout.name)
        layout_selector = plt.axes([0.25, 0.85, 0.65, 0.04])
        layout_dropdown = plt.Button(layout_selector, self.config.layout.name)
        layout_text = plt.figtext(0.1, 0.87, "Layout:", fontsize=10)
        
        # Function to cycle through layouts when button is clicked
        def on_layout_click(event):
            nonlocal layout_idx
            layout_idx = (layout_idx + 1) % len(layout_options)
            layout_dropdown.label.set_text(layout_options[layout_idx])
            new_config['layout'] = EngineLayout[layout_options[layout_idx]]
            fig_dialog.canvas.draw_idle()
        
        layout_dropdown.on_clicked(on_layout_click)
        
        # Number of cylinders
        ax_cylinders = plt.axes([0.25, 0.78, 0.65, 0.03], facecolor=axcolor)
        cylinder_slider = Slider(
            ax_cylinders, '', 1, 16, valinit=self.config.num_cylinders, valstep=1
        )
        plt.figtext(0.1, 0.78, "Cylinders:", fontsize=10)
        
        # Bore
        ax_bore = plt.axes([0.25, 0.71, 0.65, 0.03], facecolor=axcolor)
        bore_slider = Slider(
            ax_bore, '', 60, 120, valinit=self.config.bore
        )
        plt.figtext(0.1, 0.71, "Bore (mm):", fontsize=10)
        
        # Stroke
        ax_stroke = plt.axes([0.25, 0.64, 0.65, 0.03], facecolor=axcolor)
        stroke_slider = Slider(
            ax_stroke, '', 60, 120, valinit=self.config.stroke
        )
        plt.figtext(0.1, 0.64, "Stroke (mm):", fontsize=10)
        
        # Compression ratio
        ax_compression = plt.axes([0.25, 0.57, 0.65, 0.03], facecolor=axcolor)
        compression_slider = Slider(
            ax_compression, '', 7.0, 15.0, valinit=self.config.compression_ratio
        )
        plt.figtext(0.1, 0.57, "Compression:", fontsize=10)
        
        # Ignition timing
        ax_timing = plt.axes([0.25, 0.50, 0.65, 0.03], facecolor=axcolor)
        timing_slider = Slider(
            ax_timing, '', 0.0, 45.0, valinit=self.config.ignition_timing
        )
        plt.figtext(0.1, 0.50, "Ignition (°BTDC):", fontsize=10)
        
        # Air-fuel ratio
        ax_afr = plt.axes([0.25, 0.43, 0.65, 0.03], facecolor=axcolor)
        afr_slider = Slider(
            ax_afr, '', 10.0, 20.0, valinit=self.config.air_fuel_ratio
        )
        plt.figtext(0.1, 0.43, "Air-Fuel Ratio:", fontsize=10)
        
        # Crankshaft type - Use a button instead of radio buttons
        crank_options = [c.name for c in CrankshaftType]
        crank_idx = crank_options.index(self.config.crankshaft_type.name)
        
        crank_selector = plt.axes([0.25, 0.36, 0.65, 0.04])
        crank_dropdown = plt.Button(crank_selector, self.config.crankshaft_type.name)
        plt.figtext(0.1, 0.38, "Crankshaft:", fontsize=10)
        
        # Function to cycle through crankshaft types
        def on_crank_click(event):
            nonlocal crank_idx
            crank_idx = (crank_idx + 1) % len(crank_options)
            crank_dropdown.label.set_text(crank_options[crank_idx])
            new_config['crankshaft_type'] = CrankshaftType[crank_options[crank_idx]]
            fig_dialog.canvas.draw_idle()
        
        crank_dropdown.on_clicked(on_crank_click)
        
        # Forced induction pressure
        ax_boost = plt.axes([0.25, 0.29, 0.65, 0.03], facecolor=axcolor)
        boost_slider = Slider(
            ax_boost, '', 1.0, 3.0, valinit=self.config.forced_induction_pressure
        )
        plt.figtext(0.1, 0.29, "Boost (bar):", fontsize=10)
        
        # Reliability
        ax_reliability = plt.axes([0.25, 0.22, 0.65, 0.03], facecolor=axcolor)
        reliability_slider = Slider(
            ax_reliability, '', 50, 100, valinit=self.config.reliability
        )
        plt.figtext(0.1, 0.22, "Reliability:", fontsize=10)
        
        # Apply and cancel buttons
        ax_apply = plt.axes([0.35, 0.12, 0.3, 0.05])
        apply_button = plt.Button(ax_apply, 'Apply Configuration')
        
        ax_cancel = plt.axes([0.35, 0.05, 0.3, 0.05])
        cancel_button = plt.Button(ax_cancel, 'Cancel')
        
        # Callback functions
        def on_apply(event):
            # Apply new configuration
            new_config['num_cylinders'] = int(cylinder_slider.val)
            new_config['bore'] = bore_slider.val
            new_config['stroke'] = stroke_slider.val
            new_config['compression_ratio'] = compression_slider.val
            new_config['ignition_timing'] = timing_slider.val
            new_config['air_fuel_ratio'] = afr_slider.val
            new_config['forced_induction_pressure'] = boost_slider.val
            new_config['reliability'] = reliability_slider.val
            
            # Create new config and recreate simulator
            try:
                self.config = EngineConfig(**new_config)
                self.simulator = EngineSimulator(self.config)
                self.visualizer = EngineVisualizer(self.simulator)
                self._setup_ui()
                
                # Restart animation
                if self.animation is not None:
                    self.animation.event_source.start()
                
                plt.close(fig_dialog)
            except ValueError as e:
                # Show error message
                plt.figtext(0.5, 0.2, str(e), fontsize=12, ha='center', color='red')
        
        def on_cancel(event):
            # Restart animation
            if self.animation is not None:
                self.animation.event_source.start()
            
            plt.close(fig_dialog)
        
        # Connect callbacks
        apply_button.on_clicked(on_apply)
        cancel_button.on_clicked(on_cancel)
        
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()
    
    def _update(self, frame):
        """Animation update function."""
        # Update simulation
        self.simulator.update()
        
        # Update visualization
        self.visualizer.update_visualization()
        
        return []
    
    def run(self):
        """Run the simulation."""
        # Set up animation
        self.animation = FuncAnimation(
            self.visualizer.fig, self._update, frames=None, 
            interval=20, blit=True
        )
        
        # Show plot
        plt.show()


def main():
    # Create and run the simulator UI
    ui = EngineSimulatorUI()
    
    # Print instructions
    print("=== 2D Engine Simulator ===")
    print("Controls:")
    print("  - Start Engine: Begin the simulation")
    print("  - Stop Engine: Stop the simulation")
    print("  - Engine Configuration: Modify engine parameters")
    print("  - Throttle Slider: Adjust engine power")
    print("")
    print("Visualization:")
    print("  - Engine Animation: Shows the moving parts")
    print("  - Cylinder Pressure: Pressure vs. crank angle")
    print("  - P-V Diagram: Pressure-volume relationship")
    print("  - Performance: Power and torque curves")
    print("  - Cylinder Wear: Monitors engine reliability")
    print("")
    print("Special Features:")
    print("  - Engine sound effects")
    print("  - Knock detection")
    print("  - Color changes during combustion")
    print("  - Real-time performance metrics")
    print("")
    
    ui.run()


if __name__ == "__main__":
    main()