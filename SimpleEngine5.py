import pygame
import sys
import math

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
FPS = 60
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
LIGHT_GRAY = (220, 220, 220)
DARK_GRAY = (100, 100, 100)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
PINK = (255, 182, 193)
CYAN = (0, 255, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 215, 0)

class Engine:
    def __init__(self):
        # Engine dimensions and positions
        self.center_x = WIDTH // 2
        self.center_y = HEIGHT // 2 + 50
        self.crank_radius = 70
        self.rod_length = 180
        self.piston_height = 70
        self.piston_width = 90
        self.cylinder_height = 250
        self.cylinder_width = 110
        self.valve_radius = 20
        self.valve_stem_length = 50
        self.valve_lift = 25
        # Add extra offset to move cylinder away from crankshaft
        self.cylinder_offset = 40
        
        # Min/max values for adjustable parameters
        self.min_rod_length = 120
        self.max_rod_length = 250
        self.min_bore = 80
        self.max_bore = 160
        
        # Engine state
        self.angle = 0
        self.running = False
        self.rpm = 60
        self.max_rpm = 300
        
        # Track the 4-stroke cycle (2 revolutions of crankshaft)
        self.cycle_angle = 0
        
        # Engine configuration
        self.engine_type = "Single"  # Can be "Single" or "Boxer" or "Multi"
        
        # For multi-cylinder engine
        self.num_cylinders = 1
        self.max_cylinders = 6  # Maximum number of cylinders
        
        # Create single cylinder components
        self.create_single_cylinder()
        
        # For boxer engine components (will be initialized when toggled)
        self.left_cylinder = None
        self.right_cylinder = None
        self.left_piston = None
        self.right_piston = None
        self.left_connecting_rod = None
        self.right_connecting_rod = None
        self.left_intake_valve = None
        self.left_exhaust_valve = None
        self.right_intake_valve = None
        self.right_exhaust_valve = None
        
        # For multi-cylinder engine (array of components)
        self.cylinders = []
        self.pistons = []
        self.connecting_rods = []
        self.intake_valves = []
        self.exhaust_valves = []
        self.crank_angles = []  # Store crank angle for each cylinder
    
    def create_single_cylinder(self):
        """Initialize the single cylinder components"""
        self.crankshaft = Crankshaft(self.center_x, self.center_y, self.crank_radius)
        self.connecting_rod = ConnectingRod(self.crank_radius, self.rod_length)
        self.piston = Piston(self.piston_width, self.piston_height)
        
        # Position cylinder further away from crankshaft and higher
        cyl_y = self.center_y - self.rod_length - self.crank_radius - self.cylinder_offset - 50
        self.cylinder = Cylinder(self.center_x, cyl_y, self.cylinder_width, self.cylinder_height)
        
        # Create valves
        valve_y = self.center_y - self.rod_length - self.crank_radius - 100 - self.cylinder_offset
        self.intake_valve = Valve(self.center_x - 30, valve_y, 
                                 self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=True)
        self.exhaust_valve = Valve(self.center_x + 30, valve_y, 
                                  self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=False)
        
        # Display values for UI elements
        self.rod_length_display = self.rod_length
        self.bore_size_display = self.cylinder_width
    
    def create_boxer_engine(self):
        """Initialize the boxer engine components"""
        # Create crankshaft (shared between cylinders)
        self.crankshaft = Crankshaft(self.center_x, self.center_y, self.crank_radius)
        
        # Calculate cylinder distances from center (horizontal placement)
        left_offset = self.rod_length + self.cylinder_offset
        right_offset = self.rod_length + self.cylinder_offset
        
        # Create left cylinder (horizontal, facing right toward crankshaft)
        left_x = self.center_x - left_offset
        self.left_cylinder = Cylinder(left_x, self.center_y, self.cylinder_height, self.cylinder_width, angle=90, boxer_style=True)
        self.left_piston = Piston(self.piston_height, self.piston_width) # Swap height and width for horizontal orientation
        self.left_connecting_rod = ConnectingRod(self.crank_radius, self.rod_length)
        
        # Create left cylinder valves
        valve_x_offset = 30
        self.left_intake_valve = Valve(left_x, self.center_y - valve_x_offset, 
                                      self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                      is_intake=True, angle=90, color=CYAN)  # Horizontal valves
        self.left_exhaust_valve = Valve(left_x, self.center_y + valve_x_offset, 
                                       self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                       is_intake=False, angle=90, color=YELLOW)  # Horizontal valves
        
        # Create right cylinder (horizontal, facing left toward crankshaft)
        right_x = self.center_x + right_offset
        self.right_cylinder = Cylinder(right_x, self.center_y, self.cylinder_height, self.cylinder_width, angle=-90, boxer_style=True)
        self.right_piston = Piston(self.piston_height, self.piston_width) # Swap height and width for horizontal orientation
        self.right_connecting_rod = ConnectingRod(self.crank_radius, self.rod_length)
        
        # Create right cylinder valves
        self.right_intake_valve = Valve(right_x, self.center_y - valve_x_offset, 
                                      self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                      is_intake=True, angle=-90, color=CYAN)  # Horizontal valves
        self.right_exhaust_valve = Valve(right_x, self.center_y + valve_x_offset, 
                                       self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                       is_intake=False, angle=-90, color=YELLOW)  # Horizontal valves
    
    def create_multi_cylinder(self):
        """Initialize a multi-cylinder inline engine"""
        # Start with a single cylinder
        self.num_cylinders = 1
        self.cylinders = []
        self.pistons = []
        self.connecting_rods = []
        self.intake_valves = []
        self.exhaust_valves = []
        self.crank_angles = []
        
        # Create crankshaft
        self.crankshaft = Crankshaft(self.center_x, self.center_y, self.crank_radius)
        
        # Add first cylinder (will add more when button is pressed)
        self.add_cylinder()
    
    def add_cylinder(self):
        """Add a new cylinder to the multi-cylinder engine"""
        if self.num_cylinders >= self.max_cylinders:
            return  # Maximum number of cylinders reached
        
        # Calculate z-offset for the new cylinder
        z_offset = self.num_cylinders - 1  # First cylinder has z=0
        
        # Calculate transparency for the new cylinder (20% more transparent than the one in front)
        transparency = max(0, 255 - (z_offset * 50))  # 20% = ~50 in 0-255 range
        
        # Calculate crank angle for the new cylinder
        if self.num_cylinders == 1:
            # First cylinder at 0 degrees
            crank_angle = 0
        elif self.num_cylinders == 2:
            # Second cylinder at 180 degrees (flat-plane)
            crank_angle = 180
        else:
            # Additional cylinders spaced evenly (cross-plane)
            # For 3+ cylinders: 0, 240, 120, etc. (720° ÷ num_cylinders for 4-stroke)
            crank_angle = ((self.num_cylinders - 1) * (720 / self.num_cylinders)) % 360
        
        self.crank_angles.append(crank_angle)
        
        # Create cylinder components with transparency
        cyl_y = self.center_y - self.rod_length - self.crank_radius - self.cylinder_offset - 50
        cylinder = Cylinder(self.center_x, cyl_y, self.cylinder_width, self.cylinder_height, 
                          angle=0, z_index=z_offset, transparency=transparency)
        
        # Create piston
        piston = Piston(self.piston_width, self.piston_height, z_index=z_offset, transparency=transparency)
        
        # Create connecting rod
        connecting_rod = ConnectingRod(self.crank_radius, self.rod_length, 
                                     z_index=z_offset, transparency=transparency)
        
        # Create valves
        valve_y = self.center_y - self.rod_length - self.crank_radius - 100 - self.cylinder_offset
        intake_valve = Valve(self.center_x - 30, valve_y, 
                           self.valve_radius, self.valve_stem_length, self.valve_lift, 
                           is_intake=True, angle=0, z_index=z_offset, transparency=transparency)
        exhaust_valve = Valve(self.center_x + 30, valve_y, 
                            self.valve_radius, self.valve_stem_length, self.valve_lift, 
                            is_intake=False, angle=0, z_index=z_offset, transparency=transparency)
        
        # Add components to arrays
        self.cylinders.append(cylinder)
        self.pistons.append(piston)
        self.connecting_rods.append(connecting_rod)
        self.intake_valves.append(intake_valve)
        self.exhaust_valves.append(exhaust_valve)
        
        # Increment cylinder count
        self.num_cylinders += 1
    
    def set_rod_length(self, length):
        """Update connecting rod length and adjust related components"""
        self.rod_length = length
        # Need to update component positions based on engine type
        if self.engine_type == "Single":
            self.update_single_cylinder()
        elif self.engine_type == "Boxer":
            self.update_boxer_engine()
        else:
            self.update_multi_cylinder()
    
    def set_bore_size(self, bore):
        """Update piston and cylinder bore (width)"""
        self.cylinder_width = bore
        self.piston_width = bore - 20  # Piston slightly smaller than bore
        # Update components that depend on these values
        if self.engine_type == "Single":
            self.update_single_cylinder()
        elif self.engine_type == "Boxer":
            self.update_boxer_engine()
        else:
            self.update_multi_cylinder()
    
    def toggle_engine_type(self):
        """Toggle between engine configurations"""
        if self.engine_type == "Single":
            self.engine_type = "Boxer"
            self.create_boxer_engine()
        elif self.engine_type == "Boxer":
            self.engine_type = "Multi"
            self.create_multi_cylinder()
        else:
            self.engine_type = "Single"
            self.create_single_cylinder()
    
    def update_single_cylinder(self):
        """Update the single cylinder components after parameter changes"""
        # Update cylinder position
        cyl_y = self.center_y - self.rod_length - self.crank_radius - self.cylinder_offset - 50
        self.cylinder = Cylinder(self.center_x, cyl_y, self.cylinder_width, self.cylinder_height)
        
        # Update piston size
        self.piston = Piston(self.piston_width, self.piston_height)
        
        # Update valve positions
        valve_y = self.center_y - self.rod_length - self.crank_radius - 100 - self.cylinder_offset
        self.intake_valve = Valve(self.center_x - 30, valve_y, 
                                 self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=True)
        self.exhaust_valve = Valve(self.center_x + 30, valve_y, 
                                  self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=False)
        
        # Update connecting rod
        self.connecting_rod = ConnectingRod(self.crank_radius, self.rod_length)
    
    def update_boxer_engine(self):
        """Update the boxer engine components after parameter changes"""
        # Recalculate offsets based on new rod length or bore size
        left_offset = self.rod_length + self.cylinder_offset
        right_offset = self.rod_length + self.cylinder_offset
        
        # Update left cylinder
        left_x = self.center_x - left_offset
        self.left_cylinder = Cylinder(left_x, self.center_y, self.cylinder_height, self.cylinder_width, angle=90, boxer_style=True)
        self.left_piston = Piston(self.piston_height, self.piston_width)  # Swap dimensions for horizontal
        self.left_connecting_rod = ConnectingRod(self.crank_radius, self.rod_length)
        
        # Update left cylinder valves
        valve_x_offset = 30
        self.left_intake_valve = Valve(left_x, self.center_y - valve_x_offset, 
                                      self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                      is_intake=True, angle=90, color=CYAN)
        self.left_exhaust_valve = Valve(left_x, self.center_y + valve_x_offset, 
                                       self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                       is_intake=False, angle=90, color=YELLOW)
        
        # Update right cylinder
        right_x = self.center_x + right_offset
        self.right_cylinder = Cylinder(right_x, self.center_y, self.cylinder_height, self.cylinder_width, angle=-90, boxer_style=True)
        self.right_piston = Piston(self.piston_height, self.piston_width)  # Swap dimensions for horizontal
        self.right_connecting_rod = ConnectingRod(self.crank_radius, self.rod_length)
        
        # Update right cylinder valves
        self.right_intake_valve = Valve(right_x, self.center_y - valve_x_offset, 
                                      self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                      is_intake=True, angle=-90, color=CYAN)
        self.right_exhaust_valve = Valve(right_x, self.center_y + valve_x_offset, 
                                       self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                       is_intake=False, angle=-90, color=YELLOW)
    
    def update_multi_cylinder(self):
        """Update multi-cylinder engine components after parameter changes"""
        # Recreate all cylinders with new parameters
        cyl_y = self.center_y - self.rod_length - self.crank_radius - self.cylinder_offset - 50
        valve_y = self.center_y - self.rod_length - self.crank_radius - 100 - self.cylinder_offset
        
        for i in range(len(self.cylinders)):
            # Calculate transparency based on z-index
            z_offset = i
            transparency = max(0, 255 - (z_offset * 50))
            
            # Update components
            self.cylinders[i] = Cylinder(self.center_x, cyl_y, self.cylinder_width, self.cylinder_height, 
                                       angle=0, z_index=z_offset, transparency=transparency)
            self.pistons[i] = Piston(self.piston_width, self.piston_height, 
                                   z_index=z_offset, transparency=transparency)
            self.connecting_rods[i] = ConnectingRod(self.crank_radius, self.rod_length, 
                                                  z_index=z_offset, transparency=transparency)
            self.intake_valves[i] = Valve(self.center_x - 30, valve_y, 
                                        self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                        is_intake=True, angle=0, z_index=z_offset, transparency=transparency)
            self.exhaust_valves[i] = Valve(self.center_x + 30, valve_y, 
                                         self.valve_radius, self.valve_stem_length, self.valve_lift, 
                                         is_intake=False, angle=0, z_index=z_offset, transparency=transparency)
    
    def update_single_components(self, angle):
        """Update the single cylinder's components"""
        # Calculate crankshaft pin position
        crank_x = self.center_x + self.crank_radius * math.sin(angle)
        crank_y = self.center_y - self.crank_radius * math.cos(angle)
        
        # Update crankshaft
        self.crankshaft.update(angle)
        
        # Calculate horizontal offset from center
        x_offset = crank_x - self.center_x
        
        # Calculate vertical position using Pythagorean theorem to maintain fixed rod length
        # Take negative sqrt because piston is above crankshaft
        y_from_crank = -math.sqrt(self.rod_length**2 - x_offset**2)
        
        # Piston height from crankshaft center
        piston_y = crank_y + y_from_crank - self.cylinder_offset
        
        # Update connecting rod
        self.connecting_rod.update(crank_x, crank_y, self.center_x, piston_y)
        
        # Update piston
        self.piston.update(self.center_x, piston_y)
        
        # Calculate cycle position for valves
        cycle_pos = (self.cycle_angle) % 720
        
        # Update valves based on the 4-stroke cycle position
        # Intake valve timing with smoother animation
        if 0 <= cycle_pos < 180:
            # Open during intake stroke - sine wave for smooth movement
            intake_lift = math.sin(cycle_pos * math.pi / 180)
            self.intake_valve.update(lift_percentage=intake_lift)
        else:
            # Closed for the rest of the cycle
            self.intake_valve.update(lift_percentage=0)
        
        # Exhaust valve timing with smoother animation
        if 540 <= cycle_pos < 720:
            # Open during exhaust stroke - sine wave for smooth movement
            exhaust_lift = math.sin((cycle_pos - 540) * math.pi / 180)
            self.exhaust_valve.update(lift_percentage=exhaust_lift)
        else:
            # Closed for the rest of the cycle
            self.exhaust_valve.update(lift_percentage=0)
    
    def update_boxer_components(self, angle):
        """Update the boxer engine's components"""
        # Update crankshaft
        self.crankshaft.update(angle)
        
        # Boxer engines have cylinders 180° apart firing alternately
        
        # Left cylinder - calculate crankpin position
        left_crank_x = self.center_x + self.crank_radius * math.sin(angle)
        left_crank_y = self.center_y - self.crank_radius * math.cos(angle)
        
        # Right cylinder - uses 180° offset on crankshaft
        right_angle = (angle + math.pi) % (2 * math.pi)
        right_crank_x = self.center_x + self.crank_radius * math.sin(right_angle)
        right_crank_y = self.center_y - self.crank_radius * math.cos(right_angle)
        
        # Calculate piston positions for both cylinders
        # For left cylinder (moving horizontally on the left)
        left_offset = self.crank_radius * math.sin(angle)
        # The left piston's position is determined by the crankshaft angle
        # First calculate the horizontal distance from crank center to piston
        rod_effect = math.sqrt(self.rod_length**2 - math.pow(left_crank_y - self.center_y, 2))
        
        # Calculate exact left piston x position
        # It should be at center_x - rod_length - crank_offset with the piston movement
        # added/subtracted based on crank angle
        left_base_x = self.center_x - self.rod_length - self.cylinder_offset
        left_piston_x = left_base_x + (self.rod_length - rod_effect)
        
        # For right cylinder (moving horizontally on the right)
        right_offset = self.crank_radius * math.sin(right_angle)
        # Similarly calculate for right piston
        right_rod_effect = math.sqrt(self.rod_length**2 - math.pow(right_crank_y - self.center_y, 2))
        
        right_base_x = self.center_x + self.rod_length + self.cylinder_offset
        right_piston_x = right_base_x - (self.rod_length - right_rod_effect)
        
        # Update connecting rods from crankpin to piston pin
        self.left_connecting_rod.update(left_crank_x, left_crank_y, left_piston_x, self.center_y)
        self.right_connecting_rod.update(right_crank_x, right_crank_y, right_piston_x, self.center_y)
        
        # Update pistons - they move horizontally
        self.left_piston.update(left_piston_x, self.center_y, rotation=90)
        self.right_piston.update(right_piston_x, self.center_y, rotation=-90)
        
        # Update valve timing for both cylinders (180° out of phase)
        left_cycle = (self.cycle_angle) % 720
        right_cycle = (self.cycle_angle + 360) % 720  # 180° phase difference
        
        # Left cylinder valves
        if 0 <= left_cycle < 180:
            intake_lift = math.sin(left_cycle * math.pi / 180)
            self.left_intake_valve.update(lift_percentage=intake_lift)
        else:
            self.left_intake_valve.update(lift_percentage=0)
        
        if 540 <= left_cycle < 720:
            exhaust_lift = math.sin((left_cycle - 540) * math.pi / 180)
            self.left_exhaust_valve.update(lift_percentage=exhaust_lift)
        else:
            self.left_exhaust_valve.update(lift_percentage=0)
        
        # Right cylinder valves
        if 0 <= right_cycle < 180:
            intake_lift = math.sin(right_cycle * math.pi / 180)
            self.right_intake_valve.update(lift_percentage=intake_lift)
        else:
            self.right_intake_valve.update(lift_percentage=0)
        
        if 540 <= right_cycle < 720:
            exhaust_lift = math.sin((right_cycle - 540) * math.pi / 180)
            self.right_exhaust_valve.update(lift_percentage=exhaust_lift)
        else:
            self.right_exhaust_valve.update(lift_percentage=0)
    
    def update_multi_cylinder_components(self, angle):
        """Update multi-cylinder engine components"""
        # Update crankshaft
        self.crankshaft.update(angle)
        
        # Update each cylinder based on its crank angle
        for i in range(len(self.cylinders)):
            # Get crank angle for this cylinder
            crank_offset_deg = self.crank_angles[i]
            crank_offset_rad = math.radians(crank_offset_deg)
            
            # Calculate actual crank angle for this cylinder
            cylinder_angle = (angle + crank_offset_rad) % (2 * math.pi)
            
            # Calculate crankpin position for this cylinder
            crank_x = self.center_x + self.crank_radius * math.sin(cylinder_angle)
            crank_y = self.center_y - self.crank_radius * math.cos(cylinder_angle)
            
            # Calculate horizontal offset from center
            x_offset = crank_x - self.center_x
            
            # Calculate vertical position for piston
            y_from_crank = -math.sqrt(self.rod_length**2 - x_offset**2)
            piston_y = crank_y + y_from_crank - self.cylinder_offset
            
            # Update connecting rod
            self.connecting_rods[i].update(crank_x, crank_y, self.center_x, piston_y)
            
            # Update piston
            self.pistons[i].update(self.center_x, piston_y)
            
            # Calculate cycle position for this cylinder
            # 4-stroke cycle with appropriate phase
            cylinder_cycle = (self.cycle_angle + crank_offset_deg) % 720
            
            # Update valves
            # Intake valve
            if 0 <= cylinder_cycle < 180:
                intake_lift = math.sin(cylinder_cycle * math.pi / 180)
                self.intake_valves[i].update(lift_percentage=intake_lift)
            else:
                self.intake_valves[i].update(lift_percentage=0)
                
            # Exhaust valve
            if 540 <= cylinder_cycle < 720:
                exhaust_lift = math.sin((cylinder_cycle - 540) * math.pi / 180)
                self.exhaust_valves[i].update(lift_percentage=exhaust_lift)
            else:
                self.exhaust_valves[i].update(lift_percentage=0)
    
    def update(self, dt):
        if self.running:
            # Update angle based on rpm
            dtheta = 2 * math.pi * self.rpm / 60 * dt
            self.angle = (self.angle + dtheta) % (2 * math.pi)
            
            # Update the cycle angle (0-720 degrees for 4-stroke cycle)
            self.cycle_angle = (self.cycle_angle + dtheta * 180 / math.pi) % 720
            
            # Update components based on engine type
            if self.engine_type == "Single":
                self.update_single_components(self.angle)
            elif self.engine_type == "Boxer":
                self.update_boxer_components(self.angle)
            else:
                self.update_multi_cylinder_components(self.angle)
    
    def draw(self, screen):
        # Draw background
        pygame.draw.rect(screen, BLACK, (0, 0, WIDTH, HEIGHT))
        
        # Draw components based on engine type
        if self.engine_type == "Single":
            self.draw_single_cylinder(screen)
        elif self.engine_type == "Boxer":
            self.draw_boxer_engine(screen)
        else:
            self.draw_multi_cylinder(screen)
        
        # Draw engine info text
        font = pygame.font.SysFont('Arial', 20)
        rpm_text = font.render(f'RPM: {self.rpm:.0f}', True, WHITE)
        screen.blit(rpm_text, (10, 10))
        
        # Draw engine state
        state_text = font.render('Running' if self.running else 'Stopped', True, GREEN if self.running else RED)
        screen.blit(state_text, (10, 40))
        
        # Draw engine parameters
        rod_text = font.render(f'Rod Length: {self.rod_length:.0f} px', True, WHITE)
        screen.blit(rod_text, (10, 70))
        
        bore_text = font.render(f'Bore Size: {self.cylinder_width:.0f} px', True, WHITE)
        screen.blit(bore_text, (10, 100))
        
        # Display engine type
        engine_type_text = font.render(f'Engine Type: {self.engine_type}', True, WHITE)
        screen.blit(engine_type_text, (10, 130))
        
        # Display number of cylinders for multi-cylinder
        if self.engine_type == "Multi":
            cylinders_text = font.render(f'Cylinders: {self.num_cylinders-1}', True, WHITE)
            screen.blit(cylinders_text, (10, 160))
    
    def draw_single_cylinder(self, screen):
        # Draw cylinder
        self.cylinder.draw(screen)
        
        # Draw valves
        self.intake_valve.draw(screen)
        self.exhaust_valve.draw(screen)
        
        # Draw piston
        self.piston.draw(screen)
        
        # Draw connecting rod
        self.connecting_rod.draw(screen)
        
        # Draw crankshaft
        self.crankshaft.draw(screen)
    
    def draw_boxer_engine(self, screen):
        # Draw in the correct order for proper layering
        
        # Draw left cylinder components
        self.left_cylinder.draw(screen)
        self.left_intake_valve.draw(screen)
        self.left_exhaust_valve.draw(screen)
        self.left_piston.draw(screen)
        self.left_connecting_rod.draw(screen)
        
        # Draw crankshaft in center
        self.crankshaft.draw(screen)
        
        # Draw right cylinder components
        self.right_cylinder.draw(screen)
        self.right_intake_valve.draw(screen)
        self.right_exhaust_valve.draw(screen)
        self.right_piston.draw(screen)
        self.right_connecting_rod.draw(screen)
    
    def draw_multi_cylinder(self, screen):
        # Draw in reverse order (back to front) for proper depth
        
        # Draw crankshaft first (in center)
        self.crankshaft.draw(screen)
        
        # Draw cylinder components from back to front
        for i in range(len(self.cylinders) - 1, -1, -1):
            # Draw back cylinders first
            self.cylinders[i].draw(screen)
            self.intake_valves[i].draw(screen)
            self.exhaust_valves[i].draw(screen)
            self.pistons[i].draw(screen)
            self.connecting_rods[i].draw(screen)


class Crankshaft:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.angle = 0
        self.crank_pin_x = 0
        self.crank_pin_y = 0
        self.crank_weight_radius = radius * 0.8
        
    def update(self, angle):
        self.angle = angle
        # Update pin position with corrected angles
        # When angle is 0, crankpin points up (TDC)
        # When angle is π, crankpin points down (BDC)
        self.crank_pin_x = self.center_x + self.radius * math.sin(angle)
        self.crank_pin_y = self.center_y - self.radius * math.cos(angle)
        
    def draw(self, screen):
        # Draw main shaft
        pygame.draw.circle(screen, DARK_GRAY, (self.center_x, self.center_y), 20)
        
        # Draw crank arm
        pygame.draw.line(screen, DARK_GRAY, (self.center_x, self.center_y), 
                         (self.crank_pin_x, self.crank_pin_y), 12)
        
        # Draw counterweight (on the opposite side of crank pin)
        counterweight_angle = self.angle + math.pi
        counterweight_x = self.center_x + self.crank_weight_radius * math.sin(counterweight_angle)
        counterweight_y = self.center_y - self.crank_weight_radius * math.cos(counterweight_angle)
        
        # Draw counterweight as a thicker line with circle at the end
        pygame.draw.line(screen, GRAY, (self.center_x, self.center_y), 
                         (counterweight_x, counterweight_y), 25)
        pygame.draw.circle(screen, GRAY, (int(counterweight_x), int(counterweight_y)), 25)
        
        # Draw crank pin (where connecting rod attaches)
        pygame.draw.circle(screen, LIGHT_GRAY, (int(self.crank_pin_x), int(self.crank_pin_y)), 10)


class ConnectingRod:
    def __init__(self, crank_radius, length, z_index=0, transparency=255):
        self.length = length
        self.crank_end_x = 0
        self.crank_end_y = 0
        self.piston_end_x = 0
        self.piston_end_y = 0
        self.z_index = z_index
        self.transparency = transparency
        
    def update(self, crank_x, crank_y, piston_x, piston_y):
        self.crank_end_x = crank_x
        self.crank_end_y = crank_y
        self.piston_end_x = piston_x
        self.piston_end_y = piston_y
        
    def draw(self, screen):
        # Draw connecting rod
        pygame.draw.line(screen, WHITE, 
                        (int(self.crank_end_x), int(self.crank_end_y)), 
                        (int(self.piston_end_x), int(self.piston_end_y)), 10)
        
        # Draw big end (connection to crankshaft)
        pygame.draw.circle(screen, LIGHT_GRAY, 
                          (int(self.crank_end_x), int(self.crank_end_y)), 15)
        
        # Draw small end (connection to piston)
        pygame.draw.circle(screen, LIGHT_GRAY, 
                          (int(self.piston_end_x), int(self.piston_end_y)), 12)


class Piston:
    def __init__(self, width, height, z_index=0, transparency=255):
        self.width = width
        self.height = height
        self.x = 0
        self.y = 0
        self.rotation = 0  # Rotation in degrees for horizontal pistons
        self.z_index = z_index
        self.transparency = transparency
        
    def update(self, x, y, rotation=0):
        self.x = x - self.width // 2
        self.y = y - self.height // 2
        self.rotation = rotation
        
    def draw(self, screen):
        if self.rotation == 0:
            # Draw vertical piston body
            pygame.draw.rect(screen, WHITE, (self.x, self.y, self.width, self.height))
            
            # Draw piston pin (wrist pin)
            pygame.draw.circle(screen, BLACK, 
                              (int(self.x + self.width // 2), int(self.y + self.height // 2)), 8)
            
            # Draw piston crown detail
            pygame.draw.rect(screen, BLACK, 
                            (self.x + self.width//4, self.y + 5, self.width//2, 5))
            
            # Draw piston markings like in reference image
            for i in range(3):
                mark_x = self.x + self.width//2 - 10 + i*10
                pygame.draw.line(screen, BLACK, (mark_x, self.y), (mark_x, self.y + 10), 1)
        else:
            # For horizontal pistons (used in boxer engine)
            # Create a rotated surface for the piston
            piston_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
            piston_surface.fill((0, 0, 0, 0))  # Clear with transparent color
            
            # Draw piston body
            pygame.draw.rect(piston_surface, WHITE, (0, 0, self.width, self.height))
            
            # Draw piston pin (wrist pin) on the surface
            pygame.draw.circle(piston_surface, BLACK, 
                              (self.width // 2, self.height // 2), 8)
            
            # Draw piston markings on the surface
            pygame.draw.rect(piston_surface, BLACK, 
                            (self.width//4, 5, self.width//2, 5))
            
            # Add piston markings like in reference image
            for i in range(3):
                mark_y = self.height//2 - 10 + i*10
                pygame.draw.line(piston_surface, BLACK, 
                                (0, mark_y), (10, mark_y), 1)
            
            # Rotate and blit the piston surface
            if self.rotation == 90:
                # Left piston (rotated 90 degrees clockwise)
                rotated_piston = pygame.transform.rotate(piston_surface, -90)
            else:
                # Right piston (rotated 90 degrees counterclockwise)
                rotated_piston = pygame.transform.rotate(piston_surface, 90)
                
            rotated_rect = rotated_piston.get_rect(center=(self.x + self.width//2, self.y + self.height//2))
            screen.blit(rotated_piston, rotated_rect)


class Cylinder:
    def __init__(self, x, y, width, height, angle=0, z_index=0, transparency=255, boxer_style=False):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.wall_thickness = 12
        self.angle = angle  # Angle in degrees (0 = vertical, 90 = horizontal left, -90 = horizontal right)
        self.z_index = z_index
        self.transparency = transparency
        self.boxer_style = boxer_style  # Special flag for boxer engine cylinder styling
        
    def draw(self, screen):
        if self.angle == 0:
            # Draw vertical cylinder
            # Draw cylinder walls
            pygame.draw.line(screen, PINK, (self.x - self.width//2, self.y), 
                            (self.x - self.width//2, self.y + self.height), self.wall_thickness)
            pygame.draw.line(screen, PINK, (self.x + self.width//2, self.y),
                            (self.x + self.width//2, self.y + self.height), self.wall_thickness)
            
            # Draw cylinder extension pieces (T-shape at top)
            pygame.draw.rect(screen, PINK, 
                            (self.x - self.width//2 - 50, self.y, 50, self.wall_thickness * 2))
            pygame.draw.rect(screen, PINK, 
                            (self.x + self.width//2, self.y, 50, self.wall_thickness * 2))
        elif self.angle == 90:
            # Draw horizontal left-facing cylinder
            if self.boxer_style:
                # Enhanced boxer cylinder style like reference image
                # Draw main cylinder body (horizontal)
                pygame.draw.rect(screen, PINK, 
                               (self.x - self.height, self.y - self.width//2, 
                                self.height, self.width), 1)
                
                # Draw cylinder walls (horizontal)
                pygame.draw.line(screen, PINK, (self.x, self.y - self.width//2), 
                                (self.x - self.height, self.y - self.width//2), self.wall_thickness)
                pygame.draw.line(screen, PINK, (self.x, self.y + self.width//2),
                                (self.x - self.height, self.y + self.width//2), self.wall_thickness)
                
                # Draw cylinder extensions on the left end
                ext_length = 50
                pygame.draw.rect(screen, PINK, 
                                (self.x - self.height - ext_length, self.y - self.width//4, 
                                ext_length, self.wall_thickness * 2))
                pygame.draw.rect(screen, PINK, 
                                (self.x - self.height - ext_length, self.y + self.width//4 - self.wall_thickness*2, 
                                ext_length, self.wall_thickness * 2))
                
                # Draw cylinder head on right side (near crankshaft)
                pygame.draw.rect(screen, PINK, 
                               (self.x - self.wall_thickness, self.y - self.width//2 - ext_length//2, 
                                self.wall_thickness * 2, self.width + ext_length))
            else:
                # Standard horizontal cylinder
                pygame.draw.line(screen, PINK, (self.x, self.y - self.width//2), 
                                (self.x - self.height, self.y - self.width//2), self.wall_thickness)
                pygame.draw.line(screen, PINK, (self.x, self.y + self.width//2),
                                (self.x - self.height, self.y + self.width//2), self.wall_thickness)
                
                # Extension pieces
                pygame.draw.rect(screen, PINK, 
                                (self.x, self.y - self.width//2 - 50, 
                                self.wall_thickness * 2, 50))
                pygame.draw.rect(screen, PINK, 
                                (self.x, self.y + self.width//2, 
                                self.wall_thickness * 2, 50))
        elif self.angle == -90:
            # Draw horizontal right-facing cylinder
            if self.boxer_style:
                # Enhanced boxer cylinder style like reference image
                # Draw main cylinder body (horizontal)
                pygame.draw.rect(screen, PINK, 
                               (self.x, self.y - self.width//2, 
                                self.height, self.width), 1)
                
                # Draw cylinder walls (horizontal)
                pygame.draw.line(screen, PINK, (self.x, self.y - self.width//2), 
                                (self.x + self.height, self.y - self.width//2), self.wall_thickness)
                pygame.draw.line(screen, PINK, (self.x, self.y + self.width//2),
                                (self.x + self.height, self.y + self.width//2), self.wall_thickness)
                
                # Draw cylinder extensions on the right end
                ext_length = 50
                pygame.draw.rect(screen, PINK, 
                                (self.x + self.height, self.y - self.width//4, 
                                ext_length, self.wall_thickness * 2))
                pygame.draw.rect(screen, PINK, 
                                (self.x + self.height, self.y + self.width//4 - self.wall_thickness*2,
                                ext_length, self.wall_thickness * 2))
                
                # Draw cylinder head on left side (near crankshaft)
                pygame.draw.rect(screen, PINK, 
                               (self.x, self.y - self.width//2 - ext_length//2, 
                                self.wall_thickness * 2, self.width + ext_length))
            else:
                # Standard horizontal cylinder
                pygame.draw.line(screen, PINK, (self.x, self.y - self.width//2), 
                                (self.x + self.height, self.y - self.width//2), self.wall_thickness)
                pygame.draw.line(screen, PINK, (self.x, self.y + self.width//2),
                                (self.x + self.height, self.y + self.width//2), self.wall_thickness)
                
                # Extension pieces
                pygame.draw.rect(screen, PINK, 
                                (self.x, self.y - self.width//2 - 50, 
                                self.wall_thickness * 2, 50))
                pygame.draw.rect(screen, PINK, 
                                (self.x, self.y + self.width//2, 
                                self.wall_thickness * 2, 50))


class Valve:
    def __init__(self, x, y, radius, stem_length, max_lift, is_intake=True, angle=0, z_index=0, transparency=255, color=None):
        self.x = x
        self.y = y
        self.radius = radius
        self.stem_length = stem_length
        self.max_lift = max_lift
        self.is_intake = is_intake
        self.lift_percentage = 0
        self.current_lift = 0
        # Use provided color or default colors
        if color:
            self.color = color
        else:
            self.color = CYAN if is_intake else PINK
        self.angle = angle  # Angle in degrees for valve orientation
        self.z_index = z_index
        self.transparency = transparency
        
    def update(self, lift_percentage):
        self.lift_percentage = lift_percentage
        self.current_lift = self.max_lift * lift_percentage
        
    def draw(self, screen):
        if self.angle == 0:
            # Vertical valve (moves up/down)
            # Draw stem
            pygame.draw.rect(screen, self.color, 
                            (self.x - 3, self.y - self.stem_length + self.current_lift, 
                             6, self.stem_length))
            
            # Draw valve head
            valve_head_y = self.y + self.current_lift
            pygame.draw.circle(screen, self.color, (int(self.x), int(valve_head_y)), self.radius)
            
            # Add small center dot
            pygame.draw.circle(screen, BLACK, (int(self.x), int(valve_head_y)), 2)
        elif self.angle == 90:
            # Left-facing horizontal valve (moves left/right)
            # Draw stem horizontally
            stem_x = self.x - self.current_lift
            pygame.draw.rect(screen, self.color, 
                            (stem_x - self.stem_length, self.y - 3, 
                             self.stem_length, 6))
            
            # Draw valve head
            pygame.draw.circle(screen, self.color, (int(stem_x), int(self.y)), self.radius)
            
            # Add small center dot
            pygame.draw.circle(screen, BLACK, (int(stem_x), int(self.y)), 2)
        elif self.angle == -90:
            # Right-facing horizontal valve (moves left/right)
            # Draw stem horizontally
            stem_x = self.x + self.current_lift
            pygame.draw.rect(screen, self.color, 
                            (self.x, self.y - 3, 
                             self.stem_length, 6))
            
            # Draw valve head
            pygame.draw.circle(screen, self.color, (int(stem_x), int(self.y)), self.radius)
            
            # Add small center dot
            pygame.draw.circle(screen, BLACK, (int(stem_x), int(self.y)), 2)


class Button:
    def __init__(self, x, y, width, height, text, action):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.action = action
        self.color = GRAY
        self.hover_color = LIGHT_GRAY
        self.text_color = BLACK
        self.hover = False
        self.font = pygame.font.SysFont('Arial', 20)
        
    def update(self, mouse_pos):
        self.hover = self.rect.collidepoint(mouse_pos)
        
    def draw(self, screen):
        pygame.draw.rect(screen, self.hover_color if self.hover else self.color, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)
        
        text_surf = self.font.render(self.text, True, self.text_color)
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)
        
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.hover:
            self.action()
            return True
        return False


class Slider:
    def __init__(self, x, y, width, height, min_val, max_val, initial_val, action, label=""):
        self.rect = pygame.Rect(x, y, width, height)
        self.handle_radius = height * 1.5
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial_val
        self.action = action
        self.dragging = False
        self.color = GRAY
        self.handle_color = DARK_GRAY
        self.active_color = BLUE
        self.label = label
        
        # Calculate handle position
        self.handle_x = self.rect.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.rect.width
        self.handle_y = self.rect.y + self.rect.height // 2
        
    def update(self, mouse_pos, mouse_down):
        if mouse_down and self.is_handle_hovered(mouse_pos):
            self.dragging = True
            
        if not mouse_down:
            self.dragging = False
            
        if self.dragging:
            self.handle_x = max(self.rect.x, min(mouse_pos[0], self.rect.x + self.rect.width))
            # Calculate new value
            self.value = self.min_val + (self.handle_x - self.rect.x) / self.rect.width * (self.max_val - self.min_val)
            self.action(self.value)
            
    def is_handle_hovered(self, mouse_pos):
        handle_rect = pygame.Rect(self.handle_x - self.handle_radius, 
                                  self.handle_y - self.handle_radius,
                                  self.handle_radius * 2, self.handle_radius * 2)
        return handle_rect.collidepoint(mouse_pos)
        
    def draw(self, screen):
        # Draw label
        if self.label:
            font = pygame.font.SysFont('Arial', 20)
            label_text = font.render(self.label, True, WHITE)
            screen.blit(label_text, (self.rect.x - 100, self.rect.y - 5))
            
        # Draw slider track
        pygame.draw.rect(screen, self.color, self.rect)
        
        # Draw active portion
        active_width = self.handle_x - self.rect.x
        pygame.draw.rect(screen, self.active_color, 
                         (self.rect.x, self.rect.y, active_width, self.rect.height))
        
        # Draw handle
        pygame.draw.circle(screen, self.handle_color, (int(self.handle_x), self.handle_y), self.handle_radius)
        
        # Draw value text
        font = pygame.font.SysFont('Arial', 16)
        value_text = font.render(f'{self.value:.0f}', True, WHITE)
        text_rect = value_text.get_rect(center=(self.handle_x, self.handle_y - self.handle_radius - 10))
        screen.blit(value_text, text_rect)


def main():
    # Set up the display
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Multi-Cylinder Engine Simulator")
    clock = pygame.time.Clock()
    
    # Create engine
    engine = Engine()
    
    # Create UI elements
    def toggle_engine():
        engine.running = not engine.running
        
    def set_rpm(value):
        engine.rpm = value
        
    def set_rod_length(value):
        engine.set_rod_length(value)
        
    def set_bore(value):
        engine.set_bore_size(value)
        
    def toggle_engine_type():
        engine.toggle_engine_type()
        
    def add_cylinder():
        if engine.engine_type == "Multi":
            engine.add_cylinder()
    
    start_button = Button(WIDTH - 120, 20, 100, 40, "Start/Stop", toggle_engine)
    engine_type_button = Button(WIDTH - 120, 70, 100, 40, "Toggle Type", toggle_engine_type)
    add_cylinder_button = Button(WIDTH - 120, 120, 100, 40, "Add Cylinder", add_cylinder)
    
    rpm_slider = Slider(WIDTH - 200, 180, 180, 10, 30, engine.max_rpm, engine.rpm, set_rpm, "Throttle:")
    rod_slider = Slider(WIDTH - 200, 230, 180, 10, engine.min_rod_length, engine.max_rod_length, 
                        engine.rod_length, set_rod_length, "Rod Length:")
    bore_slider = Slider(WIDTH - 200, 280, 180, 10, engine.min_bore, engine.max_bore, 
                         engine.cylinder_width, set_bore, "Bore Size:")
    
    # Main game loop
    running = True
    mouse_down = False
    
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    mouse_down = True
                    start_button.handle_event(event)
                    engine_type_button.handle_event(event)
                    add_cylinder_button.handle_event(event)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    mouse_down = False
        
        # Get mouse position
        mouse_pos = pygame.mouse.get_pos()
        
        # Update UI elements
        start_button.update(mouse_pos)
        engine_type_button.update(mouse_pos)
        add_cylinder_button.update(mouse_pos)
        rpm_slider.update(mouse_pos, mouse_down)
        rod_slider.update(mouse_pos, mouse_down)
        bore_slider.update(mouse_pos, mouse_down)
        
        # Update engine
        dt = clock.tick(FPS) / 1000.0  # Time since last frame in seconds
        engine.update(dt)
        
        # Draw everything
        screen.fill(BLACK)
        
        # Draw engine
        engine.draw(screen)
        
        # Draw UI elements
        start_button.draw(screen)
        engine_type_button.draw(screen)
        add_cylinder_button.draw(screen)
        rpm_slider.draw(screen)
        rod_slider.draw(screen)
        bore_slider.draw(screen)
        
        # Update the display
        pygame.display.flip()
    
    # Clean up
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()