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

class Engine:
    def __init__(self):
        # Engine dimensions and positions
        self.center_x = WIDTH // 2
        self.center_y = HEIGHT // 2 + 100
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
        
        # Create engine components
        self.crankshaft = Crankshaft(self.center_x, self.center_y, self.crank_radius)
        self.connecting_rod = ConnectingRod(self.crank_radius, self.rod_length)
        self.piston = Piston(self.piston_width, self.piston_height)
        
        # Position cylinder further away from crankshaft and 50 pixels higher
        self.cylinder = Cylinder(self.center_x, self.center_y - self.rod_length - self.crank_radius - self.cylinder_offset - 50, 
                                self.cylinder_width, self.cylinder_height)
        
        # Create valves
        valve_y = self.center_y - self.rod_length - self.crank_radius - 100 - self.cylinder_offset
        self.intake_valve = Valve(self.center_x - 30, valve_y, 
                                 self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=True)
        self.exhaust_valve = Valve(self.center_x + 30, valve_y, 
                                  self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=False)
        
        # Display values for UI elements
        self.rod_length_display = self.rod_length
        self.bore_size_display = self.cylinder_width
        
        # Engine state
        self.angle = 0
        self.running = False
        self.rpm = 60
        self.max_rpm = 300
        
        # Track the 4-stroke cycle (2 revolutions of crankshaft)
        self.cycle_angle = 0
    
    def set_rod_length(self, length):
        """Update connecting rod length and adjust related components"""
        self.rod_length = length
        # Need to update cylinder position when rod length changes
        self.update_component_positions()
    
    def set_bore_size(self, bore):
        """Update piston and cylinder bore (width)"""
        self.cylinder_width = bore
        self.piston_width = bore - 20  # Piston slightly smaller than bore
        # Update components that depend on these values
        self.update_component_positions()
    
    def update_component_positions(self):
        """Update the positions of components after parameter changes"""
        # Update cylinder position based on current rod length
        self.cylinder = Cylinder(self.center_x, self.center_y - self.rod_length - self.crank_radius - self.cylinder_offset - 50, 
                                self.cylinder_width, self.cylinder_height)
        
        # Update piston size
        self.piston = Piston(self.piston_width, self.piston_height)
        
        # Update valve positions
        valve_y = self.center_y - self.rod_length - self.crank_radius - 100 - self.cylinder_offset
        self.intake_valve = Valve(self.center_x - 30, valve_y, 
                                 self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=True)
        self.exhaust_valve = Valve(self.center_x + 30, valve_y, 
                                  self.valve_radius, self.valve_stem_length, self.valve_lift, is_intake=False)
    
    def update(self, dt):
        if self.running:
            # Update angle based on rpm
            dtheta = 2 * math.pi * self.rpm / 60 * dt
            self.angle = (self.angle + dtheta) % (2 * math.pi)
            
            # Update the cycle angle (0-720 degrees for 4-stroke cycle)
            self.cycle_angle = (self.cycle_angle + dtheta * 180 / math.pi) % 720
            
            # Calculate crankshaft pin position
            # When angle is 0, crankpin points up (TDC)
            # When angle is π, crankpin points down (BDC)
            crank_x = self.center_x + self.crank_radius * math.sin(self.angle)
            crank_y = self.center_y - self.crank_radius * math.cos(self.angle)
            
            # Update crankshaft
            self.crankshaft.update(self.angle)
            
            # Calculate piston position using correct kinematics for fixed rod length
            # Using the Pythagorean theorem to calculate the vertical position
            # piston_y is at TDC when crankpin is at top (angle = 0)
            
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
            
            # Update valves based on the 4-stroke cycle position
            # In a 4-stroke engine:
            # 0-180 degrees: Intake stroke (intake valve open)
            # 180-360 degrees: Compression stroke (both valves closed)
            # 360-540 degrees: Power stroke (both valves closed)
            # 540-720 degrees: Exhaust stroke (exhaust valve open)
            
            # Intake valve timing with smoother animation
            if 0 <= self.cycle_angle < 180:
                # Open during intake stroke - sine wave for smooth movement
                intake_lift = math.sin(self.cycle_angle * math.pi / 180)
                self.intake_valve.update(lift_percentage=intake_lift)
            else:
                # Closed for the rest of the cycle
                self.intake_valve.update(lift_percentage=0)
            
            # Exhaust valve timing with smoother animation
            if 540 <= self.cycle_angle < 720:
                # Open during exhaust stroke - sine wave for smooth movement
                exhaust_lift = math.sin((self.cycle_angle - 540) * math.pi / 180)
                self.exhaust_valve.update(lift_percentage=exhaust_lift)
            else:
                # Closed for the rest of the cycle
                self.exhaust_valve.update(lift_percentage=0)
    
    def draw(self, screen):
        # Draw background
        pygame.draw.rect(screen, BLACK, (0, 0, WIDTH, HEIGHT))
        
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
        
        # Uncomment to debug valve timing
        # cycle_text = font.render(f'Cycle: {self.cycle_angle:.0f}°', True, WHITE)
        # screen.blit(cycle_text, (10, 130))


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
    def __init__(self, crank_radius, length):
        self.length = length
        self.crank_end_x = 0
        self.crank_end_y = 0
        self.piston_end_x = 0
        self.piston_end_y = 0
        
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
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.x = 0
        self.y = 0
        
    def update(self, x, y):
        self.x = x - self.width // 2
        self.y = y - self.height // 2
        
    def draw(self, screen):
        # Draw piston body (similar to the reference image)
        pygame.draw.rect(screen, WHITE, (self.x, self.y, self.width, self.height))
        
        # Draw piston pin (wrist pin)
        pygame.draw.circle(screen, BLACK, 
                           (int(self.x + self.width // 2), int(self.y + self.height // 2)), 8)
        
        # Draw piston crown detail (similar to the reference image)
        pygame.draw.rect(screen, BLACK, 
                         (self.x + self.width//4, self.y + 5, self.width//2, 5))


class Cylinder:
    def __init__(self, x, y, width, height):
        self.x = x - width // 2
        self.y = y
        self.width = width
        self.height = height
        self.wall_thickness = 12
        
    def draw(self, screen):
        # Draw cylinder walls similar to reference (pink)
        pygame.draw.line(screen, PINK, (self.x, self.y), (self.x, self.y + self.height), self.wall_thickness)
        pygame.draw.line(screen, PINK, (self.x + self.width, self.y), 
                         (self.x + self.width, self.y + self.height), self.wall_thickness)
        
        # Draw open cylinder extension pieces (similar to reference)
        pygame.draw.rect(screen, PINK, 
                        (self.x - 50, self.y, 50, self.wall_thickness * 2))
        pygame.draw.rect(screen, PINK, 
                        (self.x + self.width, self.y, 50, self.wall_thickness * 2))
        
        # Top part removed as requested - cylinder is now open at the top


class Valve:
    def __init__(self, x, y, radius, stem_length, max_lift, is_intake=True):
        self.x = x
        self.y = y
        self.radius = radius
        self.stem_length = stem_length
        self.max_lift = max_lift
        self.is_intake = is_intake
        self.lift_percentage = 0
        self.current_lift = 0
        # Keep using PINK for exhaust valve as in the reference image
        self.color = CYAN if is_intake else PINK
        
    def update(self, lift_percentage):
        self.lift_percentage = lift_percentage
        self.current_lift = self.max_lift * lift_percentage
        
    def draw(self, screen):
        # Draw valve stem with the same color as valve head
        stem_color = self.color
        pygame.draw.rect(screen, stem_color, 
                         (self.x - 3, self.y - self.stem_length + self.current_lift, 
                          6, self.stem_length))
        
        # Draw valve head as a circle for better visibility
        valve_head_y = self.y + self.current_lift
        pygame.draw.circle(screen, self.color, 
                           (int(self.x), int(valve_head_y)), self.radius)


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
    pygame.display.set_caption("1-Cylinder Engine Simulator")
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
    
    start_button = Button(WIDTH - 120, 20, 100, 40, "Start/Stop", toggle_engine)
    rpm_slider = Slider(WIDTH - 200, 80, 180, 10, 30, engine.max_rpm, engine.rpm, set_rpm, "Throttle:")
    rod_slider = Slider(WIDTH - 200, 130, 180, 10, engine.min_rod_length, engine.max_rod_length, 
                        engine.rod_length, set_rod_length, "Rod Length:")
    bore_slider = Slider(WIDTH - 200, 180, 180, 10, engine.min_bore, engine.max_bore, 
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
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    mouse_down = False
        
        # Get mouse position
        mouse_pos = pygame.mouse.get_pos()
        
        # Update UI elements
        start_button.update(mouse_pos)
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