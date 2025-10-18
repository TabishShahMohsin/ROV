import pygame
import numpy as np
import math

# --- Constants ---
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)
BLUE = (0, 170, 255)
RED = (255, 50, 50)
GREEN = (0, 200, 100)
VECTOR_SCALE = 100

# --- ROV Configuration ---
ROV_WIDTH_MM = 195.311
ROV_LENGTH_MM = 262.629
SIM_ROV_WIDTH = 150
SIM_ROV_LENGTH = int(SIM_ROV_WIDTH * (ROV_LENGTH_MM / ROV_WIDTH_MM))


# --- Thruster Calculation Logic ---
def compute_thruster_forces(surge_input, sway_input, yaw_input):
    """
    Computes thrust for 4 diagonal thrusters in a symmetrical vectored 'X' configuration.
    """
    positions = np.array([
        [-ROV_WIDTH_MM/2,  ROV_LENGTH_MM/2],  # T1 (Front-Left)
        [ ROV_WIDTH_MM/2,  ROV_LENGTH_MM/2],  # T2 (Front-Right)
        [-ROV_WIDTH_MM/2, -ROV_LENGTH_MM/2],  # T3 (Rear-Left)
        [ ROV_WIDTH_MM/2, -ROV_LENGTH_MM/2],  # T4 (Rear-Right)
    ])

    # CORRECTED: Physically sound angles for a vectored X-frame
    angles = np.deg2rad([135, 45, -135, -45])

    B = np.zeros((3, 4))
    for i, ((x, y), theta) in enumerate(zip(positions, angles)):
        B[0, i] = np.cos(theta)                  # Contribution to Fx (Sway)
        B[1, i] = np.sin(theta)                  # Contribution to Fy (Surge)
        B[2, i] = x * np.sin(theta) - y * np.cos(theta)  # Contribution to Torque (Yaw)

    v = np.array([sway_input, surge_input, yaw_input])
    thruster_forces = np.linalg.pinv(B) @ v

    max_force = np.max(np.abs(thruster_forces))
    if max_force > 1:
        thruster_forces /= max_force

    return thruster_forces

# --- Drawing Functions ---
def draw_rov(screen, center_pos):
    rov_rect = pygame.Rect(0, 0, SIM_ROV_WIDTH, SIM_ROV_LENGTH)
    rov_rect.center = center_pos
    pygame.draw.rect(screen, GRAY, rov_rect, border_radius=10)
    pygame.draw.rect(screen, WHITE, rov_rect, width=3, border_radius=10)
    
    front_indicator_start = (center_pos[0], center_pos[1] - SIM_ROV_LENGTH // 4)
    front_indicator_end = (center_pos[0], center_pos[1] - SIM_ROV_LENGTH // 2)
    pygame.draw.line(screen, WHITE, front_indicator_start, front_indicator_end, 5)

def draw_thruster_vectors(screen, font, center_pos, forces):
    thruster_positions = [
        (center_pos[0] - SIM_ROV_WIDTH/2, center_pos[1] - SIM_ROV_LENGTH/2),
        (center_pos[0] + SIM_ROV_WIDTH/2, center_pos[1] - SIM_ROV_LENGTH/2),
        (center_pos[0] - SIM_ROV_WIDTH/2, center_pos[1] + SIM_ROV_LENGTH/2),
        (center_pos[0] + SIM_ROV_WIDTH/2, center_pos[1] + SIM_ROV_LENGTH/2)
    ]
    # CORRECTED: Drawing angles must match the physics angles
    angles_deg = [135, 45, -135, -45]
    
    for i, (pos, angle_deg, force) in enumerate(zip(thruster_positions, angles_deg, forces)):
        angle_rad = math.radians(angle_deg)
        color = RED if force > 0 else BLUE
        
        start_x, start_y = pos
        end_x = start_x + (VECTOR_SCALE * force * math.cos(angle_rad))
        end_y = start_y - (VECTOR_SCALE * force * math.sin(angle_rad))
        
        if abs(force) > 0.05:
            pygame.draw.line(screen, color, (start_x, start_y), (end_x, end_y), 5)
            arrow_angle = math.atan2(start_y - end_y, end_x - start_x)
            p1 = (end_x - 10 * math.cos(arrow_angle - math.pi/6), end_y + 10 * math.sin(arrow_angle - math.pi/6))
            p2 = (end_x - 10 * math.cos(arrow_angle + math.pi/6), end_y + 10 * math.sin(arrow_angle + math.pi/6))
            pygame.draw.polygon(screen, color, [(end_x, end_y), p1, p2])

        text_surface = font.render(f"T{i+1}: {force:.2f}", True, WHITE)
        text_rect = text_surface.get_rect(center=(pos[0] + 45 * np.sign(pos[0]-center_pos[0]), pos[1] + 45 * np.sign(pos[1]-center_pos[1])))
        screen.blit(text_surface, text_rect)

def draw_hud(screen, font, inputs):
    surge, sway, yaw = inputs
    lines = [ f"Surge (Forward): {surge:.2f}", f"Sway (Right):    {sway:.2f}", f"Yaw (Turn):      {yaw:.2f}" ]
    for i, line in enumerate(lines):
        text_surface = font.render(line, True, GREEN)
        screen.blit(text_surface, (10, 10 + i * 30))

# --- Main Simulation Function ---
def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Error: No joysticks detected.")
        pygame.quit()
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick Detected: {joystick.get_name()}")

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ROV Thruster Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 30)
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        surge_input = -joystick.get_axis(1) 
        if abs(surge_input) < 0.1: surge_input = 0.0
        sway_input = joystick.get_axis(0)
        if abs(sway_input) < 0.1: sway_input = 0.0
        yaw_input = joystick.get_axis(2)
        if abs(yaw_input) < 0.1: yaw_input = 0.0

        thruster_forces = compute_thruster_forces(surge_input, sway_input, yaw_input)
        
        print(f"T1:{thruster_forces[0]:>6.2f} | T2:{thruster_forces[1]:>6.2f} | "
              f"T3:{thruster_forces[2]:>6.2f} | T4:{thruster_forces[3]:>6.2f}", end='\r')

        screen.fill(BLACK)
        rov_center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
        
        draw_rov(screen, rov_center)
        draw_thruster_vectors(screen, font, rov_center, thruster_forces)
        draw_hud(screen, font, (surge_input, sway_input, yaw_input))
        
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    print("\nSimulation exited.")

if __name__ == "__main__":
    main()