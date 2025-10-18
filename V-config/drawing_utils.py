import pygame
import math
from config import * # Import all constants

def draw_arrow(screen, color, start_pos, end_pos, width=5, arrow_size=10):
    """Draws a line with an arrowhead."""
    # Draw the main line
    pygame.draw.line(screen, color, start_pos, end_pos, width)
    
    # Calculate arrowhead lines
    try:
        angle = math.atan2(start_pos[1] - end_pos[1], start_pos[0] - end_pos[0])
    except ZeroDivisionError:
        return
        
    angle_left = angle + math.pi / 6
    angle_right = angle - math.pi / 6
    
    arrow_p1 = (end_pos[0] + arrow_size * math.cos(angle_left), 
                end_pos[1] + arrow_size * math.sin(angle_left))
    arrow_p2 = (end_pos[0] + arrow_size * math.cos(angle_right), 
                end_pos[1] + arrow_size * math.sin(angle_right))
    
    pygame.draw.line(screen, color, end_pos, arrow_p1, width)
    pygame.draw.line(screen, color, end_pos, arrow_p2, width)

def draw_rov(screen, center_pos):
    """Draws the ROV body."""
    cx, cy = center_pos
    rov_rect = pygame.Rect(0, 0, SIM_ROV_WIDTH, SIM_ROV_LENGTH)
    rov_rect.center = center_pos
    
    # Draw body
    pygame.draw.rect(screen, GRAY, rov_rect, border_radius=10)
    pygame.draw.rect(screen, WHITE, rov_rect, width=3, border_radius=10)
    
    # Draw front indicator
    front_indicator_start = (cx, cy - SIM_ROV_LENGTH // 4)
    front_indicator_end = (cx, cy - SIM_ROV_LENGTH // 2 + 10)
    draw_arrow(screen, WHITE, front_indicator_start, front_indicator_end, 5, 15)

    # Draw thruster mount points
    for (tx, ty) in THRUSTER_POSITIONS:
        pygame.draw.circle(screen, WHITE, (cx + tx, cy + ty), 8, 2)

def draw_thruster_vectors(screen, font, center_pos, forces):
    """Draws the force vectors and thruster labels."""
    cx, cy = center_pos

    for i, (pos_offset, angle_deg, force) in enumerate(zip(THRUSTER_POSITIONS, THRUSTER_ANGLES_DEG, forces)):
        angle_rad = math.radians(angle_deg)
        color = RED if force > 0 else BLUE # Red for positive, Blue for negative
        
        # Start of vector is at the thruster position
        start_x = cx + pos_offset[0]
        start_y = cy + pos_offset[1]
        
        # Vector points in the direction of thrust
        # Note: Pygame Y is inverted, so we subtract sin
        end_x = start_x + (VECTOR_SCALE * force * math.cos(angle_rad))
        end_y = start_y - (VECTOR_SCALE * force * math.sin(angle_rad))
        
        # Draw the vector
        if abs(force) > 0.05:
            draw_arrow(screen, color, (start_x, start_y), (end_x, end_y))
        
        # Draw force label
        text_surface = font.render(f"T{i+1}: {force:.2f}", True, WHITE)
        
        # Position label near the thruster
        text_rect = text_surface.get_rect()
        text_x = start_x + (30 * (1 if pos_offset[0] > 0 else -1.5))
        text_y = start_y + (30 * (1 if pos_offset[1] > 0 else -1.5))
        text_rect.center = (text_x, text_y)
        screen.blit(text_surface, text_rect)

def draw_hud(screen, font, inputs):
    """Draws the Head-Up Display for input values."""
    surge, sway, yaw = inputs
    
    # --- Text display ---
    lines = [
        f"Surge (Forward): {surge: .2f}",
        f"Sway (Right):   {sway: .2f}",
        f"Yaw (Turn):     {yaw: .2f}"
    ]
    for i, line in enumerate(lines):
        text_surface = font.render(line, True, GREEN)
        screen.blit(text_surface, (10, 10 + i * 30))
        
    # --- Visual Joystick HUD ---
    hud_center_x = SCREEN_WIDTH - 100
    hud_center_y = SCREEN_HEIGHT - 100
    hud_radius = 60
    
    # Draw joystick base
    pygame.draw.circle(screen, DARK_GRAY, (hud_center_x, hud_center_y), hud_radius)
    pygame.draw.circle(screen, GRAY, (hud_center_x, hud_center_y), hud_radius, 2)
    
    # Draw joystick position
    # Use -surge because screen Y-axis is inverted
    stick_x = hud_center_x + sway * (hud_radius - 10)
    stick_y = hud_center_y - surge * (hud_radius - 10)
    pygame.draw.circle(screen, WHITE, (stick_x, stick_y), 10)

    # --- Visual Yaw HUD ---
    yaw_bar_y = SCREEN_HEIGHT - 25
    bar_width = hud_radius * 2
    
    # Draw yaw bar base
    pygame.draw.rect(screen, DARK_GRAY, (hud_center_x - hud_radius, yaw_bar_y - 10, bar_width, 20))
    pygame.draw.rect(screen, GRAY, (hud_center_x - hud_radius, yaw_bar_y - 10, bar_width, 20), 2)
    
    # Draw yaw indicator
    indicator_width = 10
    indicator_x = hud_center_x + yaw * (hud_radius - indicator_width / 2)
    pygame.draw.rect(screen, GREEN, (indicator_x - indicator_width/2, yaw_bar_y - 10, indicator_width, 20))
    
    # Draw center line
    pygame.draw.line(screen, GRAY, (hud_center_x, yaw_bar_y - 10), (hud_center_x, yaw_bar_y + 10), 2)

# --- NEW FUNCTION ---
def draw_resultant_vector(screen, center_pos, forces):
    """
    Calculates and draws the total resultant force vector (Surge + Sway)
    originating from the center of the ROV.
    """
    total_fx = 0.0
    total_fy = 0.0
    
    # Sum the X and Y components of all thruster forces
    for force, angle_deg in zip(forces, THRUSTER_ANGLES_DEG):
        angle_rad = math.radians(angle_deg)
        total_fx += force * math.cos(angle_rad)
        total_fy += force * math.sin(angle_rad)

    # Define start and end points
    start_x, start_y = center_pos
    # Use RESULTANT_VECTOR_SCALE from config
    end_x = start_x + (total_fx * RESULTANT_VECTOR_SCALE)
    end_y = start_y - (total_fy * RESULTANT_VECTOR_SCALE) # Pygame Y is inverted
    
    # Draw the vector if it has significant magnitude
    if abs(total_fx) > 0.05 or abs(total_fy) > 0.05:
        # Use YELLOW from config
        draw_arrow(screen, YELLOW, (start_x, start_y), (end_x, end_y), width=7, arrow_size=15)