import pygame
import numpy as np
import math
import pigpio

# Thruster pins
thruster_1 = 17  # Center thruster 1
thruster_2 = 18  # Center Thruster 2
thruster_3 =22  # Left Thruster
thruster_4 = 23  # Right Thruster
thruster_pins = [thruster_1, thruster_2, thruster_3, thruster_4]

# Import components and updated config
from config import (
    SCREEN_WIDTH, SCREEN_HEIGHT, BLACK, FONT_SIZE,
    PWM_NEUTRAL, PWM_RANGE,
    MAX_AXIAL_FORCE, MAX_YAW_TORQUE  # <-- Correct scaling constants
)
from input_handler import JoystickController
from rov_kinematics import compute_thruster_forces
# Import all drawing functions, including the new one
from drawing_utils import draw_rov, draw_thruster_vectors, draw_hud, draw_resultant_vector




def map_force_to_pwm(normalized_force):
    """Converts a normalized force [-1.0, 1.0] to a PWM signal [1200, 1800]."""
    # Formula: PWM = 1500 + (Force * 300)
    pwm_value = PWM_NEUTRAL + (normalized_force * PWM_RANGE)
    
    # Clip the value to the valid range [1200, 1800]
    pwm_value = max(PWM_NEUTRAL - PWM_RANGE, min(PWM_NEUTRAL + PWM_RANGE, pwm_value))
    
    return int(round(pwm_value))

def main():
    pygame.init()
    
    # 1. Initialize Input
    try:
        controller = JoystickController(deadzone=0.1)
    except RuntimeError as e:
        print(e)
        pygame.quit()
        return
    
    # Arming the Thrusters
    pi = pigpio.pi()
    for pin in thruster_pins:
        pi.set_servo_pulsewidth(pin, 1500)  

    # 2. Initialize Simulation Window
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ROV Thruster Allocation Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, FONT_SIZE)
    rov_center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)

    running = True
    while running:
        # --- A. Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # --- B. Input Phase (Controller) ---
        # Get raw desired motion vector from controller (Surge, Sway, Yaw)
        raw_inputs = controller.get_input_vector()
        raw_surge, raw_sway, raw_yaw = raw_inputs

        # --- C. Allocation Phase (Core Logic) ---
        
        # Scale the raw joystick input [-1, 1] to the vehicle's
        # maximum physical force/torque capabilities.
        # This is the corrected logic.
        desired_surge = raw_surge * MAX_AXIAL_FORCE
        desired_sway = raw_sway * MAX_AXIAL_FORCE
        desired_yaw = (raw_yaw) * MAX_YAW_TORQUE

        # The allocation function computes normalized thruster forces [-1, 1]
        # required to achieve these desired physical forces.
        # The saturation logic is handled *inside* this function.
        thruster_forces = compute_thruster_forces(desired_surge, desired_sway, desired_yaw)
        
        # --- D. Conversion to PWM (Real-World Output) ---
        # Convert the normalized forces [-1.0, 1.0] to PWM signals [1200, 1800]
        thruster_pwms = [map_force_to_pwm(f) for f in thruster_forces]

        # Console output for the actual PWM values
        # This will now correctly show 1800/1200 for full axial movement
        print(f"T1:{thruster_pwms[0]:>5d} | T2:{thruster_pwms[1]:>5d} | "
              f"T3:{thruster_pwms[2]:>5d} | T4:{thruster_pwms[3]:>5d}", end='\r')

        pi.set_servo_pulsewidth(thruster_pins[0], thruster_pwms[0])
        pi.set_servo_pulsewidth(thruster_pins[1], thruster_pwms[1])
        pi.set_servo_pulsewidth(thruster_pins[2], thruster_pwms[2])
        pi.set_servo_pulsewidth(thruster_pins[3], thruster_pwms[3])

        # --- E. Visualization/Output Phase ---
        screen.fill(BLACK)
        
        draw_rov(screen, rov_center)
        # Use the final normalized 'thruster_forces' for drawing the vectors
        draw_thruster_vectors(screen, font, rov_center, thruster_forces)
        
        # --- ADDED THIS LINE ---
        # Draw the resultant force vector from the center
        draw_resultant_vector(screen, rov_center, thruster_forces)
        
        # Use the 'raw_inputs' for the HUD
        draw_hud(screen, font, raw_inputs)
        
        pygame.display.flip()
        clock.tick(5) # <-- Set to 60 FPS for smooth visualization

    pygame.quit()
    print("\nSimulation exited.")

if __name__ == "__main__":
    main()
