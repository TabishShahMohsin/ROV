import pygame
import numpy as np
import math
import pigpio

# Thruster pins
thruster_1 = 17  # Center thruster 1
thruster_2 = 18  # Center Thruster 2
thruster_3 = 22  # Left Thruster
thruster_4 = 23  # Right Thruster
thruster_pins = [thruster_1, thruster_2, thruster_3, thruster_4]

# Import components and updated config
from config import (
    SCREEN_WIDTH, SCREEN_HEIGHT, BLACK, FONT_SIZE,
    PWM_NEUTRAL, PWM_RANGE,
    MAX_AXIAL_FORCE, MAX_YAW_TORQUE
)
from input_handler import JoystickController
from rov_kinematics import compute_thruster_forces
from drawing_utils import draw_rov, draw_thruster_vectors, draw_hud, draw_resultant_vector


def map_force_to_pwm(normalized_force):
    """Converts a normalized force [-1.0, 1.0] to a PWM signal [1200, 1800]."""
    pwm_value = PWM_NEUTRAL + (normalized_force * PWM_RANGE)
    pwm_value = max(PWM_NEUTRAL - PWM_RANGE, min(PWM_NEUTRAL + PWM_RANGE, pwm_value))
    return int(round(pwm_value))

def set_all_thrusters(pi, pwm_values):
    """Send PWM values to all thrusters."""
    for pin, pwm in zip(thruster_pins, pwm_values):
        pi.set_servo_pulsewidth(pin, pwm)

def stop_all_thrusters(pi):
    """Set all thrusters to neutral (1500μs)."""
    for pin in thruster_pins:
        pi.set_servo_pulsewidth(pin, PWM_NEUTRAL)

def main():
    pygame.init()

    # Initialize input
    try:
        controller = JoystickController(deadzone=0.1)
    except RuntimeError as e:
        print(e)
        pygame.quit()
        return

    # Initialize pigpio and arm thrusters
    pi = pigpio.pi()
    stop_all_thrusters(pi)

    # Initialize simulation window
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("ROV Thruster Allocation Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, FONT_SIZE)
    rov_center = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)

    running = True
    while running:
        # Handle quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Read joystick input
        raw_inputs = controller.get_input_vector()
        raw_surge, raw_sway, raw_yaw = raw_inputs

        # Convert to desired forces
        desired_surge = raw_surge * MAX_AXIAL_FORCE
        desired_sway = raw_sway * MAX_AXIAL_FORCE
        desired_yaw = raw_yaw * MAX_YAW_TORQUE

        # Get thruster force distribution
        thruster_forces = compute_thruster_forces(desired_surge, desired_sway, desired_yaw)

        # Convert forces to PWM
        thruster_pwms = [map_force_to_pwm(f) for f in thruster_forces]

        # Apply PWM to thrusters
        set_all_thrusters(pi, thruster_pwms)

        # Debug output
        print(f"T1:{thruster_pwms[0]:>5d} | T2:{thruster_pwms[1]:>5d} | "
              f"T3:{thruster_pwms[2]:>5d} | T4:{thruster_pwms[3]:>5d}", end='\r')

        # Draw visuals
        screen.fill(BLACK)
        draw_rov(screen, rov_center)
        draw_thruster_vectors(screen, font, rov_center, thruster_forces)
        draw_resultant_vector(screen, rov_center, thruster_forces)
        draw_hud(screen, font, raw_inputs)
        pygame.display.flip()
        clock.tick(5)

    # On exit: stop thrusters safely
    stop_all_thrusters(pi)
    pygame.quit()
    print("\nSimulation exited.")

if __name__ == "__main__":
    main()
