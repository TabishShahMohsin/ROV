import pygame
import numpy as np

class JoystickController:
    """Handles connection and reading of a PS4 DualShock controller."""
    def __init__(self, deadzone=0.1):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Error: No PS4 controller detected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Controller Connected: {self.joystick.get_name()}")
        self.deadzone = deadzone

    def get_input_vector(self):
        """Reads PS4 axes and returns (surge, sway, yaw) raw inputs."""
        # --- AXIS MAPPING ---
        # axis(1): Left Stick Y  (-1 = up)
        # axis(0): Left Stick X
        # axis(2): Right Stick X  (-1 = left, +1 = right)
        # axis(3), axis(4): L2 / R2 (can also be used for throttle)

        surge_input = -self.joystick.get_axis(1)  # Invert Y-axis for forward
        sway_input = self.joystick.get_axis(0)
        yaw_input = self.joystick.get_axis(2) - self.joystick.get_axis(5)  # Right stick X-axis controls yaw

        # Apply deadzone filtering
        if abs(surge_input) < self.deadzone: surge_input = 0.0
        if abs(sway_input) < self.deadzone: sway_input = 0.0
        if abs(yaw_input) < self.deadzone: yaw_input = 0.0

        return np.array([surge_input, sway_input, yaw_input])