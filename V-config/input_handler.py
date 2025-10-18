import pygame
import numpy as np

class JoystickController:
    """Handles connection and reading of a standard joystick/gamepad."""
    def __init__(self, deadzone=0.1):
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Error: No joysticks detected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Joystick Detected: {self.joystick.get_name()}")
        self.deadzone = deadzone

    def get_input_vector(self):
        """Reads axes and returns (surge, sway, yaw) raw inputs."""
        
        # --- AXIS MAPPING ---
        # This mapping is common (e.g., Xbox controller) but may need
        # to be changed based on your specific gamepad.
        #
        # axis(1) -> Left Stick Y (Inverted: -1 is up)
        # axis(0) -> Left Stick X
        # axis(2) -> Right Stick X
        
        surge_input = -self.joystick.get_axis(1) # Invert Y-axis
        sway_input = self.joystick.get_axis(0)
        yaw_input = (self.joystick.get_axis(5) - self.joystick.get_axis(2)) / 2

        # Apply deadzone
        if abs(surge_input) < self.deadzone: surge_input = 0.0
        if abs(sway_input) < self.deadzone: sway_input = 0.0
        if abs(yaw_input) < self.deadzone: yaw_input = 0.0

        # Returns raw inputs, typically in [-1.0, 1.0]
        return np.array([surge_input, sway_input, yaw_input])