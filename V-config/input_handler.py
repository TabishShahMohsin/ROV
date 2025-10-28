import pygame
import numpy as np
from config import CONTROLLER_TYPE, XBOX, PS


class XboxController:
    """Handles connection and reading of an Xbox controller."""
    def __init__(self, deadzone=0.1):
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Error: No Xbox controller detected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Xbox Controller Connected: {self.joystick.get_name()}")
        self.deadzone = deadzone

    def get_input_vector(self):
        """Reads axes and returns (surge, sway, yaw)."""
        surge_input = -self.joystick.get_axis(1)   # Left stick Y (invert)
        sway_input = self.joystick.get_axis(0)     # Left stick X
        yaw_input = (self.joystick.get_axis(5) - self.joystick.get_axis(2)) / 2  # Triggers

        # Apply deadzone
        surge_input = 0.0 if abs(surge_input) < self.deadzone else surge_input
        sway_input = 0.0 if abs(sway_input) < self.deadzone else sway_input
        yaw_input = 0.0 if abs(yaw_input) < self.deadzone else yaw_input

        return np.array([surge_input, sway_input, yaw_input])


class PSController:
    """Handles connection and reading of a PS4/PS5 controller."""
    def __init__(self, deadzone=0.1):
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("Error: No PS controller detected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"PS Controller Connected: {self.joystick.get_name()}")
        self.deadzone = deadzone

    def get_input_vector(self):
        """Reads axes and returns (surge, sway, yaw)."""
        surge_input = -self.joystick.get_axis(1)   # Left stick Y
        sway_input = self.joystick.get_axis(0)     # Left stick X
        yaw_input = self.joystick.get_axis(2) - self.joystick.get_axis(5)  # Right stick X diff

        # Apply deadzone
        surge_input = 0.0 if abs(surge_input) < self.deadzone else surge_input
        sway_input = 0.0 if abs(sway_input) < self.deadzone else sway_input
        yaw_input = 0.0 if abs(yaw_input) < self.deadzone else yaw_input

        return np.array([surge_input, sway_input, yaw_input])


controller_type = CONTROLLER_TYPE.upper()

if controller_type == XBOX:
    JoystickController = XboxController
elif controller_type == PS:
    JoystickController = PSController
else:
    raise ValueError(f"Unknown CONTROLLER_TYPE '{CONTROLLER_TYPE}'. Use 'XBOX' or 'PS'.")


