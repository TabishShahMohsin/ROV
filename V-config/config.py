import pygame
import math

# --- Screen Constants ---
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (100, 100, 100)
DARK_GRAY = (50, 50, 50)
BLUE = (0, 170, 255)
RED = (255, 50, 50)
GREEN = (0, 200, 100)
YELLOW = (255, 255, 0) # <-- ADDED THIS
VECTOR_SCALE = 80
RESULTANT_VECTOR_SCALE = 50 # <-- ADDED THIS
FONT_SIZE = 30

# --- ROV Physical Configuration (for Allocation Logic) ---
ROV_WIDTH_MM = 262.629
ROV_LENGTH_MM = 195.311

# --- Simulation Drawing Configuration ---
SIM_ROV_WIDTH = 150
SIM_ROV_LENGTH = int(SIM_ROV_WIDTH * (ROV_LENGTH_MM / ROV_WIDTH_MM))
THRUSTER_POSITIONS = [
    (-SIM_ROV_WIDTH/2, -SIM_ROV_LENGTH/2), # T1 (Front-Left)
    ( SIM_ROV_WIDTH/2, -SIM_ROV_LENGTH/2), # T2 (Front-Right)
    (-SIM_ROV_WIDTH/2,  SIM_ROV_LENGTH/2), # T3 (Rear-Left)
    ( SIM_ROV_WIDTH/2,  SIM_ROV_LENGTH/2)  # T4 (Rear-Right)
]

# --- Thruster Angles ---
# "V" Configuration: T1(45), T2(135), T3(-45), T4(-135)
THRUSTER_ANGLES_DEG = [45, 135, -45, -135] 

# --- Thruster Control & Scaling ---

SIN_45 = math.sin(math.radians(45)) # approx 0.7071

# Maximum theoretical force/torque for "V" configuration
#
# Max axial (Surge/Sway) force = 4 * sin(45) * 1.0_thruster_force
# (All 4 thrusters contribute, same as "X" config)
MAX_AXIAL_FORCE = 4 * SIN_45  # approx 2.828

# Max yaw torque = 2 * sin(45) * (L + W) * 1.0_thruster_force
# (All 4 thrusters contribute)
MAX_YAW_TORQUE = 2 * SIN_45 * (ROV_LENGTH_MM + ROV_WIDTH_MM) # approx 647.6

# PWM Mapping Constants (T200)
PWM_NEUTRAL = 1500
PWM_RANGE = 70 # Max deviation from neutral
PWM_MIN = PWM_NEUTRAL - PWM_RANGE
PWM_MAX = PWM_NEUTRAL + PWM_RANGE