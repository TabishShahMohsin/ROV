import numpy as np
from config import ROV_WIDTH_MM, ROV_LENGTH_MM # Import physical constants

def compute_thruster_forces(desired_surge, desired_sway, desired_yaw):
    """
    Computes thrust for 4 diagonal thrusters.
    
    Inputs are the SCALED desired physical forces/torques.
    Outputs are normalized thruster forces (-1.0 to 1.0).
    """
    positions = np.array([
        [-ROV_WIDTH_MM/2,  ROV_LENGTH_MM/2],  # T1 (Front-Left)
        [ ROV_WIDTH_MM/2,  ROV_LENGTH_MM/2],  # T2 (Front-Right)
        [-ROV_WIDTH_MM/2, -ROV_LENGTH_MM/2],  # T3 (Rear-Left)
        [ ROV_WIDTH_MM/2, -ROV_LENGTH_MM/2],  # T4 (Rear-Right)
    ])

    # "V" Configuration: T1(45), T2(135), T3(-45), T4(-135)
    angles = np.deg2rad([45, 135, -45, -135]) # Thruster angles
    
    # Thruster Allocation Matrix (B): [Sway(x), Surge(y), Yaw(t)] x [T1, T2, T3, T4]
    B = np.zeros((3, 4))
    for i, ((x, y), theta) in enumerate(zip(positions, angles)):
        B[0, i] = np.cos(theta)                  # Contribution to Fx (Sway)
        B[1, i] = np.sin(theta)                  # Contribution to Fy (Surge)
        B[2, i] = x * np.sin(theta) - y * np.cos(theta) # Contribution to Torque (Yaw)

    # Desired Force Vector (v contains the scaled demands)
    v = np.array([desired_sway, desired_surge, desired_yaw]) 
    
    # Calculate required thruster forces using the pseudo-inverse (B+)
    # This finds the most efficient (least-squares) force combination.
    thruster_forces = np.linalg.pinv(B) @ v

    # --- Saturation/Normalization ---
    # This step is critical: it finds the maximum required force and ensures 
    # it does not exceed 1.0, scaling ALL forces proportionally if needed.
    max_force = np.max(np.abs(thruster_forces))
    if max_force > 1.0:
        thruster_forces /= max_force

    return thruster_forces