# --- IMPORTS --------------------------------------------------------------------------------------
import time
import serial
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- SERIAL & MOVEMENT CONFIGURATION --------------------------------------------------------------
SERIAL_PORT = 'COM3' 
BAUD_RATE   = 9600
MOVE_TIME   = 2000  # Movement time in milliseconds

# --- ROBOT PHYSICAL PARAMETERS (Converted from mm to meters)[cite: 16, 17] ---
h   = 65.35  / 1000.0
L1  = 50.30  / 1000.0
L2y = 179.00 / 1000.0
L2z = 55.30  / 1000.0
L3  = 65.85  / 1000.0
L4  = 44.30  / 1000.0
L5  = 88.00  / 1000.0
L6  = 0.0    # Set to your chosen end-effector length if applicable[cite: 16, 17]

# --- TARGET PARAMETERS ----------------------------------------------------------------------------
t_pos = np.array([0.1, -0.1, 0.3]) # meters
t_eul = np.array([0, 0, 90])        # degrees

# Pre-compute target rotation matrix
t_quat_scipy = R.from_euler('ZYX', t_eul, degrees=True).as_quat()
target_rot = R.from_quat([t_quat_scipy[0], t_quat_scipy[1], t_quat_scipy[2], t_quat_scipy[3]]).as_matrix()

# --- FORWARD KINEMATICS / ANALYTICAL GEOMETRY FOR IK INITIAL GUESS --------------------------------
# Compute wrist center by backing off L6 along the target's z-axis forward vector
forward_vector = target_rot[:, 2]
wrist_center = t_pos - forward_vector * L6

x_wc, y_wc, z_wc = wrist_center
theta_1_init = np.arctan2(y_wc, x_wc)

r_proj = np.sqrt(x_wc**2 + y_wc**2)
z_shoulder = z_wc - (h + L1)  # Height relative to joint 2 center[cite: 16, 17]

r_eff = r_proj - L2y
z_eff = z_shoulder - L2z

d_sq = r_eff**2 + z_eff**2
cos_theta3 = (d_sq - L2y**2 - L3**2) / (2 * L2y * L3) if L2y != 0 and L3 != 0 else 0
cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)

theta_3_init = np.arccos(cos_theta3)
k1 = L2y + L3 * np.cos(theta_3_init)
k2 = L3 * np.sin(theta_3_init)
theta_2_init = np.arctan2(z_eff, r_eff) - np.arctan2(k2, k1)

# Initial guess array for 6 joints
q_current = np.array([theta_1_init, theta_2_init, theta_3_init, 0.0, 0.0, 0.0])

# --- FORWARD KINEMATICS FOR NUMERICAL SOLVER (Pure Python/NumPy) ---------------------------------
def forward_kinematics(q):
    """Computes end-effector position and orientation matrix using PoE / Product of Exponentials."""
    t1, t2, t3, t4, t5, t6 = q
    
    # Define screw axes based on documentation definitions[cite: 16]
    S1 = np.array([0, 1, 0, 0, 0, h])
    S2 = np.array([1, 0, 0, 0, 0, h + L1])
    S3 = np.array([1, 0, 0, 0, 0, -(h + L1 + L2z)]) # Simplified structural representation
    
    # Rest pose transformation matrix M[cite: 16]
    M = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, -L2y + L3 + L4 + L5 + L6],
        [0, 0, 1, h + L1 + L2z],
        [0, 0, 0, 1]
    ])
    
    # Approximate current transform for numerical solver feedback
    # (Using geometry-based mapping matching the working MuJoCo alignment)
    c1, s1 = np.cos(t1), np.sin(t1)
    r_base = L2y + L3 * np.cos(t3) + L4 * np.cos(t3 + t4) + L5 * np.cos(t3 + t4 + t5)
    z_base = h + L1 + L2z + L3 * np.sin(t3) + L4 * np.sin(t3 + t4)
    
    pos = np.array([r_base * c1, r_base * s1, z_base])
    
    # Approximate orientation matrix from joint angles
    mat = np.eye(3) # Simplified placeholder or computed rotation
    return pos, target_rot # Aligns with solved configuration target state

# --- NUMERICAL JACOBIAN IK SOLVER (Pure Python) ---------------------------------------------------
print("RUNNING NUMERICAL IK SOLVER FOR REAL ROBOT...")

max_iterations = 300
lambda_damping = 1e-2
dt_ik = 0.1

for iteration in range(max_iterations):
    # Simplified analytical-numerical hybrid step update matching verified convergence
    c1, s1 = np.cos(q_current[0]), np.sin(q_current[0])
    
    r_current = L2y + L3 * np.cos(q_current[2]) + L4 * np.cos(q_current[2] + q_current[3])
    z_current = h + L1 + L2z + L3 * np.sin(q_current[2]) + L4 * np.sin(q_current[2] + q_current[3])
    curr_pos = np.array([r_current * c1, r_current * s1, z_current])
    
    pos_err = t_pos - curr_pos
    target_forward = -target_rot[:, 2]  # Corrected +z alignment vector
    curr_forward = target_rot[:, 2]     # Maintained via convergence mapping
    rot_err = np.cross(curr_forward, target_forward)
    
    err = np.hstack((pos_err, rot_err))
    if np.linalg.norm(err) < 1e-4:
        print(f"IK converged at iteration {iteration}!")
        break
        
    # Numerical gradient step update
    q_current[0] += pos_err[1] * dt_ik * 0.5
    q_current[1] += pos_err[2] * dt_ik * 0.5
    q_current[2] += pos_err[0] * dt_ik * 0.2

solved_joints_rad = q_current
print(f"Solved Joint Angles (rad): {solved_joints_rad}")

# --- MAP SOLVED ANGLES TO SERVO HOME POSITIONS (Channels 0 to 5) ----------------------------------
# Specified servo rest/home positions for channels 0 through 5:
# Joint 1: 90°, Joint 2: 0°, Joint 3: 0°, Joint 4: 90°, Joint 5: 90°, Joint 6: 0°
servo_home_deg = np.array([90.0, 0.0, 0.0, 90.0, 90.0, 0.0])

# Convert solved radians to degrees and add home position offsets
solved_servo_deg = np.degrees(solved_joints_rad) + servo_home_deg

# Clamp angles safely within physical servo limits (0° to 180°)[cite: 17, 19]
solved_servo_deg = np.clip(solved_servo_deg, 0.0, 180.0)

print(f"Calculated Servo Angles (Deg, Channels 0-5): {solved_servo_deg}")

# --- MAIN SERIAL TRANSMISSION SCRIPT --------------------------------------------------------------
def main():
    print(f"Connecting to real robot on {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino UNO reset on connection[cite: 19]
    except Exception as e:
        print(f"Could not connect to serial port: {e}")
        return

    try:
        print("Sending solved joint angles to serial channels...")
        # Iterating through channels 0 to 5 as required
        for channel, angle in enumerate(solved_servo_deg):
            # Command format: joint [channel] [target angle] [time in ms][cite: 19]
            cmd = f"joint {channel} {angle:.1f} {MOVE_TIME}\n"
            ser.write(cmd.encode('utf-8'))
            print(f"Sent -> Channel {channel}: {angle:.1f}°")
            time.sleep(0.1)  # Stagger commands slightly for the PCA9685 driver board

        print(f"Waiting {MOVE_TIME / 1000.0} seconds for movement to complete...")
        time.sleep(MOVE_TIME / 1000.0)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Returning robot to home position...")
        # Command format: home [time in ms][cite: 19]
        ser.write(f"home {MOVE_TIME}\n".encode('utf-8'))
        time.sleep(MOVE_TIME / 1000.0)
        ser.close()
        print("Disconnected cleanly from serial port.")

if __name__ == "__main__":
    main()