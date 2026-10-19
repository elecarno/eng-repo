# --- IMPORTS --------------------------------------------------------------------------------------
import time
import serial
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- SERIAL & MOVEMENT CONFIGURATION --------------------------------------------------------------
SERIAL_PORT = 'COM3' 
BAUD_RATE   = 9600
MOVE_TIME   = 2000  # Movement time in milliseconds

# --- VARIABLES (MUJoCo MODEL) ---------------------------------------------------------------------
model = mujoco.MjModel.from_xml_path("../morris-v2_mujoco/scene.xml")
data  = mujoco.MjData(model)

# disable contact physics
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# target frame
target_mocap_id = model.body('target_frame').mocapid[0] if 'target_frame' in [model.body(i).name for i in range(model.nbody)] else -1

# get actuator IDs and corresponding joint addresses
actuator_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
actuator_ids = [model.actuator(name).id for name in actuator_names]
joint_ids = [model.actuator_trnid[act_id, 0] for act_id in actuator_ids]
qpos_addrs = [model.jnt_qposadr[j_id] for j_id in joint_ids]

# Find end-effector body
ee_body_name = "m_v2_end_effector"
if ee_body_name not in [model.body(i).name for i in range(model.nbody)]:
    ee_body_name = model.body(model.nbody - 1).name
ee_body_id = model.body(ee_body_name).id

def disable_collision_on_body(model, body_name):
    try:
        body1_id = model.body(body_name).id
        for geom_id in range(model.ngeom):
            if model.geom_bodyid[geom_id] == body1_id:
                model.geom_contype[geom_id] = 0
                model.geom_conaffinity[geom_id] = 0
    except KeyError:
        pass

disable_collision_on_body(model, "m_v2_base")

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

# Pre-compute target quaternion and rotation matrix
t_quat_scipy = R.from_euler('ZYX', t_eul, degrees=True).as_quat()
t_quat = np.array([t_quat_scipy[3], t_quat_scipy[0], t_quat_scipy[1], t_quat_scipy[2]])
target_rot = R.from_quat([t_quat_scipy[0], t_quat_scipy[1], t_quat_scipy[2], t_quat_scipy[3]]).as_matrix()

# --- NUMERICAL JACOBIAN IK SOLVER ----------------------------------------------------------------
print("RUNNING NUMERICAL JACOBIAN IK SOLVER...")

# Set initial guess to zeros
for qpos_idx in qpos_addrs:
    data.qpos[qpos_idx] = 0.0
mujoco.mj_forward(model, data)

max_iterations = 300
lambda_damping = 1e-2
dt_ik = 0.1

for iteration in range(max_iterations):
    mujoco.mj_forward(model, data)
    
    curr_pos = data.xpos[ee_body_id]
    curr_mat = data.xmat[ee_body_id].reshape(3, 3)
    
    pos_err = t_pos - curr_pos
    
    # Align z-axis forward vector (+z to +z)
    curr_forward = curr_mat[:, 2]
    target_forward = -target_rot[:, 2]  
    rot_err = np.cross(curr_forward, target_forward)
    
    err = np.hstack((pos_err, rot_err))
    err_norm = np.linalg.norm(err)
    
    if err_norm < 1e-4:
        print(f"IK successfully converged at iteration {iteration}!")
        break
        
    jac_pos = np.zeros((3, model.nv))
    jac_rot = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jac_pos, jac_rot, ee_body_id)
    
    nv_indices = [model.jnt_dofadr[j_id] for j_id in joint_ids]
    J = np.vstack((jac_pos[:, nv_indices], jac_rot[:, nv_indices]))
    
    # Damped Least Squares
    J_pinv = J.T @ np.linalg.inv(J @ J.T + (lambda_damping**2) * np.eye(6))
    dq = J_pinv @ err
    
    for i, qpos_idx in enumerate(qpos_addrs):
        data.qpos[qpos_idx] += dq[i] * dt_ik

mujoco.mj_forward(model, data)
solved_joints_rad = np.array([data.qpos[qpos_idx] for qpos_idx in qpos_addrs])
solved_joints_deg = np.degrees(solved_joints_rad)

print(f"Solved Joint Angles (rad): {solved_joints_rad}")
print(f"Solved Joint Angles (deg): {solved_joints_deg}")

# --- MAP SOLVED ANGLES TO SERVO HOME POSITIONS (Channels 0 to 5) ----------------------------------
# Specified servo rest/home positions for channels 0 through 5:
servo_home_deg = np.array([90.0, 0.0, 0.0, 0.0, 90.0, 0.0])

# DIRECTION MULTIPLIERS: Toggle any index between 1.0 and -1.0 if unclipped angles go out of [0, 180] bounds
servo_directions = np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0])

# Convert solved radians to degrees, apply direction multiplier, and add home position offsets
raw_servo_deg = (solved_joints_deg * servo_directions) + servo_home_deg
print("--- SERVO ANGLE MAPPING ---")
for i, val in enumerate(raw_servo_deg):
    print(f"Channel {i}: Unclipped = {val:.2f}°")

# Clamp angles safely within physical servo limits (0° to 180°)[cite: 17, 19]
solved_servo_deg = np.clip(raw_servo_deg, 0.0, 180.0)
for i, val in enumerate(solved_servo_deg):
    print(f"Channel {i}: Clamped   = {val:.2f}°")


# --- MAIN EXECUTION SCRIPT (SERIAL + MUJOCO VISUALISER) -------------------------------------------
def main():
    # 1. Connect to Real Robot via Serial
    print(f"Connecting to real robot on {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino UNO reset on connection[cite: 19]
    except Exception as e:
        print(f"Could not connect to serial port: {e} (Running simulation only)")

    try:
        if ser and ser.is_open:
            print("Sending solved joint angles to real robot channels...")
            for channel, angle in enumerate(solved_servo_deg):
                cmd = f"joint {channel} {angle:.1f} {MOVE_TIME}\n"
                ser.write(cmd.encode('utf-8'))
                ser.flush()
                print(f"Sent -> Channel {channel}: {angle:.1f}°")
                time.sleep(0.2)  # 200ms delay for Arduino parsing

            print(f"Waiting {MOVE_TIME / 1000.0} seconds for real robot movement...")
            time.sleep(MOVE_TIME / 1000.0)

        # 2. Launch MuJoCo Simulation Visualiser
        print("Launching MuJoCo Visualiser (Close window to exit and home robot)...")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            if target_mocap_id != -1:
                data.mocap_pos[target_mocap_id] = t_pos
                data.mocap_quat[target_mocap_id] = t_quat

            # Apply solved simulation joint positions
            for i, qpos_idx in enumerate(qpos_addrs):
                data.qpos[qpos_idx] = solved_joints_rad[i]

            while viewer.is_runnning() if hasattr(viewer, 'is_runnning') else viewer.is_running():
                step_start = time.time()
                
                for i, qpos_idx in enumerate(qpos_addrs):
                    data.qpos[qpos_idx] = solved_joints_rad[i]
                    data.ctrl[actuator_ids[i]] = solved_joints_rad[i]

                mujoco.mj_step(model, data)
                viewer.sync()

                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # 3. Clean up Serial Connection and Home Robot on Exit (Triggered whether window is closed or Ctrl+C is pressed)
        if ser and ser.is_open:
            print(f"Returning robot to home position ({MOVE_TIME} ms)...")
            home_cmd = f"home {MOVE_TIME}\n"
            ser.write(home_cmd.encode('utf-8'))
            ser.flush()
            
            # Allow full duration of movement time plus a buffer for the Arduino serial buffer to process
            h_sleep_time = (MOVE_TIME / 1000.0) + 1.5
            print(f"Waiting {h_sleep_time}s for homing sequence to finish...")
            time.sleep(h_sleep_time)
            
            ser.close()
            print("Disconnected cleanly from serial port.")

if __name__ == "__main__":
    main()