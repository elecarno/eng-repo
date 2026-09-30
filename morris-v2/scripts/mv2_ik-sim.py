# MORRIS ROBOT MUJOCO INVERSE KINEMATICS SIMULATION
# This script is used to simulate the inverse kinematics solver on the morris robot. It was developed
# to test the code before implementing it on the hardware.


# --- IMPORTS --------------------------------------------------------------------------------------
import time
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R


# --- VARIABLES ------------------------------------------------------------------------------------
# load model -> path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../morris-v2_mujoco/scene.xml")
data  = mujoco.MjData(model)

# disable contact physics
# model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# target frame
target_mocap_id = model.body('target_frame').mocapid[0]

# get joints
joint1 = model.actuator("j1").id
joint2 = model.actuator("j2").id
joint3 = model.actuator("j3").id
joint4 = model.actuator("j4").id
joint5 = model.actuator("j5").id
joint6 = model.actuator("j6").id

# --- TARGET PARAMETERS ----------------------------------------------------------------------------
# target position vector
t_pos = np.array([0.00, -0.1, 0.3])
# target orientation (euler in degrees, zyx)
t_eul = np.array([0, 0, 90])

# target orientation as rotation matrix
rot_obj = R.from_euler('ZYX', t_eul, degrees=True)
t_rot = rot_obj.as_matrix()

# --- IK SOLVER ------------------------------------------------------------------------------------
print("BEGINNING IK SOLVER")

p_b = t_pos 
d6 = 95.350 / 1000  # 95.35 mm
R_sb = t_rot

# 1. Wrist center position
# Tool approach vector is local Z (column 2 of R_sb)
p_w = p_b - d6 * R_sb[:, 2]

# 2. Joint 1 angle
theta1 = np.atan2(p_w[1], p_w[0])

# 3. Major joint dimensions
d1 = 110 / 1000
a2x = 179 / 1000
a2y = 11.85 / 1000
a2 = np.sqrt(a2x**2 + a2y**2)
theta2_offset = np.atan2(a2y, a2x)
a3 = 110.15 / 1000

r = np.sqrt(p_w[0]**2 + p_w[1]**2)
s = p_w[2] - d1

# 4. Joint 3 angle
cos_theta3 = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)
sin_theta3 = np.sqrt(1.0 - cos_theta3**2)
theta3 = np.atan2(sin_theta3, cos_theta3)

# 5. Joint 2 angle
k1 = a2 + a3 * cos_theta3
k2 = a3 * sin_theta3
theta2 = np.atan2(s, r) - np.atan2(k2, k1) - theta2_offset

# 6. Wrist orientation factoring in local Z -> global -Y rest state
# Transform rotation into the arm frame relative to the rest frame
R_30 = np.array([
    [ np.cos(theta1)*np.cos(theta2+theta3), -np.cos(theta1)*np.sin(theta2+theta3),  np.sin(theta1)],
    [ np.sin(theta1)*np.cos(theta2+theta3), -np.sin(theta1)*np.sin(theta2+theta3), -np.cos(theta1)],
    [ np.sin(theta2+theta3),                 np.cos(theta2+theta3),                 0]
])

R_36 = R_30.T @ R_sb

# Extract ZYZ Euler angles from wrist rotation matrix R_36
theta5 = np.atan2(np.sqrt(R_36[0, 2]**2 + R_36[1, 2]**2), R_36[2, 2])

if np.isclose(np.sin(theta5), 0.0, atol=1e-5):
    theta4 = 0.0
    theta6 = np.atan2(R_36[1, 0], R_36[0, 0])
else:
    theta4 = np.atan2(R_36[1, 2], R_36[0, 2])
    theta6 = np.atan2(R_36[2, 1], -R_36[2, 0])

# joints_deg = np.degrees([theta1, theta2, theta3, theta4, theta5, theta6])
# print(f"Joint Angles (deg): {np.round(joints_deg, 2)}")
joints_rad = [theta1, theta2, theta3, theta4, theta5, theta6]
print(f"Joint Angles (rad): {np.round(joints_rad, 2)}")

# position to set joints to
joints = [
    theta1 + np.pi/2,
    -theta2,
    -(theta3 - np.pi/2),
    theta4 + np.pi,
    theta5 - np.pi/2,
    theta6
]

print(f"Joint Angles (set): {np.round(joints, 2)}")

data.ctrl[joint1] = joints[0]
data.ctrl[joint2] = joints[1]
data.ctrl[joint3] = joints[2]
data.ctrl[joint4] = joints[3]
data.ctrl[joint5] = joints[4]
data.ctrl[joint6] = joints[5]


# --- VISUALISER -----------------------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # apply target transforms
        t_quat_scipy = R.from_euler('ZYX', t_eul, degrees=True).as_quat()
        t_quat = np.array([t_quat_scipy[3], t_quat_scipy[0], t_quat_scipy[1], t_quat_scipy[2]])

        data.mocap_pos[target_mocap_id] = t_pos
        data.mocap_quat[target_mocap_id] = t_quat

        # step physics forward
        mujoco.mj_step(model, data)

        # refresh viewer each step
        viewer.sync()

        # real-time sim
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
