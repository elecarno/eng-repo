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

# target position vector
t_pos = np.array([0.0, -0.04, 0.24])
# target orientation (euler in degrees, zyx)
t_eul = [0, 0, 0]
# target orientation (rotation matrix)
t_rot = R.from_euler("zyx", t_eul, degrees=True).as_matrix()
# target orientation quaternion [w,x,y,z] (mujoco format)
t_quat_scipy = R.from_euler("zyx", t_eul, degrees=True).as_quat()
t_quat = np.array([t_quat_scipy[3], t_quat_scipy[0], t_quat_scipy[1], t_quat_scipy[2]])

# get joints
joint1 = model.actuator("j1").id
joint2 = model.actuator("j2").id
joint3 = model.actuator("j3").id
joint4 = model.actuator("j4").id
joint5 = model.actuator("j5").id

# --- IK SOLVER ------------------------------------------------------------------------------------
print("BEGINNING IK SOLVER")

# height offset of J2 from the ground
h = 0.110

# lengths betweens J2 and J3
L1x = 179 / 1000
L1y = 11.85 / 1000
L1 = np.sqrt( L1x**2 + L1y**2 )

# length between J3 and J5
L2 = 110.15 / 1000

# wrist position
pos_w = t_pos
pos_w2d = np.array([ # for 2R solver
    np.sqrt( pos_w[0]**2 + pos_w[1]**2 ),
    pos_w[2] - h
])

print(f"2d wrist position: {pos_w2d}")

# joint 1 (base) rotation
theta1 = np.arctan2(pos_w[1], pos_w[0])

# 2R open chain
theta0 = np.arctan2(L1y, L1x) # angle of the L1 line when robot is at rest
d = np.sqrt( pos_w2d[0]**2 + pos_w2d[1]**2 ) # distance of the line between J2 and J5
theta2 = np.arctan2(pos_w2d[1], pos_w2d[0]) + np.arccos( (L1**2 + d**2 - L2**2) / (2*L1*d) ) + theta0
theta3 = np.arccos( (L1**2 + L2**2 - d**2) / (2*L1*L2) ) - theta0 - np.pi/2

print(f"theta0: {theta0}")
print(f"d: {d}")
print(f"theta2: {theta2}")
print(f"theta3: {theta3}")


# position to set joints to
joints = [
    -(theta1 + np.pi/2),
    theta2 - np.pi,
    -(theta3 + np.pi/2),
    0,
    0
]

data.ctrl[joint1] = joints[0]
data.ctrl[joint2] = joints[1]
data.ctrl[joint3] = joints[2]
data.ctrl[joint4] = joints[3]
data.ctrl[joint5] = joints[4]


# --- VISUALISER -----------------------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # apply target transforms
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
