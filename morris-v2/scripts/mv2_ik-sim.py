# MORRIS ROBOT MUJOCO INVERSE KINEMATICS SIMULATION
# This script is used to simulate the inverse kinematics solver on the morris robot. It was developed
# to test the code before implementing it on the hardware.


# --- IMPORTS --------------------------------------------------------------------------------------
import time
import mujoco
import mujoco.viewer
import numpy as np


# --- VARIABLES ------------------------------------------------------------------------------------
# load model -> path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../morris-v2_mujoco/scene.xml")
data  = mujoco.MjData(model)

# disable contact physics
# model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# target frame
# target_mocap_id = model.body('target_frame').mocapid[0]
# # target position vector
# t_pos = np.array([0.0, -0.1, 0.15])
# # target orientation quaternion [w,x,y,z] (mujoco format)
# t_quat = np.array([1.0, 0.0, 0.0, 0.0])

# get joints
joint1 = model.actuator("j1").id
joint2 = model.actuator("j2").id
joint3 = model.actuator("j3").id
joint4 = model.actuator("j4").id
joint5 = model.actuator("j5").id

# joint rest positions (radians for rotation, meters for linear)
rest = [ # resting position of joints
    90,
    180,
    90,
    90,
    90
]

# --- IK SOLVER ------------------------------------------------------------------------------------
print("BEGINNING IK SOLVER")

# joints = [ # position to set joints to
#     90,
#     180,
#     90,
#     90,
#     90
# ]

# data.ctrl[joint1] = joints[0]
# data.ctrl[joint2] = joints[1]
# data.ctrl[joint3] = joints[2]
# data.ctrl[joint4] = joints[3]
# data.ctrl[joint5] = joints[4]


# --- VISUALISER -----------------------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # apply target transforms
        # data.mocap_pos[target_mocap_id] = t_pos
        # data.mocap_quat[target_mocap_id] = t_quat

        # step physics forward
        mujoco.mj_step(model, data)

        # refresh viewer each step
        viewer.sync()

        # real-time sim
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
