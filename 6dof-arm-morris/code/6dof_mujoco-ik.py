import time
import mujoco
import mujoco.viewer
import numpy as np

# load model -> path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../6dof-morris_mujoco/scene.xml")
data = mujoco.MjData(model)

# disable contact physics
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# target frame
target_mocap_id = model.body('target_frame').mocapid[0]
# target position vector
target_pos = np.array([0.1, 0.1, 0.1])
# target orientation quaternion [w,x,y,z] (mujoco format)
target_quat = np.array([1.0, 0.0, 0.0, 0.0])

# get joints
joint1 = model.actuator("joint1").id
joint2 = model.actuator("joint2").id
joint3 = model.actuator("joint3").id
joint4 = model.actuator("joint4").id
joint5 = model.actuator("joint5").id

# set joint positions (radians for rotation, meters for linear)
rest = [ # resting position of joints
    90,
    180,
    180,
    0,
    90
]

joints = [ # position to set joints to
    90,
    180,
    180,
    0,
    90
]

data.ctrl[joint1] = np.radians(joints[0] - rest[0])
data.ctrl[joint2] = np.radians(joints[1] - rest[1])
data.ctrl[joint3] = np.radians(joints[2] - rest[2])
data.ctrl[joint4] = np.radians(joints[3] - rest[3])
data.ctrl[joint5] = np.radians(joints[4] - rest[4])

# open interactive visualizer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # apply target transforms
        data.mocap_pos[target_mocap_id] = target_pos
        data.mocap_quat[target_mocap_id] = target_quat

        # step physics forward
        mujoco.mj_step(model, data)

        # refresh viewer each step
        viewer.sync()

        # real-time sim
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)