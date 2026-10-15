# --- IMPORTS --------------------------------------------------------------------------------------
import time
import mujoco
import mujoco.viewer
import numpy as np


# --- VARIABLES ------------------------------------------------------------------------------------
# load model -> path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../arm-7dof_mujoco/scene.xml")
data  = mujoco.MjData(model)

# disable contact physics
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

def disable_collision_between_bodies(model, body1_name):
    body1_id = model.body(body1_name).id
    
    # Iterate over all geoms attached to body1 and disable their contact properties
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] == body1_id:
            # Setting contype and conaffinity to 0 prevents contact generation
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0

# disable_collision_between_bodies(model, "base")


# --- TARGET PARAMETERS ----------------------------------------------------------------------------
# target frame
target_mocap_id = model.body('target_frame').mocapid[0]
# target position vector
t_pos = np.array([0.0, -0.1, 0.15])
# target orientation quaternion [w,x,y,z] (mujoco format)
t_quat = np.array([1.0, 0.0, 0.0, 0.0])


# --- IK SOLVER ------------------------------------------------------------------------------------


# --- VISUALISER -----------------------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        data.mocap_pos[target_mocap_id] = t_pos
        data.mocap_quat[target_mocap_id] = t_quat

        # step physics forward
        mujoco.mj_step(model, data)

        # refresh viewer each step
        viewer.sync()

        # real-time sim sleep
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)