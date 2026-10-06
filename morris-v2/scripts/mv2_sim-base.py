# MORRIS V2 MUJOCO SIMULATION
# This is a template script for setting up pythion controlled mujoco simulations of the Morris v2 robot.

# --- IMPORTS --------------------------------------------------------------------------------------
import time
import mujoco
import mujoco.viewer


# --- VARIABLES ------------------------------------------------------------------------------------
# load model -> path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../morris-v2_mujoco/scene.xml")
data  = mujoco.MjData(model)

# disable contact physics
# model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

def disable_collision_between_bodies(model, body1_name, body2_name):
    body1_id = model.body(body1_name).id
    body2_id = model.body(body2_name).id
    
    # Iterate over all geoms attached to body1 and disable their contact properties
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] == body1_id:
            # Setting contype and conaffinity to 0 prevents contact generation
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0

disable_collision_between_bodies(model, "m_v2_base", "m_v2_link1")


# --- VISUALISER -----------------------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # step physics forward
        mujoco.mj_step(model, data)

        # refresh viewer each step
        viewer.sync()

        # real-time sim sleep
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


