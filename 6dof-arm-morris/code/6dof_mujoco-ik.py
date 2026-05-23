import time
import mujoco
import mujoco.viewer

# load model -> path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../6dof-morris_mujoco/scene.xml")
data = mujoco.MjData(model)

# open interactive visualizer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # step physics forward
        mujoco.mj_step(model, data)

        # refresh viewer each step
        viewer.sync()

        # real-time sim
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)