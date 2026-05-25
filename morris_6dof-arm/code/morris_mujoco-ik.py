import time
import mujoco
import mujoco.viewer
import numpy as np

# load model -> path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../morris_mujoco/scene.xml")
data = mujoco.MjData(model)

# disable contact physics
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# target frame
target_mocap_id = model.body('target_frame').mocapid[0]
# target position vector
t_pos = np.array([0.0, -0.1, 0.15])
# target orientation quaternion [w,x,y,z] (mujoco format)
t_quat = np.array([1.0, 0.0, 0.0, 0.0])

# get joints
joint1 = model.actuator("joint1").id
joint2 = model.actuator("joint2").id
joint3 = model.actuator("joint3").id
joint4 = model.actuator("joint4").id
joint5 = model.actuator("joint5").id

# joint rest positions (radians for rotation, meters for linear)
rest = [ # resting position of joints
    90,
    180,
    180,
    90,
    90
]

# IK SOLVER ----------------------------------------------------------------------------------------
print("BEGINNING IK SOLVER")

h = 0.086

L2x = 0.1
L2y = 0.032
L3  = 0.09
L4  = 0.062

phi = np.radians(0)

target_x = np.sqrt( t_pos[0]**2 + t_pos[1]**2 )
target_y = t_pos[2]

target = np.array([target_x, target_y])
target_offset = ([
    target[0] - L4*np.cos(phi),
    target[1] - L4*np.sin(phi) - h
])

print(f"\nTARGET: {target}\nOFFSET TARGET: {target_offset}")

theta_0 = np.arctan2(L2y, L2x)
d = np.sqrt( target_offset[0]**2 + target_offset[1]**2 )
L2 = np.sqrt( L2x**2 + L2y**2 )

target_distance = np.sqrt(target_x**2 + target_y**2)
max_distance = L2+L3
if target_distance > max_distance:
    print("TARGET OUTSIDE OF WORKSPACE")

print(f"\ntheta_0: {theta_0}\nd: {d}\nL2: {L2}")

theta_2 = np.arctan2(target_offset[1], target_offset[0]) + np.arccos( (L2**2 + d**2 - L3**2) / (2*L2*d) ) + theta_0
theta_3 = np.arccos( (L2**2 + L3**2 - d**2) / (2*L2*L3) ) - theta_0 - np.pi/2
theta_4 = phi - (theta_2 + theta_3) + np.pi/2

theta_1 = -np.arctan2(t_pos[1], t_pos[0])

print(f"\ntheta_1: {theta_1}\ntheta_2: {theta_2}\ntheta_3: {theta_3}\ntheta_4: {theta_4}\n")

# set joint targets
joints = [ # position to set joints to
    theta_1 - np.pi/2,
    theta_2 - np.pi,
    -(np.pi/2) - theta_3,
    theta_4,
    np.radians(90) - np.pi/2,
]

data.ctrl[joint1] = joints[0] 
data.ctrl[joint2] = joints[1]
data.ctrl[joint3] = joints[2]
data.ctrl[joint4] = joints[3]
data.ctrl[joint5] = joints[4]
# IK SOLVER ^^^ ------------------------------------------------------------------------------------

# open interactive visualizer
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