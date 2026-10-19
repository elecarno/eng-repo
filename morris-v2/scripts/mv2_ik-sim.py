# --- IMPORTS --------------------------------------------------------------------------------------
import time
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- VARIABLES ------------------------------------------------------------------------------------
model = mujoco.MjModel.from_xml_path("../morris-v2_mujoco/scene.xml")
data  = mujoco.MjData(model)

# disable contact physics
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# target frame
target_mocap_id = model.body('target_frame').mocapid[0]

# get actuator IDs and corresponding joint addresses
actuator_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
actuator_ids = [model.actuator(name).id for name in actuator_names]
joint_ids = [model.actuator_trnid[act_id, 0] for act_id in actuator_ids]
qpos_addrs = [model.jnt_qposadr[j_id] for j_id in joint_ids]

# Find end-effector body (typically the last link or tool body)
ee_body_name = "m_v2_end_effector" if "m_v2_end_effector" in [model.body(i).name for i in range(model.nbody)] else model.body(model.nbody - 1).name
ee_body_id = model.body(ee_body_name).id

def disable_collision_on_body(model, body_name):
    body1_id = model.body(body_name).id
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] == body1_id:
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0

disable_collision_on_body(model, "m_v2_base")

# --- TARGET PARAMETERS ----------------------------------------------------------------------------
t_pos = np.array([0.1, -0.1, 0.3]) # meters
t_eul = np.array([0, 0, 90])        # degrees

# Pre-compute target quaternion once (MuJoCo expects [w, x, y, z])
t_quat_scipy = R.from_euler('ZYX', t_eul, degrees=True).as_quat()
t_quat = np.array([t_quat_scipy[3], t_quat_scipy[0], t_quat_scipy[1], t_quat_scipy[2]])

# --- NUMERICAL JACOBIAN IK SOLVER ----------------------------------------------------------------
print("RUNNING NUMERICAL JACOBIAN IK SOLVER FOR MUJOCO")

# Set initial guess to zeros
for qpos_idx in qpos_addrs:
    data.qpos[qpos_idx] = 0.0
mujoco.mj_forward(model, data)

target_rot = R.from_quat([t_quat_scipy[0], t_quat_scipy[1], t_quat_scipy[2], t_quat_scipy[3]]).as_matrix()

# Iterative Damped Least-Squares IK
max_iterations = 300
lambda_damping = 1e-2
dt_ik = 0.1

for iteration in range(max_iterations):
    mujoco.mj_forward(model, data)
    
    # Current end-effector pose
    curr_pos = data.xpos[ee_body_id]
    curr_mat = data.xmat[ee_body_id].reshape(3, 3)
    
    # Position error
    pos_err = t_pos - curr_pos
    
    # Orientation error: flipped target forward vector so tool +z aligns with target +z
    curr_forward = curr_mat[:, 2]
    target_forward = -target_rot[:, 2]  # Flipped from +z to point correctly
    rot_err = np.cross(curr_forward, target_forward)
    
    # Combined error vector (6 DOF)
    err = np.hstack((pos_err, rot_err))
    
    if np.linalg.norm(err) < 1e-4:
        print(f"IK converged at iteration {iteration}!")
        break
        
    # Compute Jacobians for the end-effector body
    jac_pos = np.zeros((3, model.nv))
    jac_rot = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jac_pos, jac_rot, ee_body_id)
    
    # Extract columns corresponding to our 6 active robot joints
    nv_indices = [model.jnt_dofadr[j_id] for j_id in joint_ids]
    
    J_pos = jac_pos[:, nv_indices]
    J_rot = jac_rot[:, nv_indices]
    J = np.vstack((J_pos, J_rot))
    
    # Damped Least Squares pseudo-inverse
    J_pinv = J.T @ np.linalg.inv(J @ J.T + (lambda_damping**2) * np.eye(6))
    
    # Joint velocity update
    dq = J_pinv @ err
    
    # Update joint positions
    for i, qpos_idx in enumerate(qpos_addrs):
        data.qpos[qpos_idx] += dq[i] * dt_ik

# Final forward kinematics update with solved qpos
mujoco.mj_forward(model, data)

# Extract solved joint positions
solved_joints = [data.qpos[qpos_idx] for qpos_idx in qpos_addrs]
print(f"Solved Joint Angles: {solved_joints}")

# --- VISUALISER -----------------------------------------------------------------------------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    data.mocap_pos[target_mocap_id] = t_pos
    data.mocap_quat[target_mocap_id] = t_quat

    while viewer.is_running():
        step_start = time.time()
        
        # Force the robot to hold the solved joint positions directly every step
        for i, qpos_idx in enumerate(qpos_addrs):
            data.qpos[qpos_idx] = solved_joints[i]
            data.ctrl[actuator_ids[i]] = solved_joints[i]

        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)