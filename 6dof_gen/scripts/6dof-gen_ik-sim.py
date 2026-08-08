# STEVEN ROBOT MUJOCO INVERSE KINEMATICS SIMULATION

# --- IMPORTS --------------------------------------------------------------------------------------
import time
import mujoco
import mujoco.viewer
import numpy as np
import keyboard

# --- LOAD MODEL AS GLOBAL -------------------------------------------------------------------------
# path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../6dof-gen_mujoco/scene.xml")
data = mujoco.MjData(model)


# --- MATHS FUNCTIONS ------------------------------------------------------------------------------
def skew_matrix(v):
    M = np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ]
    ])
    return M

def skew_squared_matrix(v):
    M = np.array([
        [-(v[1]**2 + v[2]**2),   v[0]*v[1],             v[0]*v[2]         ],
        [ v[0]*v[1],            -(v[0]**2 + v[2]**2),   v[1]*v[2]         ],
        [ v[0]*v[2],             v[1]*v[2],           -(v[0]**2 + v[1]**2)]
    ])

def planar_2r_to_3d(x, y, theta):
    return np.array([
        x*np.cos(theta),
        x*np.sin(theta),
        y
    ])


# --- LEG IK SOLVER --------------------------------------------------------------------------------
def ik_solver(T_3d, phi_x, phi_y, phi_z):
    # link lengths
    L1 = 0.027
    L2 = 0.120
    L3 = 0.060
    L4 = 0.064
    L5 = 0.025

    # end effector rotation matrix
    Rot_E = np.array(
        [
            [
                np.cos(phi_z) * np.cos(phi_y),
                np.cos(phi_z) * np.sin(phi_y) * np.sin(phi_x)
                - np.sin(phi_z) * np.cos(phi_x),
                np.cos(phi_z) * np.sin(phi_y) * np.cos(phi_x)
                + np.sin(phi_z) * np.sin(phi_x),
            ],
            [
                np.sin(phi_z) * np.cos(phi_y),
                np.sin(phi_z) * np.sin(phi_y) * np.sin(phi_x)
                + np.cos(phi_z) * np.cos(phi_x),
                np.sin(phi_z) * np.sin(phi_y) * np.cos(phi_x)
                - np.cos(phi_z) * np.sin(phi_x),
            ],
            [
                -np.sin(phi_y),
                np.cos(phi_y) * np.sin(phi_x),
                np.cos(phi_y) * np.cos(phi_x),
            ],
        ]
    )

    # spherical offset target point
    T_offset = T_3d - L5 * np.array([Rot_E[0, 2], Rot_E[1, 2], Rot_E[2, 2]])

    # end effector frame axes
    T_x = T_3d + Rot_E[:, 0]
    T_y = T_3d + Rot_E[:, 1]
    T_z = T_3d + Rot_E[:, 2]

    theta1 = np.arctan2(T_3d[1], T_3d[0])

    # 2R solver -----------------------------------------------
    T_2r = np.array(
        [
            np.sqrt(T_offset[0] ** 2 + T_offset[1] ** 2),
            T_offset[2],
        ]
    )

    gamma = np.arctan2(T_2r[1], T_2r[0])

    beta = np.arccos(
        np.clip(
            (L2**2 + (L3 + L4) ** 2 - T_2r[0] ** 2 - T_2r[1] ** 2)
            / (2 * L2 * (L3 + L4)),
            -1.0,
            1.0,
        )
    )

    alpha = np.arccos(
        np.clip(
            (T_2r[0] ** 2 + T_2r[1] ** 2 + L2**2 - (L3 + L4) ** 2)
            / (2 * L2 * np.sqrt(T_2r[0] ** 2 + T_2r[1] ** 2)),
            -1.0,
            1.0,
        )
    )

    theta2 = gamma + alpha
    theta3 = beta - np.pi

    J3_planar = np.array([L2 * np.cos(theta2), L2 * np.sin(theta2)])
    J4_planar = np.array(
        [
            L2 * np.cos(theta2) + (L3 + L4) * np.cos(theta2 + theta3),
            L2 * np.sin(theta2) + (L3 + L4) * np.sin(theta2 + theta3),
        ]
    )

    # end-effector solver -------------------------------------
    J1 = np.array([0, 0, 0])
    J3 = planar_2r_to_3d(J3_planar[0], J3_planar[1], theta1)
    J4 = planar_2r_to_3d(J4_planar[0], J4_planar[1], theta1)

    # wrist frame axes
    J_4x = np.cross(J1 - J4, J3 - J4) / np.linalg.norm(
        np.cross(J1 - J4, J3 - J4)
    )
    J_4z = (J4 - J3) / np.linalg.norm(J4 - J3)
    J_4y = np.cross(J_4z, J_4x) / np.linalg.norm(np.cross(J_4z, J_4x))

    Rot_J4 = np.column_stack((J_4x, J_4y, J_4z))

    # relative rotation matrix
    Q = Rot_J4.T @ Rot_E

    theta4 = np.arctan2(Q[1, 2], Q[0, 2])
    theta5 = np.arctan2(np.sqrt(Q[2, 0] ** 2 + Q[2, 1] ** 2), Q[2, 2])
    theta6 = np.arctan2(Q[2, 1], -Q[2, 0])

    # joint offset angles
    theta1j = theta1
    theta2j = theta2
    theta3j = theta3
    theta4j = theta4
    theta5j = theta5
    theta6j = theta6

    return [theta1j, theta2j, theta3j, theta4j, theta5j, theta6j]


# --- MAIN -----------------------------------------------------------------------------------------
if __name__ == "__main__":
    # --- VISUALISER ----
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # start timer
            step_start = time.time()

            # --- IK IMPLEMENTATION ---
            # get timer
            t = data.time

            target_3d = np.array([0.05, 0.05, 0.15])
            phi_x = 0
            phi_y = np.pi/2
            phi_z = 0

            joint_angles = ik_solver(target_3d, phi_x, phi_y, phi_z)

            # print(joint_angles)
            data.ctrl[model.actuator("base").id] = -joint_angles[0]
            data.ctrl[model.actuator("shoulder").id] = -joint_angles[1] + np.pi/2
            data.ctrl[model.actuator("elbow").id] = -joint_angles[2]
            data.ctrl[model.actuator("wrist1").id] = joint_angles[3] + np.pi/2
            data.ctrl[model.actuator("wrist2").id] = -joint_angles[4]
            data.ctrl[model.actuator("wrist3").id] = joint_angles[5] - np.pi/2

            # step physics forward & refresh viewer each step
            mujoco.mj_step(model, data)
            viewer.sync()

            # real-time sim
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
