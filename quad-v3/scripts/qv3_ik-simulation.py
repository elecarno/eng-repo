# STEVEN ROBOT MUJOCO INVERSE KINEMATICS SIMULATION

# --- IMPORTS --------------------------------------------------------------------------------------
import time
import mujoco
import mujoco.viewer
import numpy as np


# --- LOAD MODEL AS GLOBAL -------------------------------------------------------------------------
# path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../q-v3_mujoco/scene.xml")
data = mujoco.MjData(model)


# --- FUNCTIONS ------------------------------------------------------------------------------------
def get_actuators():
    leg_fl = [
        model.actuator("fl-hip").id,
        model.actuator("fl-knee").id,
        model.actuator("fl-ankle").id,
    ]
    leg_fr = [
        model.actuator("fr-hip").id,
        model.actuator("fr-knee").id,
        model.actuator("fr-ankle").id,
    ]
    leg_bl = [
        model.actuator("bl-hip").id,
        model.actuator("bl-knee").id,
        model.actuator("bl-ankle").id,
    ]
    leg_br = [
        model.actuator("br-hip").id,
        model.actuator("br-knee").id,
        model.actuator("br-ankle").id,
    ]

    legs = {
        "fl": leg_fl,
        "fr": leg_fr,
        "bl": leg_bl,
        "br": leg_br
    }
    return legs

def get_leg_actuator_transforms(leg):
    # correctional transform coefficients

    if leg == "fl":
        return [ -1, 1, 1 ]
    elif leg == "fr":
        return [ 1, -1, -1 ]
    elif leg == "br":
        return [ -1, 1, 1 ]
    elif leg == "bl":
        return [ 1, -1, -1 ]
    else:
        return [0, 0, 0]

def set_leg_position(leg, position):
    legs = get_actuators()

    hip   = legs[leg][0]
    knee  = legs[leg][1]
    ankle = legs[leg][2]

    coeffs = get_leg_actuator_transforms(leg)

    data.ctrl[hip]   = position[0]*coeffs[0]
    data.ctrl[knee]  = position[1]*coeffs[1]
    data.ctrl[ankle] = position[2]*coeffs[2]


# --- LEG IK SOLVER --------------------------------------------------------------------------------
def leg_ik_solver(target_3d):
    # link lengths (m)
    L1  = 0.065
    L2  = 0.060
    L3x = 0.015
    L3y = 0.145

    # link 3 offsets
    L3 = np.sqrt(L3x**2 + L3y**2)
    L3_theta = np.arcsin(L3x/L3)

    # target position in leg plane
    target_2d = np.array([
        np.sqrt(target_3d[0]**2 + target_3d[1]**2) - L1,
        target_3d[2]
    ])

    # 2D angles
    gamma = np.atan2(target_2d[1], target_2d[0])
    beta = np.arccos(
        (L2**2 + L3**2 - target_2d[0]**2 - target_2d[1]**2) / (2*L2*L3)
    )
    alpha = np.arccos(
        (target_2d[0]**2 + target_2d[1]**2 + L2**2 - L3**2) / (2 * L2 * np.sqrt(target_2d[0]**2 + target_2d[1]**2))
    )

    # joint direct angles
    theta_1 = np.atan2(target_3d[1], target_3d[0])
    theta_2 = gamma + alpha
    theta_3 = beta - np.pi - L3_theta

    # joint angles with actuator offsets
    theta_1j = np.pi/2 + theta_1
    theta_2j = np.pi/2 - theta_2
    theta_3j = np.pi + theta_3

    return [theta_1j, theta_2j, theta_3j]


# --- MAIN -----------------------------------------------------------------------------------------
if __name__ == "__main__":
    # --- RESTING POSITION ---
    set_leg_position("fl", [
        np.radians(45), 
        np.radians(20), 
        np.radians(30)
    ])
    set_leg_position("fr", [
        np.radians(45), 
        np.radians(20), 
        np.radians(30)
    ])
    set_leg_position("br", [
        np.radians(45), 
        np.radians(20), 
        np.radians(30)
    ])
    set_leg_position("bl", [
        np.radians(45), 
        np.radians(20), 
        np.radians(30)
    ])

    # --- VISUALISER ----
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # start timer
            step_start = time.time()

            # --- LEG MOVEMENT ---
            # get timer
            t = data.time
            frequency = 5

            # define leg ik targets & set positions
            fl_x = 0.18
            fl_y = 0.06*np.maximum(0, np.cos(t * frequency))
            fl_z = 0.04*np.maximum(0, np.sin(t * frequency)) - 0.07
            fl_ik = leg_ik_solver(np.array([fl_x, fl_y, fl_z]))
            set_leg_position("fl", [ fl_ik[0], fl_ik[1], fl_ik[2] ])

            br_offset = 0
            br_x = 0.18
            br_y = -0.06*np.maximum(0, np.cos(t * frequency + br_offset))
            br_z = 0.04*np.maximum(0, np.sin(t * frequency + br_offset)) - 0.07
            br_ik = leg_ik_solver(np.array([br_x, br_y, br_z]))
            set_leg_position("br", [ br_ik[0], br_ik[1], br_ik[2] ])

            fr_offset = np.pi * frequency
            fr_x = 0.18
            fr_y = 0.06*np.maximum(0, np.cos(t * frequency + fr_offset))
            fr_z = 0.04*np.maximum(0, np.sin(t * frequency + fr_offset)) - 0.07
            fr_ik = leg_ik_solver(np.array([fr_x, fr_y, fr_z]))
            set_leg_position("fr", [ fr_ik[0], fr_ik[1], fr_ik[2] ])

            bl_offset = np.pi * frequency
            bl_x = 0.18
            bl_y = -0.06*np.maximum(0, np.cos(t * frequency + bl_offset))
            bl_z = 0.04*np.maximum(0, np.sin(t * frequency + bl_offset)) - 0.07
            bl_ik = leg_ik_solver(np.array([bl_x, bl_y, bl_z]))
            set_leg_position("bl", [ bl_ik[0], bl_ik[1], bl_ik[2] ])
            # --- LEG MOVEMENT ---

            # step physics forward & refresh viewer each step
            mujoco.mj_step(model, data)
            viewer.sync()

            # real-time sim
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
