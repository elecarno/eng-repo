# --- IMPORTS ---------------------------------------------------------------------------------------
import serial
import time
import numpy as np


# --- SERIAL ---------------------------------------------------------------------------------------
try:
    ser = serial.Serial('COM4', 115200, timeout=1)
    time.sleep(2)  # wait for ESP32 serial pipeline to stabilize
    print("Connected to PCA9685/ESP32")
except Exception as e:
    print(f"Connection Error: {e}")
    ser = None


def send_pulse_widths(pulse_widths):
    """
    Packages and transmits raw microsecond values to the PCA9685.

    Args:
        pulse_widths: An array of 5 integers between 500 and 2500 repesenting microsecond values.
    """

    # check if serial is open
    if not ser or not ser.is_open:
        print("Error: Serial port is not open.")
        return

    # check if there are five PWM values given
    if len(pulse_widths) != 14:
        print("Error: You must provide exactly 14 pulse width values.")
        return

    # attempt to send data
    try:
        # generate format: "C0:1500,C1:2500,C2:500,C3:1500,C4:1500\n"
        msg = ",".join([f"C{i}:{int(val)}" for i, val in enumerate(pulse_widths)]) + "\n"
        ser.write(msg.encode('utf-8'))
        print(f"Sent: {msg.strip()}")
    except Exception as e:
        print(f"Failed to send data: {e}")


# --- IK SOLVER ------------------------------------------------------------------------------------
def reset_to_rest_pose():
    """
        Defines and sets the robot to its resting, or zero, position.
    """

    rest_poses = [
        500, 2500, 2500,

        2500, 500, 500,

        2500, 500, 500,

        500, 2500, 2500,

        600, 
        2400
    ]
    print("Resetting to rest position...")
    send_pulse_widths(rest_poses)


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

    print(theta_1, theta_2, theta_3)

    # joint angles with actuator offsets
    theta_1j = np.pi/2 + theta_1
    theta_2j = np.pi/2 - theta_2
    theta_3j = np.pi + theta_3

    print(theta_1j, theta_2j, theta_3j)

    return [theta_1j, theta_2j, theta_3j]


# def ik_to_robot(joint_ik):
#     print("RUNNING IK TO TRUE JOINT CONVERTER ----------------------------------")
#     alpha_1 = np.pi + joint_ik[0]
#     alpha_2 = np.pi - joint_ik[1]
#     alpha_3 = joint_ik[2] + np.pi/2
#     alpha_4 = np.pi/2 - joint_ik[3]

#     print(f"\nalpha_1: {alpha_1}\nalpha_2: {alpha_2}\nalpha_3: {alpha_3}\nalpha_4: {alpha_4}\n")

#     return [ alpha_1, alpha_2, alpha_3, alpha_4 ]


def radians_to_us(rad, rad_min=0.0, rad_max=np.pi, us_min=500, us_max=2500):
    # linear values mapping
    total_rad_range = rad_max - rad_min
    total_us_range  = us_max  - us_min

    # calc microseconds value
    us_value = us_min + ((rad - rad_min) / total_rad_range) * total_us_range
    
    # clamp values for safety
    clamped_us = max(us_min, min(us_max, int(us_value)))
    
    return clamped_us


# --- MAIN -----------------------------------------------------------------------------------------
if __name__ == "__main__":
    # run ik solver
    fl_target = np.array([0.1, -0.1, -0.12])
    fl_ik = leg_ik_solver(fl_target)
    fr_target = np.array([0.1, 0.1, -0.12])
    fr_ik = leg_ik_solver(fr_target)
    bl_target = np.array([0.1, 0.1, -0.12])
    bl_ik = leg_ik_solver(bl_target)
    br_target = np.array([0.1, -0.1, -0.12])
    br_ik = leg_ik_solver(br_target)

    # send joint angles to robot
    print("\nSENDING JOINT ANGLE DATA ------------------------------------------")
    try:
        # initialize to zero position on startup
        reset_to_rest_pose()
        time.sleep(1)

        # define target positions
        target_positions = [
            radians_to_us(fl_ik[0]),    
            radians_to_us(np.pi - fl_ik[1]), 
            radians_to_us(np.pi - fl_ik[2]),

            radians_to_us(fr_ik[0]), 
            radians_to_us(fr_ik[1]), 
            radians_to_us(fr_ik[2]),

            radians_to_us(bl_ik[0]), 
            radians_to_us(bl_ik[1]),
            radians_to_us(bl_ik[2]),

            radians_to_us(br_ik[0]), 
            radians_to_us(np.pi - br_ik[1]), 
            radians_to_us(np.pi - br_ik[2]),

            600, 
            2400
        ]
        
        # send target position data
        send_pulse_widths(target_positions)
        time.sleep(2) # give hardware time to move

        # return to zero position before exiting
        # reset_to_rest_pose()

    finally:
        # close the serial port before exiting
        if ser:
            ser.close()
            print("Serial connection closed.")