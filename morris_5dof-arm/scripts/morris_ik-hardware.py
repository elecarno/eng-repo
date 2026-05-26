# MORRIS INVERSE KINEMATICS HARDWARE SCRIPT
# This script acts as a basic implementation of the inverse kinematics solver for the Morris robot.


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


# --- FUNCTIONS ------------------------------------------------------------------------------------
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
    if len(pulse_widths) != 5:
        print("Error: You must provide exactly 5 pulse width values.")
        return

    # attempt to send data
    try:
        # generate format: "C0:1500,C1:2500,C2:500,C3:1500,C4:1500\n"
        msg = ",".join([f"C{i}:{int(val)}" for i, val in enumerate(pulse_widths)]) + "\n"
        ser.write(msg.encode('utf-8'))
        print(f"Sent: {msg.strip()}")
    except Exception as e:
        print(f"Failed to send data: {e}")


def reset_to_rest_pose():
    """
        Defines and sets the robot to its resting, or zero, position.
    """

    # [J1/base, J2/shoulder, J3/elbow, J4/wrist, J5/cuff]
    rest_poses = [
        1500, 
        500, 
        500, 
        1500, 
        1500
    ]
    print("Resetting to rest position...")
    send_pulse_widths(rest_poses)


def ik_solver(robot_desc, target_3d, phi):
    """
    Calculates the angles between joints for the robot to target a specific point in 3D space.

    Args:
        robot_desc: A dictionary describing the physical parameters of the robot.
        target_3d: The point in 3D space which the IK is targeted towards (given as a numpy array).
        phi: The orientation angle of the end-effector of the robot relative to the target point.

    Returns:
        A list containing four angles corresponding to the first four joints of the robot.
    """
    
    print("RUNNING IK SOVER ----------------------------------------------------")
    # get variables from robot description
    h   = robot_desc["h"  ]
    L2x = robot_desc["L2x"]
    L2y = robot_desc["L2y"]
    L3  = robot_desc["L3" ]
    L4  = robot_desc["L4" ]

    # define arm plane points
    target_2d_x = np.sqrt( target_3d[0]**2 + target_3d[1]**2 )
    target_2d_y = target_3d[2]
    target_2d = np.array([target_2d_x, target_2d_y])
    target_2d_offset = ([
        target_2d[0] - L4*np.cos(phi),
        target_2d[1] - L4*np.sin(phi) - h
    ])
    print(f"\ntarget (3d): {target_3d}\ntarget (2d): {target_2d}\ntarget offset (2d): {target_2d_offset}")

    # intermediate variables
    theta_0 = np.arctan2(L2y, L2x)
    d = np.sqrt( target_2d_offset[0]**2 + target_2d_offset[1]**2 )
    L2 = np.sqrt( L2x**2 + L2y**2 )
    print(f"\ntheta_0: {theta_0}\nd: {d}\nL2: {L2}")

    # calculate joint angles
    theta_1 = np.arctan2(target_3d[1], target_3d[0])
    theta_2 = np.arctan2(target_2d_offset[1], target_2d_offset[0]) + np.arccos( (L2**2 + d**2 - L3**2) / (2*L2*d) ) + theta_0
    theta_3 = np.arccos( (L2**2 + L3**2 - d**2) / (2*L2*L3) ) - theta_0 - np.pi/2
    theta_4 = phi - (theta_2 + theta_3) + np.pi/2
    print(f"\ntheta_1: {theta_1}\ntheta_2: {theta_2}\ntheta_3: {theta_3}\ntheta_4: {theta_4}\n")

    return [ theta_1, theta_2, theta_3, theta_4 ]


def ik_to_robot(joint_ik):
    """
    Converts the ik angles given by ik_solver() from angles between each joint to servo angles.

    Args:
        joint_ik: A list of four angles as outputted by ik_solver()
    
    Returns:
        A list containing four angles corresponding to the first four servos of the robot.
    """

    print("RUNNING IK TO TRUE JOINT CONVERTER ----------------------------------")
    alpha_1 = np.pi + joint_ik[0]
    alpha_2 = np.pi - joint_ik[1]
    alpha_3 = joint_ik[2] + np.pi/2
    alpha_4 = np.pi/2 - joint_ik[3]

    print(f"\nalpha_1: {alpha_1}\nalpha_2: {alpha_2}\nalpha_3: {alpha_3}\nalpha_4: {alpha_4}\n")

    return [ alpha_1, alpha_2, alpha_3, alpha_4 ]


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
    # define robot and run ik solver
    robot_desc = {
        "h": 0.086,
        "L2x": 0.1,
        "L2y": 0.032,
        "L3": 0.09,
        "L4": 0.062
    }
    target_3d = np.array([-0.05, -0.1, 0.15])
    phi = np.radians(0)
    
    joint_ik = ik_solver(robot_desc, target_3d, phi)
    joint_angles = ik_to_robot(joint_ik)

    # send joint angles to robot
    print("\nSENDING JOINT ANGLE DATA ------------------------------------------")
    try:
        # initialize to zero position on startup
        reset_to_rest_pose()
        time.sleep(1)

        # define target positions
        target_positions = [
            radians_to_us(joint_angles[0]), 
            radians_to_us(joint_angles[1]), 
            radians_to_us(joint_angles[2]), 
            radians_to_us(joint_angles[3]), 
            1500
            ]
        
        # send target position data
        send_pulse_widths(target_positions)
        time.sleep(2) # give hardware time to move

        # return to zero position before exiting
        reset_to_rest_pose()

    finally:
        # close the serial port before exiting
        if ser:
            ser.close()
            print("Serial connection closed.")