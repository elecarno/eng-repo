# MORRIS ROBOT DIRECT CONTROL INTERFACE
# This script is used to connect to the Morris robot and provides a basic GUI that allows the user
# to direct control the position of each joint using sliders


# --- IMPORTS --------------------------------------------------------------------------------------
import tkinter as tk
from tkinter import ttk
import serial
import time


# --- SERIAL ---------------------------------------------------------------------------------------
try:
    ser = serial.Serial('COM4', 115200, timeout=1)
    time.sleep(2)  # Wait for ESP32 serial pipeline to stabilize
    print("Connected to PCA9685/ESP32 Controller!")
except Exception as e:
    print(f"Connection Error: {e}")
    ser = None


def send_pulse_widths(*args):
    """
        Packages and transmits raw microsecond values to the PCA9685.
    """
    if ser and ser.is_open and len(sliders) == 5:
        try:
            # generate format: "C0:1500,C1:2500,C2:500,C3:1500,C4:1500\n"
            msg = ",".join(
                [f"C{i}:{int(slider.get())}" for i, slider in enumerate(sliders)]
            ) + "\n"
            ser.write(msg.encode('utf-8'))
        except NameError:
            pass  # protection during initialization


# --- IK SOLVER ------------------------------------------------------------------------------------
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


# --- GUI ------------------------------------------------------------------------------------------
def create_us_slider(label_text, default_us):
    frame = ttk.Frame(root)
    frame.pack(fill='x', padx=20, pady=10)
    
    label = ttk.Label(frame, text=label_text, width=22)
    label.pack(side='left')
    
    slider = ttk.Scale(frame, from_=500, to=2500, orient='horizontal', command=send_pulse_widths)
    slider.set(default_us)
    slider.pack(side='right', expand=True, fill='x')
    return slider


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Morris 5-DOF IK Controller")
    root.geometry("450x480")

    ttk.Label(root, text="Morris 5-DOF IK Controller", font=("Arial", 14, "bold")).pack(pady=15)

    # list to hold slider references
    sliders = []

    configs = [
        ("Channel 0 - (J1) Base:",      1500 ),  # USMID
        ("Channel 1 - (J2) Shoulder:",  500  ),  # USMIN
        ("Channel 2 - (J3) Elbow:",     500  ),  # USMIN
        ("Channel 3 - (J4) Wrist:",     1500 ),  # USMID
        ("Channel 4 - (J5) Cuff:",      1500 )   # USMID
    ]

    # create sliders
    for label_text, default_val in configs:
        sliders.append(create_us_slider(label_text, default_val))

    # rest function for 5 servos
    def reset_to_rest_pose():
        rest_poses = [1500, 500, 500, 1500, 1500]
        for slider, pose in zip(sliders, rest_poses):
            slider.set(pose)
        send_pulse_widths()

    home_btn = ttk.Button(root, text="Return to Rest Pose", command=reset_to_rest_pose)
    home_btn.pack(pady=25)

    # syncs hardware with gui on startup
    send_pulse_widths()

    root.mainloop()

    if ser:
        ser.close()