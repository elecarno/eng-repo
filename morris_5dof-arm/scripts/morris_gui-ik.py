# MORRIS ROBOT INVERSE KINEMATICS CONTROL INTERFACE
# This script is used to connect to the Morris via IK kinematics
# targeted towards a point in 3D space.


# --- IMPORTS --------------------------------------------------------------------------------------
import tkinter as tk
from tkinter import ttk
import serial
import time
import numpy as np


# --- SERIAL ---------------------------------------------------------------------------------------
try:
    ser = serial.Serial('COM4', 115200, timeout=1)
    time.sleep(2)  # wait for ESP32 serial pipeline to stabilize
    print("Connected to PCA9685/ESP32 Controller!")
except Exception as e:
    print(f"Connection Error: {e}")
    ser = None


def send_pulse_widths(pulse_widths):
    """
    Packages and transmits raw microsecond values to the PCA9685.

    Args:
        pulse_widths: An array of 5 integers between 500 and 2500 repesenting microsecond values.
    """
    if not ser or not ser.is_open:
        print("Error: Serial port is not open.")
        return

    if len(pulse_widths) != 5:
        print("Error: You must provide exactly 5 pulse width values.")
        return

    try:
        msg = ",".join([f"C{i}:{int(val)}" for i, val in enumerate(pulse_widths)]) + "\n"
        ser.write(msg.encode('utf-8'))
        print(f"Sent Pulse Widths: {msg.strip()}")
    except Exception as e:
        print(f"Failed to send data: {e}")


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


def radians_to_us(rad, rad_min=0.0, rad_max=np.pi, us_min=500, us_max=2500):
    # linear values mapping
    total_rad_range = rad_max - rad_min
    total_us_range  = us_max  - us_min

    # calc microseconds value
    us_value = us_min + ((rad - rad_min) / total_rad_range) * total_us_range
    
    # clamp values for safety
    clamped_us = max(us_min, min(us_max, int(us_value)))
    
    return clamped_us


# --- GUI ------------------------------------------------------------------------------------------
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


def on_slider_move(_=None):
    """
    Called on slider interaction and only updates hardware if set to 'IK Target' mode.
    """
    x_val = float(slider_x.get())
    y_val = float(slider_y.get())
    z_val = float(slider_z.get())
    phi_val = float(slider_phi.get())
    
    # update values regardless of mode
    lbl_x_val.config(text=f"{x_val:.1f}")
    lbl_y_val.config(text=f"{y_val:.1f}")
    lbl_z_val.config(text=f"{z_val:.1f}")
    lbl_phi_val.config(text=f"{phi_val:.1f}°")
    
    # check if in IK mode
    if current_mode.get() == "IK":
        # define robot and run ik solver
        robot_desc = {
            "h": 0.086,
            "L2x": 0.1,
            "L2y": 0.032,
            "L3": 0.09,
            "L4": 0.062
        }
        target_3d = np.array([x_val/1000, y_val/1000, z_val/1000])
        phi = np.radians(phi_val)
        
        joint_ik = ik_solver(robot_desc, target_3d, phi)
        joint_angles = ik_to_robot(joint_ik)

        target_positions = [
            radians_to_us(joint_angles[0]), 
            radians_to_us(joint_angles[1]), 
            radians_to_us(joint_angles[2]), 
            radians_to_us(joint_angles[3]), 
            1500
        ]

        joint_pulses = target_positions
        send_pulse_widths(joint_pulses)


def toggle_control_mode():
    """
    Switches controller between rest position mode and ik targeting mode.
    """
    if current_mode.get() == "REST":
        # change to rest mode
        btn_toggle.config(text="Mode: REST POSITION", style="Rest.TButton")
        # disable sliders when in rest mode
        for s in [slider_x, slider_y, slider_z, slider_phi]:
            s.config(state="disabled")
        reset_to_rest_pose()
    else:
        # change to IK mode
        btn_toggle.config(text="Mode: IK TARGETING", style="IK.TButton")
        # enable sliders
        for s in [slider_x, slider_y, slider_z, slider_phi]:
            s.config(state="normal")
        # call data update when sliders are moved
        on_slider_move()


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Morris 5-DOF Hybrid Controller")
    root.geometry("512x512")

    # styling for toggle button
    style = ttk.Style()
    style.configure("Rest.TButton", font=("Arial", 11, "bold"), foreground="red")
    style.configure("IK.TButton", font=("Arial", 11, "bold"), foreground="green")

    ttk.Label(root, text="Morris 5-DOF Cartesian Controller", font=("Arial", 14, "bold")).pack(pady=15)

    # mode selector frame
    mode_frame = ttk.Frame(root)
    mode_frame.pack(fill='x', padx=25, pady=5)
    
    current_mode = tk.StringVar(value="REST")
    
    # toggle button
    btn_toggle = ttk.Checkbutton(
        mode_frame, 
        text="Mode: REST POSITION", 
        variable=current_mode, 
        onvalue="IK", 
        offvalue="REST", 
        style="Rest.TButton",
        command=toggle_control_mode
    )
    btn_toggle.pack(fill='x', ipady=8)

    ttk.Separator(root, orient='horizontal').pack(fill='x', padx=25, pady=15)

    # target coordinates frame
    slider_frame = ttk.Frame(root)
    slider_frame.pack(fill='both', expand=True, padx=25)

    def create_cartesian_slider(parent, label_text, min_val, max_val, default_val, row_idx):
        lbl = ttk.Label(parent, text=label_text, width=15, anchor="w")
        lbl.grid(row=row_idx, column=0, sticky="w", pady=10)
        
        slider = ttk.Scale(parent, from_=min_val, to=max_val, orient='horizontal', command=on_slider_move)
        slider.set(default_val)
        slider.grid(row=row_idx, column=1, sticky="ew", padx=10, pady=10)
        
        val_lbl = ttk.Label(parent, text="", width=8, anchor="e")
        val_lbl.grid(row=row_idx, column=2, sticky="e", pady=10)
        
        parent.grid_columnconfigure(1, weight=1)
        return slider, val_lbl

    # create sliders
    slider_x,   lbl_x_val   = create_cartesian_slider(slider_frame, "Target X (mm):", -150,  150,  0.0,   0)
    slider_y,   lbl_y_val   = create_cartesian_slider(slider_frame, "Target Y (mm):", -150, -50,  -100.0, 1)
    slider_z,   lbl_z_val   = create_cartesian_slider(slider_frame, "Target Z (mm):",  10,   300,  150,  2)
    slider_phi, lbl_phi_val = create_cartesian_slider(slider_frame, "Orientation φ (°):", -90, 90, 0.0,   3)

    # initialise in rest mode by default
    toggle_control_mode()

    root.mainloop()

    if ser:
        reset_to_rest_pose()
        ser.close()