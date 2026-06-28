# STEVEN ROBOT DIRECT CONTROL INTERFACE
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
    time.sleep(2)  # wait for ESP32 serial pipeline to stabilize
    print("Connected to PCA9685/ESP32 Controller!")
except Exception as e:
    print(f"Connection Error: {e}")
    ser = None


def send_pulse_widths(*args):
    """
        Packages and transmits raw microsecond values to the PCA9685.
    """
    if ser and ser.is_open and len(sliders) == 14:
        try:
            # generate format: "C0:1500,C1:2500,C2:500,C3:1500,C4:1500\n"
            msg = ",".join(
                [f"C{i}:{int(slider.get())}" for i, slider in enumerate(sliders)]
            ) + "\n"
            ser.write(msg.encode('utf-8'))
        except NameError:
            pass  # protection during initialization

# --- GUI ------------------------------------------------------------------------------------------
def create_us_slider(label_text, default_us):
    frame = ttk.Frame(root)
    frame.pack(fill='x', padx=20, pady=5)
    
    label = ttk.Label(frame, text=label_text, width=18)
    label.pack(side='left')
    
    val_label = ttk.Label(frame, text=f"{default_us} µs", width=8, anchor='e')
    val_label.pack(side='right', padx=(10, 0))
    
    def on_slider_move(val):
        val_label.config(text=f"{int(float(val))} µs")
        send_pulse_widths()

    # 3. Middle: The Slider itself
    slider = ttk.Scale(frame, from_=500, to=2500, orient='horizontal', command=on_slider_move)
    slider.set(default_us)
    slider.pack(side='right', expand=True, fill='x')
    
    return slider


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Q-v3 Servo Controller")
    root.geometry("450x720")

    ttk.Label(root, text="Q-v3 Servo Controller", font=("Arial", 14, "bold")).pack(pady=15)

    # list to hold slider references
    sliders = []

    rest_poses = [
        500, 2500, 2500,

        2500, 500, 500,

        2500, 500, 500,

        500, 2500, 2500,

        600, 
        2400
    ]

    configs = [
        ("C0  FL Hip:  ", rest_poses[0 ]),
        ("C1  FL Knee: ", rest_poses[1 ]),
        ("C2  FL Ankle:", rest_poses[2 ]),
         
        ("C3  FR Hip:  ", rest_poses[3 ]),
        ("C4  FR Knee: ", rest_poses[4 ]),
        ("C5  FR Ankle:", rest_poses[5 ]),
        
        ("C6  BL Hip:  ", rest_poses[6 ]),
        ("C7  BL Knee: ", rest_poses[7 ]),
        ("C8  BL Ankle:", rest_poses[8 ]),

        ("C9  BR Hip:  ", rest_poses[9 ]),
        ("C10 BR Knee: ", rest_poses[10]),
        ("C11 BR Ankle:", rest_poses[11]),

        ("C12 L Antenna:", rest_poses[12]),
        ("C13 R Antenna:", rest_poses[13]),
    ]

    # create sliders
    for label_text, default_val in configs:
        sliders.append(create_us_slider(label_text, default_val))

    # rest function for 12 servos
    def reset_to_rest_pose():
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