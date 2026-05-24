import tkinter as tk
from tkinter import ttk
import serial
import time

# --- SERIAL SETUP ---
try:
    ser = serial.Serial('COM4', 115200, timeout=1)
    time.sleep(2)  # Wait for ESP32 serial pipeline to stabilize
    print("Connected to PCA9685/ESP32 Controller!")
except Exception as e:
    print(f"Connection Error: {e}")
    ser = None

def send_pulse_widths(*args):
    """Packages and transmits the raw microsecond values to the PCA9685."""
    # Adjusted to ensure exactly 5 sliders exist before sending data
    if ser and ser.is_open and len(sliders) == 5:
        try:
            # Generates format: "C0:1500,C1:2500,C2:500,C3:1500,C4:1500\n"
            msg = ",".join([f"C{i}:{int(slider.get())}" for i, slider in enumerate(sliders)]) + "\n"
            ser.write(msg.encode('utf-8'))
        except NameError:
            pass  # Protection during initialization

# --- GUI WINDOW SETUP ---
root = tk.Tk()
root.title("5-Axis Microsecond Arm Controller")
root.geometry("450x480")  # Made the window slightly shorter since there are 5 sliders

def create_us_slider(label_text, default_us):
    frame = ttk.Frame(root)
    frame.pack(fill='x', padx=20, pady=10)
    
    label = ttk.Label(frame, text=label_text, width=22)
    label.pack(side='left')
    
    slider = ttk.Scale(frame, from_=500, to=2500, orient='horizontal', command=send_pulse_widths)
    slider.set(default_us)
    slider.pack(side='right', expand=True, fill='x')
    return slider

ttk.Label(root, text="PCA9685 Servo Controller (μs)", font=("Arial", 14, "bold")).pack(pady=15)

# Array to hold slider references
sliders = []

# --- CONFIGURATIONS FOR 5 SERVOS ---
configs = [
    ("Channel 0 - Base (J1):", 1500),      # USMID
    ("Channel 1 - Shoulder (J2):", 500),   # USMIN
    ("Channel 2 - Elbow (J3):", 500),      # USMIN
    ("Channel 3 - Wrist (J4):", 1500),     # USMID
    ("Channel 4 - Cuff (J5):", 1500)       # USMID
]

# Build the 5 sliders
for label_text, default_val in configs:
    sliders.append(create_us_slider(label_text, default_val))

# Safety Home function for 5 servos
def reset_to_rest_pose():
    rest_poses = [1500, 500, 500, 1500, 1500]
    for slider, pose in zip(sliders, rest_poses):
        slider.set(pose)
    send_pulse_widths()

home_btn = ttk.Button(root, text="Return to Rest Pose", command=reset_to_rest_pose)
home_btn.pack(pady=25)

# Initial push to ensure hardware syncs with GUI on startup
send_pulse_widths()

root.mainloop()

if ser:
    ser.close()