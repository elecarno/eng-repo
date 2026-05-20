import tkinter as tk
from tkinter import ttk
import serial
import time

# --- SERIAL SETUP ---
# REMEMBER: Change 'COM3' to match your actual ESP32 port!
try:
    ser = serial.Serial('COM4', 115200, timeout=1)
    time.sleep(2)  # Wait for ESP32 serial pipeline to stabilize
    print("Connected to PCA9685/ESP32 Controller!")
except Exception as e:
    print(f"Connection Error: {e}")
    ser = None

def send_pulse_widths(*args):
    """Packages and transmits the raw microsecond values to the PCA9685."""
    if ser and ser.is_open:
        msg = f"C0:{slider0.get()},C1:{slider1.get()},C2:{slider2.get()},C3:{slider3.get()},C4:{slider4.get()},C5:{slider5.get()}\n"
        ser.write(msg.encode('utf-8'))

# --- GUI WINDOW SETUP ---
root = tk.Tk()
root.title("6DoF Microsecond Arm Controller")
root.geometry("450x550")

def create_us_slider(label_text, default_us):
    frame = ttk.Frame(root)
    frame.pack(fill='x', padx=20, pady=10)
    
    label = ttk.Label(frame, text=label_text, width=22)
    label.pack(side='left')
    
    # Range scaled perfectly from 500us (0°) to 2500us (180°)
    slider = ttk.Scale(frame, from_=500, to=2500, orient='horizontal', command=send_pulse_widths)
    slider.set(default_us)
    slider.pack(side='right', expand=True, fill='x')
    return slider

ttk.Label(root, text="PCA9685 Servo Controller (μs)", font=("Arial", 14, "bold")).pack(pady=15)

# Sliders set explicitly to your hardware's exact rest configurations
slider0 = create_us_slider("Channel 0 - Base:", 1500)
slider1 = create_us_slider("Channel 1 - Shoulder:", 500)
slider2 = create_us_slider("Channel 2 - Elbow 1:", 500)
slider3 = create_us_slider("Channel 3 - Elbow 2:", 2500)
slider4 = create_us_slider("Channel 4 - Wrist 1:", 1500)
slider5 = create_us_slider("Channel 5 - Wrist 2:", 1500)

# Safety Home function returning everything instantly to your custom rest pose
def reset_to_rest_pose():
    slider0.set(1500)
    slider1.set(500)
    slider2.set(500)
    slider3.set(2500)
    slider4.set(1500)
    slider5.set(1500)
    send_pulse_widths()

home_btn = ttk.Button(root, text="Return to Rest Pose", command=reset_to_rest_pose)
home_btn.pack(pady=25)

root.mainloop()

if ser:
    ser.close()