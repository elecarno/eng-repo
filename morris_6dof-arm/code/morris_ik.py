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


# --- CONTROL --------------------------------------------------------------------------------------
def send_pulse_widths(pulse_widths):
    """
    Packages and transmits raw microsecond values to PCA9685
    Takes in an array of 5 integer values between 500 and 2500
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
    # [J1/base, J2/shoulder, J3/elbow, J4/wrist, J5/cuff]
    rest_poses = [1500, 500, 500, 1500, 1500]
    print("Resetting to rest position...")
    send_pulse_widths(rest_poses)


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
    try:
        # initialize to zero position on startup
        reset_to_rest_pose()
        time.sleep(1) 

        # set to target positions
        custom_positions = [1500, 1000, 1200, 1500, 1800]
        
        send_pulse_widths(custom_positions)
        time.sleep(2) # give hardware time to move

        # return to zero position before exiting
        reset_to_rest_pose()

    finally:
        # close the serial port cleanly when the script finishes
        if ser:
            ser.close()
            print("Serial connection closed.")