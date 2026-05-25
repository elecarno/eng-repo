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
    # IK VALUES
    h = 0.086

    L2x = 0.1
    L2y = 0.032
    L3  = 0.09
    L4  = 0.062

    phi = np.radians(0)

    # IK SOLVER
    print("RUNNING IK SOVER ----------------------------------------------------")
    target_3d = np.array([-0.05, -0.1, 0.15])

    target_2d_x = np.sqrt( target_3d[0]**2 + target_3d[1]**2 )
    target_2d_y = target_3d[2]
    target_2d = np.array([target_2d_x, target_2d_y])
    target_2d_offset = ([
        target_2d[0] - L4*np.cos(phi),
        target_2d[1] - L4*np.sin(phi) - h
    ])

    print(f"\ntarget (3d): {target_3d}\ntarget (2d): {target_2d}\ntarget offset (2d): {target_2d_offset}")

    theta_0 = np.arctan2(L2y, L2x)
    d = np.sqrt( target_2d_offset[0]**2 + target_2d_offset[1]**2 )
    L2 = np.sqrt( L2x**2 + L2y**2 )

    print(f"\ntheta_0: {theta_0}\nd: {d}\nL2: {L2}")

    # TODO: improve workspace analysis
    target_distance = np.sqrt(target_2d_x**2 + target_2d_y**2)
    max_distance = L2+L3
    if target_distance > max_distance:
        print("\nTARGET OUTSIDE OF WORKSPACE")
    else:
        print("\nTARGET INSIDE OF WORKSPACE")

    theta_1 = np.arctan2(target_3d[1], target_3d[0])
    theta_2 = np.arctan2(target_2d_offset[1], target_2d_offset[0]) + np.arccos( (L2**2 + d**2 - L3**2) / (2*L2*d) ) + theta_0
    theta_3 = np.arccos( (L2**2 + L3**2 - d**2) / (2*L2*L3) ) - theta_0 - np.pi/2
    theta_4 = phi - (theta_2 + theta_3) + np.pi/2

    print(f"\ntheta_1: {theta_1}\ntheta_2: {theta_2}\ntheta_3: {theta_3}\ntheta_4: {theta_4}\n")

    alpha_1 = np.pi + theta_1
    alpha_2 = np.pi - theta_2
    alpha_3 = theta_3 + np.pi/2
    alpha_4 = np.pi/2 - theta_4

    print(f"\nalpha_1: {alpha_1}\nalpha_2: {alpha_2}\nalpha_3: {alpha_3}\nalpha_4: {alpha_4}\n")

    print("\nSENDING IK DATA -----------------------------------------------------")
    # SEND DATA TO IK
    try:
        # initialize to zero position on startup
        reset_to_rest_pose()
        time.sleep(1)

        # set to target positions
        target_positions = [
            radians_to_us(alpha_1), 
            radians_to_us(alpha_2), 
            radians_to_us(alpha_3), 
            radians_to_us(alpha_4), 
            1500
            ]
        
        send_pulse_widths(target_positions)
        time.sleep(2) # give hardware time to move

        # return to zero position before exiting
        reset_to_rest_pose()

    finally:
        # close the serial port cleanly when the script finishes
        if ser:
            ser.close()
            print("Serial connection closed.")