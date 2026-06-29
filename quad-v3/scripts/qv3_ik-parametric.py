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
        # print(f"Sent: {msg.strip()}")
    except Exception as e:
        print(f"Failed to send data: {e}")


# --- IK SOLVER ------------------------------------------------------------------------------------
def reset_to_rest_pose():
    """
        Defines and sets the robot to its resting, or zero, position.
    """

    rest_poses = [
        1000, 1764, 1735,

        2000, 1235, 1264,

        2000, 1235, 1264,

        1000, 1764, 1735,

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

    # joint angles with actuator offsets
    theta_1j = np.pi/2 + theta_1
    theta_2j = np.pi/2 - theta_2
    theta_3j = np.pi + theta_3

    return [theta_1j, theta_2j, theta_3j]


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
    # send joint angles to robot
    print("\nSENDING JOINT ANGLE DATA ------------------------------------------")
    try:
        # initialize to zero position on startup
        reset_to_rest_pose()
        time.sleep(1)

        # trajectory config
        start_time = time.time()
        duration = 10.0 # Run the parametric loop for 10 seconds
        dt = 0.02        # Time step between updates (50 Hz refresh rate)

        while True:
            t = time.time() - start_time
            if t > duration:
                break  # Exit loop after the specified duration

            # run ik solver
            frequency = 5 # must be an odd number

            w_spread = 0.1
            w_length = 0.06
            w_floor = 0.12
            w_gait_width = 0.04
            w_gait_rise = 0.04

            def leg_walk(direction, side, offset):
                w_offset = offset * frequency
                w_x = w_spread
                w_y = direction * w_gait_width*np.cos(t * frequency + w_offset) - w_length
                w_z = w_gait_rise*np.maximum(0, np.sin(t * frequency + w_offset)) - w_floor
                w_ik = leg_ik_solver(np.array([w_x, side * w_y, w_z]))
                return w_ik

            # FORWARDS
            fl_ik = leg_walk(1, 1, 0)
            fr_ik = leg_walk(1, -1, np.pi)
            bl_ik = leg_walk(-1, -1, np.pi)
            br_ik = leg_walk(-1, 1, 0)

            # BACKWARDS
            # fl_ik = leg_walk(-1, 1, 0)
            # fr_ik = leg_walk(-1, -1, np.pi)
            # bl_ik = leg_walk(1, -1, np.pi)
            # br_ik = leg_walk(1, 1, 0)

            # TURN LEFT
            # fl_ik = leg_walk(-1, 1, 0)
            # fr_ik = leg_walk(1, -1, np.pi)
            # bl_ik = leg_walk(1, -1, np.pi)
            # br_ik = leg_walk(-1, 1, 0)

            # TURN RIGHT
            # fl_ik = leg_walk(1, 1, 0)
            # fr_ik = leg_walk(-1, -1, np.pi)
            # bl_ik = leg_walk(-1, -1, np.pi)
            # br_ik = leg_walk(1, 1, 0)

            # DANCE
            # fl_ik = leg_walk(1, 1, 0)
            # fr_ik = leg_walk(1, -1, np.pi)
            # bl_ik = leg_walk(1, -1, np.pi)
            # br_ik = leg_walk(1, 1, 0)

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

            # print(target_positions)
            
            # send target position data
            send_pulse_widths(target_positions)

            # control loop frequency
            time.sleep(dt)
        
        # return to zero position before exiting
        print("\nTRAJECTORY COMPLETE. RETURNING TO REST...")
        reset_to_rest_pose()
        time.sleep(3)

    finally:
        # close the serial port before exiting
        if ser:
            ser.close()
            print("Serial connection closed.")