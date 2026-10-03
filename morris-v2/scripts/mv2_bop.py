import serial
import time

# --- CONFIGURATION ---------------------------------------------------
SERIAL_PORT = 'COM3'   # Update for your OS: 'COM3' (Windows) or '/dev/ttyUSB0' / '/dev/ttyACM0' (Linux/Mac)
BAUD_RATE   = 9600

# Joint 1 Configuration (Slower joint: 1 sweep = Min -> Max)
JOINT_1     = 0        # Channel 0
MIN_DEG_1   = 60.0     # Lower degree bound
MAX_DEG_1   = 120.0    # Upper degree bound

# Joint 2 Configuration (2x Faster joint: 1 full cycle = Min -> Max -> Min per Joint 1 sweep)
JOINT_2     = 2        # Channel 2
MIN_DEG_2   = 30.0     # Lower degree bound
MAX_DEG_2   = 40.0     # Upper degree bound

BPM         = 57.5     # Oscillations per minute for Joint 1
# ---------------------------------------------------------------------


def main():
    # Joint 1 sweep duration (half cycle for Joint 1)
    duration_1_ms = int((60.0 / BPM / 2.0) * 1000)
    
    # Joint 2 sweep duration (quarter cycle for Joint 2, i.e., half of Joint 1 duration)
    duration_2_ms = duration_1_ms // 2
    duration_2_sec = duration_2_ms / 1000.0

    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino UNO reset on connection

    print(f"Starting 2:1 harmonic oscillation:")
    print(f"  - Joint {JOINT_1}: {MIN_DEG_1}° <-> {MAX_DEG_1}° ({duration_1_ms} ms per sweep)")
    print(f"  - Joint {JOINT_2}: 2x speed ({duration_2_ms} ms per sweep)")
    print("Press Ctrl+C to exit and return to home.\n")

    target_1 = MAX_DEG_1

    try:
        while True:
            # --- SUB-STEP 1: Joint 1 starts moving to target; Joint 2 moves MIN -> MAX ---
            cmd1 = (
                f"joint {JOINT_1} {target_1:.1f} {duration_1_ms}\n"
                f"joint {JOINT_2} {MAX_DEG_2:.1f} {duration_2_ms}\n"
            )
            ser.write(cmd1.encode('utf-8'))
            time.sleep(duration_2_sec)

            # --- SUB-STEP 2: Joint 1 continues to target; Joint 2 returns MAX -> MIN ---
            cmd2 = f"joint {JOINT_2} {MIN_DEG_2:.1f} {duration_2_ms}\n"
            ser.write(cmd2.encode('utf-8'))
            time.sleep(duration_2_sec)

            # Toggle target for Joint 1 for the next pass
            target_1 = MIN_DEG_1 if target_1 == MAX_DEG_1 else MAX_DEG_1

    except KeyboardInterrupt:
        print("\nStopping oscillation...")
    finally:
        print("Returning robot to home position (1500 ms)...")
        ser.write(b"home 1500\n")
        time.sleep(1.5)  # Pause briefly so bytes transmit fully before closing
        ser.close()
        print("Disconnected cleanly.")

if __name__ == "__main__":
    main()