import serial
import time
import random

# --- CONFIGURATION ---------------------------------------------------
SERIAL_PORT = 'COM3' 
BAUD_RATE   = 9600

def main():
    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino UNO reset on connection

    try:
        while True:
            # joint | channel | angle in degrees (0 to 180) | time to move in milliseconds |
            ser.write(f"joint 2 {random.randint(0, 180)} 5000\n".encode('utf-8'))
            ser.write(f"joint 4 90 5000\n".encode('utf-8'))
            time.sleep(500)

    except KeyboardInterrupt:
        print("\nStopping oscillation...")
    finally:
        print("Returning robot to home position (1500 ms)...")
        ser.write(b"home 1500\n")
        time.sleep(1.5)
        ser.close()
        print("Disconnected cleanly.")

if __name__ == "__main__":
    main()