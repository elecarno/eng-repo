import serial
import time
import random

# --- CONFIGURATION ---------------------------------------------------
SERIAL_PORT = 'COM4' 
BAUD_RATE   = 9600

def main():
    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino UNO reset on connection

    try:
        while True:
            time_between = 20
            move_time = 1500

            ser.write(f"joint 0 90  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 1 45  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 2 30  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 3 90  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 4 115 {move_time}\n".encode('utf-8'))
            ser.write(f"joint 5 90  {move_time}\n".encode('utf-8'))
            time.sleep(time_between)

            ser.write(f"joint 0 45  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 1 60  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 2 30  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 3 30  {move_time}\n".encode('utf-8'))
            ser.write(f"joint 4 130 {move_time}\n".encode('utf-8'))
            ser.write(f"joint 5 135 {move_time}\n".encode('utf-8'))
            time.sleep(time_between)

            ser.write(f"joint 0 90 {move_time}\n".encode('utf-8'))
            ser.write(f"joint 1 60 {move_time}\n".encode('utf-8'))
            ser.write(f"joint 2 80 {move_time}\n".encode('utf-8'))
            ser.write(f"joint 3 90 {move_time}\n".encode('utf-8'))
            ser.write(f"joint 4 60 {move_time}\n".encode('utf-8'))
            ser.write(f"joint 5 90 {move_time}\n".encode('utf-8'))
            time.sleep(time_between)


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