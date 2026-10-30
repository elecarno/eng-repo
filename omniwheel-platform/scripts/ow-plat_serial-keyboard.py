import time
import keyboard
import serial

# Configure your serial port (change 'COM3' to your Arduino's port, e.g., '/dev/ttyUSB0' on Linux)
PORT = "COM5"
BAUD = 115200

print(f"Connecting to Arduino on {PORT}...")
try:
  ser = serial.Serial(PORT, BAUD, timeout=1)
  time.sleep(2)  # Allow Arduino reset time
  print("Connected successfully!")
except Exception as e:
  print(f"Connection failed: {e}")
  exit()

# Base speed setting (RPM)
BASE_RPM = 40.0


def send_rpms(rx, ry, rz):
  command = f"{rx},{ry},{rz}\n"
  ser.write(command.encode("utf-8"))


print("\n--- Kiwi Drive Keyboard Control ---")
print(" [W] : Move Forward")
print(" [S] : Move Backward")
print(" [A] : Strafe Left")
print(" [D] : Strafe Right")
print(" [Q] : Rotate Counter-Clockwise")
print(" [E] : Rotate Clockwise")
print(" [X] : Stop / Idle")
print(" Press [Esc] to exit.\n")

try:
  while True:
    if keyboard.is_pressed("esc"):
      print("Exiting...")
      send_rpms(0, 0, 0)
      break

    # Determine RPMs based on pressed keys
    # Note: Depending on your omniwheel physical orientation,
    # you may need to invert some signs (+/-) if a wheel spins the wrong way.
    rx, ry, rz = 0.0, 0.0, 0.0

    if keyboard.is_pressed("w"):
      ry = -BASE_RPM
      rx = BASE_RPM
    elif keyboard.is_pressed("s"):
      ry = BASE_RPM
      rx = -BASE_RPM

    if keyboard.is_pressed("a"):
      # Adjust strafe vectors based on your 3-wheel geometry
      rx -= BASE_RPM
      ry -= BASE_RPM
      rz += BASE_RPM
    elif keyboard.is_pressed("d"):
      rx += BASE_RPM
      ry += BASE_RPM
      rz -= BASE_RPM

    if keyboard.is_pressed("q"):
      rx = BASE_RPM
      ry = BASE_RPM
      rz = BASE_RPM
    elif keyboard.is_pressed("e"):
      rx = -BASE_RPM
      ry = -BASE_RPM
      rz = -BASE_RPM

    if keyboard.is_pressed("x"):
      rx, ry, rz = 0.0, 0.0, 0.0

    # Send command to Arduino
    send_rpms(rx, ry, rz)

    # Small delay to prevent flooding the serial buffer (~20Hz update rate)
    time.sleep(0.05)

except KeyboardInterrupt:
  send_rpms(0, 0, 0)
  ser.close()
  print("Program closed safely.")