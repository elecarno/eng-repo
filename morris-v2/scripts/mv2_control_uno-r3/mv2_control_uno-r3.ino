// --- LIBRARIES ------------------------------------------------------
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ramp.h>

// --- GLOBALS --------------------------------------------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // 50Hz servo refresh rate

// Servo Limits
#define usMS24_MIN 500
#define usMS24_MAX 2500
#define degMS24_RANGE 270

#define usMG996R_MIN 500
#define usMG996R_MAX 2400
#define degMG996R_RANGE 180

// Motor to joint mapping (0 = MS24, 1 = MG996R)
#define MOTOR_MS24 0
#define MOTOR_MG996R 1

struct ServoProfile {
  uint16_t usMin;
  uint16_t usMax;
  float maxRangeDeg;
};

const ServoProfile SERVO_PROFILES[] = {
  {usMS24_MIN, usMS24_MAX, degMS24_RANGE},      // 0: MS24
  {usMG996R_MIN, usMG996R_MAX, degMG996R_RANGE} // 1: MG996R
};

const uint8_t NUM_SERVOS = 7;
int motorTypes[NUM_SERVOS] = {
  MOTOR_MS24, MOTOR_MS24,                                 // shoulder (ch 0, 1)
  MOTOR_MG996R, MOTOR_MG996R, MOTOR_MG996R, MOTOR_MG996R, // elbow & wrist (ch 2, 3, 4, 5)
  MOTOR_MG996R                                            // end-effector (ch 6)
};

// Initial Home Angles in Degrees for Channels 0 to 6
const float HOME_POSITIONS[NUM_SERVOS] = {
  90.0f, // Ch 0
  0.0f,  // Ch 1
  0.0f,  // Ch 2
  90.0f, // Ch 3
  90.0f, // Ch 4
  90.0f, // Ch 5
  0.0f   // Ch 6
};

// ramp objects for all 7 servos
rampInt servoRamps[NUM_SERVOS];

// Serial Buffer Variables
String inputBuffer = "";


// --- FUNCTIONS ------------------------------------------------------
uint16_t usToTicks(uint16_t microseconds) {
  return (uint16_t)(((uint32_t)microseconds * 4096) / 20000);
}

uint16_t degreesToUs(float targetDegrees, uint16_t usMin, uint16_t usMax, float maxRangeDeg) {
  if (targetDegrees < 0.0f) targetDegrees = 0.0f;
  if (targetDegrees > maxRangeDeg) targetDegrees = maxRangeDeg;

  float usValue = (float)usMin + (targetDegrees / maxRangeDeg) * (float)(usMax - usMin);
  return (uint16_t)round(usValue);
}

/**
 * @brief Commands a servo channel to move smoothly to a target angle in degrees.
 */
void moveServoToAngle(uint8_t channel, float targetDegrees, unsigned long moveDurationMs = 1500, ramp_mode easing = SINUSOIDAL_INOUT) {
  if (channel >= NUM_SERVOS) {
    Serial.print("ERROR: Invalid channel ");
    Serial.println(channel);
    return;
  }

  // 1. Fetch motor profile based on channel type
  int motorType = motorTypes[channel];
  ServoProfile profile = SERVO_PROFILES[motorType];

  // 2. Convert degrees -> us -> 12-bit PCA9685 ticks
  uint16_t targetUs    = degreesToUs(targetDegrees, profile.usMin, profile.usMax, profile.maxRangeDeg);
  uint16_t targetTicks = usToTicks(targetUs);

  // 3. Start movement interpolating over moveDurationMs
  servoRamps[channel].go(targetTicks, moveDurationMs, easing, ONCEFORWARD);

  Serial.print("Moving Ch ");
  Serial.print(channel);
  Serial.print(" to ");
  Serial.print(targetDegrees);
  Serial.print(" deg over ");
  Serial.print(moveDurationMs);
  Serial.println(" ms");
}

// non-blocking task function to be called continuously inside loop()
void updateServos() {
  static unsigned long lastUpdate = 0;
  const unsigned long UPDATE_INTERVAL = 1000 / SERVO_FREQ; // 20 ms for 50Hz refresh rate

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
      int currentTick = servoRamps[ch].update();

      if (servoRamps[ch].isRunning()) {
        pwm.setPWM(ch, 0, currentTick);
      }
    }
  }
}

/**
 * Parses and processes serial command string
 * Command format: "joint [channel] [degrees] [duration_ms]"
 */
void processSerialCommand(String command) {
  command.trim(); // Clean leading/trailing spaces or newlines

  if (command.startsWith("joint ")) {
    // Remove "joint " prefix
    String params = command.substring(6);

    // Extract channel
    int firstSpace = params.indexOf(' ');
    if (firstSpace == -1) return;
    uint8_t channel = params.substring(0, firstSpace).toInt();

    // Extract target degrees
    params = params.substring(firstSpace + 1);
    int secondSpace = params.indexOf(' ');
    if (secondSpace == -1) return;
    float targetDegrees = params.substring(0, secondSpace).toFloat();

    // Extract move duration in milliseconds
    unsigned long durationMs = params.substring(secondSpace + 1).toInt();

    // Execute move
    moveServoToAngle(channel, targetDegrees, durationMs);
  } else if (command.startsWith("home")) {
    // Default duration if no parameter is provided
    unsigned long durationMs = 1000; 

    // Check if a time parameter was passed (e.g., "home 2500")
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      unsigned long parsedTime = command.substring(spaceIndex + 1).toInt();
      if (parsedTime > 0) durationMs = parsedTime;
    }

    // Move all joints to home positions
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      moveServoToAngle(i, HOME_POSITIONS[i], durationMs);
    }
  } else if (command.startsWith("stance")) {
    // Default duration if no parameter is provided
    unsigned long durationMs = 1000;

    // Check if a time parameter was passed (e.g., "stance 2000")
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      unsigned long parsedTime = command.substring(spaceIndex + 1).toInt();
      if (parsedTime > 0) durationMs = parsedTime;
    }

    // Move all joints to stance preset angles
    moveServoToAngle(0, 90.0f,  durationMs);
    moveServoToAngle(1, 45.0f,  durationMs);
    moveServoToAngle(2, 30.0f,  durationMs);
    moveServoToAngle(3, 90.0f, durationMs);
    moveServoToAngle(4, 115.0f,  durationMs);
    moveServoToAngle(5, 90.0f,  durationMs);
    moveServoToAngle(6, 0.0f,   durationMs);
  } else if (command.length() > 0) {
    Serial.println("ERROR: Unknown command format or incorrect number of arguments.");
  }
}

/**
 * Non-blocking check for incoming Serial characters
 */
void checkSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processSerialCommand(inputBuffer);
        inputBuffer = ""; // Reset buffer after processing
      }
    } else {
      inputBuffer += c;
    }
  }
}


// --- MAIN -----------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("=============================================");
  Serial.println("   Morris v2 Arduino UNO R3 Control Script   ");
  Serial.println("   Serial format: joint [ch] [deg] [ms]      ");
  Serial.println("=============================================");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Set home position for each channel based on HOME_POSITIONS array
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    ServoProfile p = SERVO_PROFILES[motorTypes[i]];
    uint16_t homeTicks = usToTicks(degreesToUs(HOME_POSITIONS[i], p.usMin, p.usMax, p.maxRangeDeg));
    
    servoRamps[i].go(homeTicks); // Set internal ramp starting point
    pwm.setPWM(i, 0, homeTicks); // Send initial home PWM pulse to PCA9685
  }

  delay(500); // Small pause to let hardware settle

  // Move initial sequence
  moveServoToAngle(1, 45.0f, 1000);
  moveServoToAngle(2, 30.0f, 1000);
  moveServoToAngle(4, 115.0f, 1000);
}

void loop() {
  checkSerial();   // Parse incoming commands without blocking
  updateServos();  // Advance ramp values and send PWM outputs
}