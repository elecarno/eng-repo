// --- LIBRARIES ------------------------------------------------------
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ramp.h>

// --- GLOBALS --------------------------------------------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // 50Hz servo refresh rate

// servo Limits
#define usMS24_MIN 500
#define usMS24_MAX 2500
#define degMS24_RANGE 270

#define usMG996R_MIN 500
#define usMG996R_MAX 2400
#define degMG996R_RANGE 180

// motor to joint mapping (0 = MS24, 1 = MG996R)
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
  MOTOR_MS24, MOTOR_MS24,                                 // shoulder      (0, 1)
  MOTOR_MG996R, MOTOR_MG996R, MOTOR_MG996R, MOTOR_MG996R, // elbow & wrist (2, 3, 4, 5)
  MOTOR_MG996R                                            // end-effector  (6)
};

const float HOME_POSITIONS[NUM_SERVOS] = {
  90.0f, // base
  0.0f,  // shoulder
  0.0f,  // elbow
  90.0f, // wrist roll 1
  90.0f, // wrist pitch
  90.0f, // wrist roll 2
  0.0f   // end-effector
};

// ramp objects for all 7 servos
rampInt servoRamps[NUM_SERVOS];

// serial buffer Variables
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

  // get motor profile based on channel type
  int motorType = motorTypes[channel];
  ServoProfile profile = SERVO_PROFILES[motorType];

  // conversion: degrees -> us -> 12-bit PCA9685 ticks
  uint16_t targetUs    = degreesToUs(targetDegrees, profile.usMin, profile.usMax, profile.maxRangeDeg);
  uint16_t targetTicks = usToTicks(targetUs);

  // start movement interpolating over moveDurationMs
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
  command.trim(); // remove leading/trailing spaces or newlines

  if (command.startsWith("joint ")) {
    // remove "joint " prefix
    String params = command.substring(6);

    // extract channel
    int firstSpace = params.indexOf(' ');
    if (firstSpace == -1) return;
    uint8_t channel = params.substring(0, firstSpace).toInt();

    // extract target degrees
    params = params.substring(firstSpace + 1);
    int secondSpace = params.indexOf(' ');
    if (secondSpace == -1) return;
    float targetDegrees = params.substring(0, secondSpace).toFloat();

    // extract move duration in milliseconds
    unsigned long durationMs = params.substring(secondSpace + 1).toInt();

    // execute movement
    moveServoToAngle(channel, targetDegrees, durationMs);

  } else if (command.startsWith("home")) {
    // default duration if no parameter is provided
    unsigned long durationMs = 1000; 

    // check if a time parameter was passed
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      unsigned long parsedTime = command.substring(spaceIndex + 1).toInt();
      if (parsedTime > 0) durationMs = parsedTime;
    }

    // move all joints to home positions
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      moveServoToAngle(i, HOME_POSITIONS[i], durationMs);
    }

  } else if (command.startsWith("stance")) {
    // default duration if no parameter is provided
    unsigned long durationMs = 1000;

    // check if a time parameter was passed
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      unsigned long parsedTime = command.substring(spaceIndex + 1).toInt();
      if (parsedTime > 0) durationMs = parsedTime;
    }

    // move all joints to stance preset angles
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
        inputBuffer = ""; // reset buffer after processing
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

  // set home position for each channel based on HOME_POSITIONS array
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    ServoProfile p = SERVO_PROFILES[motorTypes[i]];
    uint16_t homeTicks = usToTicks(degreesToUs(HOME_POSITIONS[i], p.usMin, p.usMax, p.maxRangeDeg));
    
    servoRamps[i].go(homeTicks); // set internal ramp starting point
    pwm.setPWM(i, 0, homeTicks); // send initial home PWM pulse to PCA9685
  }

  delay(500); // let hardware settle

  // initial move sequence to stance position
  moveServoToAngle(1, 45.0f, 1000);
  moveServoToAngle(2, 30.0f, 1000);
  moveServoToAngle(4, 115.0f, 1000);
}

void loop() {
  checkSerial();   // parse incoming commands without blocking
  updateServos();  // advance ramp values and send PWM outputs
}