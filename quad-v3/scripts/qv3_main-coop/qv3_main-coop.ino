// DESCRIPTION ----------------------------------------------------------------------------------------------------------------
// This is the main hardware script for the Steven robot
// It has four functions:
// 1. Servo control (IK + Gait Generator + Interpolation)
// 2. Sensor control & data processing
// 3. Matrix display control
// 4. Buzzer control


// INCLUDES -------------------------------------------------------------------------------------------------------------------
#include <math.h>
// MAX7219 8x8 Matrix Displays
#include <MD_MAX72xx.h>
#include <SPI.h>
// PCA9685 Servo Driver
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// GLOBALS & FUNCTIONS --------------------------------------------------------------------------------------------------------
enum DisplaySide { D_LEFT = 0, D_RIGHT = 1 }; // define display side enum at top to prevent compiler error

struct Point3D {
  float x;
  float y;
  float z;

  // Constructors
  Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
  Point3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

  // Basic Vector Arithmetic
  Point3D operator+(const Point3D& b) const { return Point3D(x + b.x, y + b.y, z + b.z); }
  Point3D operator-(const Point3D& b) const { return Point3D(x - b.x, y - b.y, z - b.z); }
  Point3D operator*(float scalar) const { return Point3D(x * scalar, y * scalar, z * scalar); }
  Point3D operator/(float scalar) const { return Point3D(x / scalar, y / scalar, z / scalar); }

  // Common IK Helper Functions
  float magnitude() const { return sqrt(x * x + y * y + z * z); }

  float distanceTo(const Point3D& p) const {
    float dx = x - p.x;
    float dy = y - p.y;
    float dz = z - p.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }
};

struct Point2D {
  float x;
  float y;

  // Constructors
  Point2D() : x(0.0f), y(0.0f) {}
  Point2D(float _x, float _y) : x(_x), y(_y) {}

  // Basic Vector Arithmetic
  Point2D operator+(const Point2D& b) const { return Point2D(x + b.x, y + b.y); }
  Point2D operator-(const Point2D& b) const { return Point2D(x - b.x, y - b.y); }
  Point2D operator*(float scalar) const { return Point2D(x * scalar, y * scalar); }
  Point2D operator/(float scalar) const { return Point2D(x / scalar, y / scalar); }

  float distanceTo(const Point2D& p) const {
    float dx = x - p.x;
    float dy = y - p.y;
    return sqrt(dx * dx + dy * dy);
  }

  float magnitude() const { return sqrt(x * x + y * y); }

  float distanceToSq(const Point2D& p) const {
    float dx = x - p.x;
    float dy = y - p.y;
    return (dx * dx + dy * dy);
  }

  float magnitudeSq() const { return (x * x + y * y); }
  float angle() const { return atan2(y, x); }
  float angleTo(const Point2D& p) const { return atan2(p.y - y, p.x - x); }
};

// COOPERATIVE SCHEDULER ----------------------------------------------
typedef void (*TaskFunction)();

struct Task {
  TaskFunction run;           // function to execute
  unsigned long interval;     // how often to run (in milliseconds)
  unsigned long lastRun;      // timestamp of last execution
};

// Predefine task functions
void taskServos();
void taskSensors();
void taskDisplays();
void taskBuzzer();

Task tasks[] = {
  {taskServos,    20,   0},
  {taskSensors,   300,  0},
  {taskDisplays,  300,  0},
  {taskBuzzer,    500,  0}
};

const int tasksCount = sizeof(tasks) / sizeof(Task);


// SERVOS & GAIT ENGINE -----------------------------------------------
#define SERVO_FREQ 50 // Standard 50Hz update rate for analog/digital servos

// Pulse width bounds in microseconds (us)
#define USMIN_MG996R 500
#define USMAX_MG996R 2500

#define USMIN_MG90S  584
#define USMAX_MG90S  2450

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const int NUM_SERVOS = 14;

// Microsecond-based rest pose array
const uint16_t robotRestPose[NUM_SERVOS] = {
  1000, 1764, 1735,  // Front Left  (Channels 0, 1, 2)
  2000, 1235, 1264,  // Front Right (Channels 3, 4, 5)
  2000, 1235, 1264,  // Back Left   (Channels 6, 7, 8)
  1000, 1764, 1735,  // Back Right  (Channels 9, 10, 11)
  USMIN_MG90S, USMAX_MG90S  // Antennas (Channels 12, 13)
};

// Tracking positions in microseconds for smooth non-blocking interpolation
float currentUs[NUM_SERVOS];
float targetUs[NUM_SERVOS];

// Speed limit per 20ms tick (~25us per tick = smooth move)
const float MAX_US_PER_TICK = 25.0;

// GAIT MODES
enum GaitMode {
  GAIT_STOP = 0,
  GAIT_FORWARD,
  GAIT_BACKWARD,
  GAIT_TURN_LEFT,
  GAIT_TURN_RIGHT,
  GAIT_DANCE
};

GaitMode currentGaitMode = GAIT_STOP; // Robot starts by walking forward

// Gait parameters from Python
const float FREQUENCY    = 5.0f;   // Step frequency
const float W_SPREAD     = 0.10f;  // X spread (m)
const float W_LENGTH     = 0.06f;  // Y center offset (m)
const float W_FLOOR      = 0.12f;  // Z depth / floor (m)
const float W_GAIT_WIDTH = 0.04f;  // Step stride (m)
const float W_GAIT_RISE  = 0.04f;  // Lift height (m)

struct LegAngles {
  float theta1;
  float theta2;
  float theta3;
};

// 3-DOF Analytical Inverse Kinematics Solver
LegAngles legIKSolver(Point3D target3D) {
  const float L1  = 0.065f;
  const float L2  = 0.060f;
  const float L3x = 0.015f;
  const float L3y = 0.145f;

  const float L3       = sqrt(L3x * L3x + L3y * L3y);
  const float L3_theta = asin(L3x / L3);

  float xyLength = sqrt(target3D.x * target3D.x + target3D.y * target3D.y);
  Point2D target2D(xyLength - L1, target3D.z);

  float r2 = target2D.x * target2D.x + target2D.y * target2D.y;
  float r  = sqrt(r2);
  if (r < 0.0001f) r = 0.0001f; // Prevent zero division

  float gamma = atan2(target2D.y, target2D.x);
  
  float cosBeta  = (L2 * L2 + L3 * L3 - r2) / (2.0f * L2 * L3);
  float cosAlpha = (r2 + L2 * L2 - L3 * L3) / (2.0f * L2 * r);
  
  float beta  = acos(constrain(cosBeta, -1.0f, 1.0f));
  float alpha = acos(constrain(cosAlpha, -1.0f, 1.0f));

  float theta_1 = atan2(target3D.y, target3D.x);
  float theta_2 = gamma + alpha;
  float theta_3 = beta - PI - L3_theta;

  LegAngles angles;
  angles.theta1 = (PI / 2.0f) + theta_1;
  angles.theta2 = (PI / 2.0f) - theta_2;
  angles.theta3 = PI + theta_3;

  return angles;
}

// Single Leg Trajectory Calculator
LegAngles legWalk(float direction, float side, float offset, float tSec) {
  float wOffset = offset * FREQUENCY;
  float phase   = (tSec * FREQUENCY) + wOffset;

  float wX = W_SPREAD;
  float wY = (direction * W_GAIT_WIDTH * cosf(phase)) - W_LENGTH;
  float wZ = (W_GAIT_RISE * fmaxf(0.0f, sinf(phase))) - W_FLOOR;

  Point3D footTarget(wX, side * wY, wZ);
  return legIKSolver(footTarget);
}

// Helper: Convert radians to microsecond pulses
uint16_t radiansToUs(float rad, uint16_t usMin = USMIN_MG996R, uint16_t usMax = USMAX_MG996R) {
  float safeRad = constrain(rad, 0.0f, PI);
  return (uint16_t)(usMin + (safeRad / PI) * (usMax - usMin));
}

// Update Leg Microsecond Targets Based on Active Gait
void updateGait(GaitMode mode) {
  if (mode == GAIT_STOP) return;

  float tSec = millis() / 1000.0f;
  LegAngles fl_ik, fr_ik, bl_ik, br_ik;

  switch (mode) {
    case GAIT_FORWARD:
      fl_ik = legWalk( 1.0f,  1.0f, 0.0f, tSec);
      fr_ik = legWalk( 1.0f, -1.0f,   PI, tSec);
      bl_ik = legWalk(-1.0f, -1.0f,   PI, tSec);
      br_ik = legWalk(-1.0f,  1.0f, 0.0f, tSec);
      break;

    case GAIT_BACKWARD:
      fl_ik = legWalk(-1.0f,  1.0f, 0.0f, tSec);
      fr_ik = legWalk(-1.0f, -1.0f,   PI, tSec);
      bl_ik = legWalk( 1.0f, -1.0f,   PI, tSec);
      br_ik = legWalk( 1.0f,  1.0f, 0.0f, tSec);
      break;

    case GAIT_TURN_LEFT:
      fl_ik = legWalk(-1.0f,  1.0f, 0.0f, tSec);
      fr_ik = legWalk( 1.0f, -1.0f,   PI, tSec);
      bl_ik = legWalk( 1.0f, -1.0f,   PI, tSec);
      br_ik = legWalk(-1.0f,  1.0f, 0.0f, tSec);
      break;

    case GAIT_TURN_RIGHT:
      fl_ik = legWalk( 1.0f,  1.0f, 0.0f, tSec);
      fr_ik = legWalk(-1.0f, -1.0f,   PI, tSec);
      bl_ik = legWalk(-1.0f, -1.0f,   PI, tSec);
      br_ik = legWalk( 1.0f,  1.0f, 0.0f, tSec);
      break;

    case GAIT_DANCE:
      fl_ik = legWalk( 1.0f,  1.0f, 0.0f, tSec);
      fr_ik = legWalk( 1.0f, -1.0f,   PI, tSec);
      bl_ik = legWalk( 1.0f, -1.0f,   PI, tSec);
      br_ik = legWalk( 1.0f,  1.0f, 0.0f, tSec);
      break;

    default:
      return;
  }

  // Front Left (Channels 0, 1, 2)
  targetUs[0] = radiansToUs(fl_ik.theta1);
  targetUs[1] = radiansToUs(PI - fl_ik.theta2);
  targetUs[2] = radiansToUs(PI - fl_ik.theta3);

  // Front Right (Channels 3, 4, 5)
  targetUs[3] = radiansToUs(fr_ik.theta1);
  targetUs[4] = radiansToUs(fr_ik.theta2);
  targetUs[5] = radiansToUs(fr_ik.theta3);

  // Back Left (Channels 6, 7, 8)
  targetUs[6] = radiansToUs(bl_ik.theta1);
  targetUs[7] = radiansToUs(bl_ik.theta2);
  targetUs[8] = radiansToUs(bl_ik.theta3);

  // Back Right (Channels 9, 10, 11)
  targetUs[9]  = radiansToUs(br_ik.theta1);
  targetUs[10] = radiansToUs(PI - br_ik.theta2);
  targetUs[11] = radiansToUs(PI - br_ik.theta3);

  // Antennas (Channels 12, 13)
  targetUs[12] = USMIN_MG90S; // 584 us
  targetUs[13] = USMAX_MG90S; // 2450 us
}

// Helper to constrain microseconds based on motor model
uint16_t getSafeMicroseconds(uint8_t servoIndex, float us) {
  if (servoIndex >= 12) {
    return (uint16_t)constrain(us, USMIN_MG90S, USMAX_MG90S);
  }
  return (uint16_t)constrain(us, USMIN_MG996R, USMAX_MG996R);
}

// Command a single servo immediately without interpolation
void setServoPulseImmediate(uint8_t servoNum, uint16_t us) {
  uint16_t safeUs = getSafeMicroseconds(servoNum, us);
  currentUs[servoNum] = safeUs;
  targetUs[servoNum]  = safeUs;
  pwm.writeMicroseconds(servoNum, safeUs);
}

// Command an entire pose instantly (e.g. at boot)
void loadPoseImmediate(const uint16_t pose[]) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoPulseImmediate(i, pose[i]);
  }
}

// Non-blocking Servo Task (Runs every 20ms)
void taskServos() {
  // 1. Calculate trajectory targets based on current gait mode
  updateGait(currentGaitMode);

  // 2. Interpolate outputs smoothly towards targets
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (currentUs[i] != targetUs[i]) {
      float diff = targetUs[i] - currentUs[i];
      
      if (abs(diff) <= MAX_US_PER_TICK) {
        currentUs[i] = targetUs[i];
      } else {
        currentUs[i] += (diff > 0) ? MAX_US_PER_TICK : -MAX_US_PER_TICK;
      }
      
      pwm.writeMicroseconds(i, (uint16_t)currentUs[i]);
    }
  }
}


// SENSORS ------------------------------------------------------------
const int TRIG_PIN = 12;
const int ECHO_PINS[] = {14, 27, 26}; 
const char* SENSOR_LABELS[] = {"Left", "Middle", "Right"};
const int SENSOR_COUNT = 3;

int currentSensorIndex = 0;

void taskSensors() {
  // 1. Trigger a clean pulse on the shared TRIG pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 2. Read the specific echo pin for the CURRENT sensor (~170 cm timeout)
  long duration = pulseIn(ECHO_PINS[currentSensorIndex], HIGH, 10000); 
  float distance = (duration * 0.0343) / 2;
  
  // 3. Print out data
  Serial.print("Sensor ");
  Serial.print(SENSOR_LABELS[currentSensorIndex]); 
  Serial.print(": ");
  
  if (distance == 0) {
    Serial.println("Out of range");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  // 4. Cycle to next sensor index
  currentSensorIndex++;
  if (currentSensorIndex >= SENSOR_COUNT) {
    currentSensorIndex = 0;
  }
}


// MATRIX DISPLAYS ----------------------------------------------------
#define dHARDWARE_TYPE MD_MAX72XX::FC16_HW
const int DISPLAYS_MAX_DEVICES = 2;

const int DISPLAYS_CLK_PIN  = 18;
const int DISPLAYS_DATA_PIN = 23;
const int DISPLAYS_CS_PIN   = 5;

MD_MAX72XX mx = MD_MAX72XX(dHARDWARE_TYPE, DISPLAYS_CS_PIN, DISPLAYS_MAX_DEVICES);

// Patterns
const uint8_t patternLeft[8] = {
  0b00000000, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00111100, 0b00000000
};

const uint8_t patternRight[8] = {
  0b00000000, 0b00011100, 0b00100100, 0b00100100, 0b00011100, 0b00100100, 0b00100100, 0b00000000
};

const uint8_t patternSmiley[8] = {
  0b00111100, 0b01000010, 0b10100101, 0b10000001, 0b10100101, 0b10011001, 0b01000010, 0b00111100
};

const uint8_t patternHeart[8] = {
  0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000, 0b00000000       
};

const uint8_t patternFull[8] = {
  0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111
};

void displayDrawMatrixPattern(enum DisplaySide side, const uint8_t pattern[]) {
  uint8_t deviceIndex = (uint8_t)side;

  mx.control(deviceIndex, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  
  for (uint8_t col = 0; col < 8; col++) {
    mx.setColumn(deviceIndex, col, pattern[col]); 
  }
  
  mx.control(deviceIndex, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void taskDisplays() {
  displayDrawMatrixPattern(D_LEFT, patternFull);
  displayDrawMatrixPattern(D_RIGHT, patternHeart);
}


// BUZZER -------------------------------------------------------------
const int BUZZER_PIN = 25;
unsigned long buzzerTargetTime = 0;
bool buzzerPlaying = false;

void requestBuzzerTone(double frequency, int duration) {
  if (frequency > 0) {
    ledcWriteTone(BUZZER_PIN, frequency);
    buzzerTargetTime = millis() + duration;
    buzzerPlaying = true;
  }
}

void taskBuzzer() {
  if (buzzerPlaying && millis() >= buzzerTargetTime) {
    ledcWriteTone(BUZZER_PIN, 0);
    buzzerPlaying = false;
  }
}


// SETUP -----------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // SERVOS -----------------------------------------------------------
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Instantly send all 14 channels to rest pose on startup
  loadPoseImmediate(robotRestPose);

  // SENSORS ----------------------------------------------------------
  pinMode(TRIG_PIN, OUTPUT);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(ECHO_PINS[i], INPUT);
  }

  // MATRIX DISPLAYS --------------------------------------------------
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2);
  mx.clear();

  // STARTUP TEST
  displayDrawMatrixPattern(D_LEFT, patternLeft);
  delay(500);
  displayDrawMatrixPattern(D_RIGHT, patternRight);
  delay(500);

  mx.clear();
  delay(500);

  // BUZZER -----------------------------------------------------------
  ledcAttach(BUZZER_PIN, 2000, 8);

  delay(500);
}

// MASTER LOOP ----------------------------------------------------------------------------------------------------------------
void loop() {
  unsigned long timeCurrent = millis();

  for (int i = 0; i < tasksCount; i++) {
    if (timeCurrent - tasks[i].lastRun >= tasks[i].interval) {
      tasks[i].run();
      tasks[i].lastRun = timeCurrent;
    }
  }
}