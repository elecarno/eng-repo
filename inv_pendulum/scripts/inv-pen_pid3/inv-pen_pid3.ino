#include <Arduino.h>

// --- Pin Definitions ---
const int ENCODER_PIN = 34; 
const int STEP_PIN    = 25;
const int DIR_PIN     = 26;
const int EN_PIN      = 27;

// --- Physical System Geometry Constants ---
const float ARM_RADIUS_MM      = 70.0;
const float PENDULUM_LENGTH_MM = 70.0;

// --- Calibration Variables ---
float upwardZeroAngle = 0.0; 

// --- Angle Trim Offset ---
// Compensation for physical magnet misalignment or structural offset.
// Added to shift the calculated 12 o'clock setpoint directly (Default: -5.0 deg).
float angleTrimDeg = -5.0; 

// --- Volatile Control Flags & Tuning Parameters ---
bool pidEnabled = true; 

float Kp = 4425;   
float Ki = 925;    
float Kd = 75;    

// --- Acceleration Limits (Jerk Control) ---
const float MAX_ACCELERATION = 25000.0; // Steps / sec^2

// --- Loop Timing Control ---
const unsigned long LOOP_TIME_US = 4000; // 250 Hz control loop
unsigned long lastLoopTime = 0;

// --- PID State Variables ---
float errorSum = 0.0;
float lastError = 0.0;

// --- Non-blocking Stepper Engine Variables ---
unsigned long lastStepTime = 0;
unsigned long stepIntervalUs = 0; 
bool currentDirection = true;
bool motorActive = false;
float currentSpeedStepsPerSec = 0.0;

// --- Serial Command Buffer ---
String inputBuffer = "";

// --- Read Raw Analog Angle with EMA Low-Pass Filter ---
float getRawAngle() {
  static float filteredAngle = -1.0;
  
  uint32_t rawSum = 0;
  for (int i = 0; i < 16; i++) {
    rawSum += analogRead(ENCODER_PIN);
    delayMicroseconds(10);
  }
  float newAngle = ((rawSum / 16.0) / 4095.0) * 360.0;

  if (filteredAngle < 0) {
    filteredAngle = newAngle;
    return filteredAngle;
  }

  const float alpha = 0.25; 
  
  float diff = newAngle - filteredAngle;
  while (diff > 180.0)  diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  
  filteredAngle += alpha * diff;

  while (filteredAngle >= 360.0) filteredAngle -= 360.0;
  while (filteredAngle < 0.0)    filteredAngle += 360.0;

  return filteredAngle;
}

void calibrateZeroPosition() {
  delay(1000); // Allow physical settling at rest
  float sum = 0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    sum += getRawAngle();
    delay(20);
  }
  float downwardZeroAngle = sum / (float)samples;
  
  // Base 180 deg offset plus manual physical trim adjustment
  upwardZeroAngle = downwardZeroAngle + 180.0 + angleTrimDeg;
  
  while (upwardZeroAngle >= 360.0) upwardZeroAngle -= 360.0;
  while (upwardZeroAngle < 0.0)   upwardZeroAngle += 360.0;

  Serial.print("Baseline Zero Recorded. Calculated Upright Setpoint (with Trim): ");
  Serial.println(upwardZeroAngle, 2);
}

// --- Non-blocking Serial Command Parser ---
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer.trim();
      inputBuffer.toUpperCase();

      if (inputBuffer.length() > 0) {
        if (inputBuffer == "START" || inputBuffer == "RUN") {
          pidEnabled = true;
          errorSum = 0;
          lastError = 0;
          Serial.println(">> PID CONTROL ACTIVE");
        } 
        else if (inputBuffer == "STOP") {
          pidEnabled = false;
          motorActive = false;
          currentSpeedStepsPerSec = 0.0;
          stepIntervalUs = 0;
          errorSum = 0;
          Serial.println(">> PID CONTROL PAUSED");
        }
        else if (inputBuffer.length() > 1) {
          char command = inputBuffer.charAt(0);
          float value = inputBuffer.substring(1).toFloat();

          if (command == 'P') {
            Kp = value;
            Serial.print(">> Kp Updated: "); Serial.println(Kp);
          } else if (command == 'I') {
            Ki = value;
            errorSum = 0;
            Serial.print(">> Ki Updated: "); Serial.println(Ki);
          } else if (command == 'D') {
            Kd = value;
            Serial.print(">> Kd Updated: "); Serial.println(Kd);
          } 
          // New Trim Offset Command
          else if (command == 'T') {
            float trimChange = value - angleTrimDeg;
            angleTrimDeg = value;
            upwardZeroAngle += trimChange; // Update active setpoint on the fly
            while (upwardZeroAngle >= 360.0) upwardZeroAngle -= 360.0;
            while (upwardZeroAngle < 0.0)   upwardZeroAngle += 360.0;
            Serial.print(">> Angle Trim Updated: "); Serial.print(angleTrimDeg);
            Serial.print(" | New Setpoint: "); Serial.println(upwardZeroAngle);
          }
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

// --- Stepper Step Generation ---
void updateStepperEngine() {
  if (!pidEnabled || !motorActive || stepIntervalUs == 0) return;

  unsigned long now = micros();
  if (now - lastStepTime >= stepIntervalUs) {
    lastStepTime = now;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3); 
    digitalWrite(STEP_PIN, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(ENCODER_PIN, ADC_11db);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Enable TMC2209 driver (Active LOW)
  digitalWrite(EN_PIN, LOW);

  calibrateZeroPosition();

  Serial.println("System Initialized & PID Active. Lift pendulum upright to balance.");
  Serial.println("Use 'T-5.0' or 'T-3.5' to fine-tune zero offset live via serial.");

  inputBuffer.reserve(32);
  lastLoopTime = micros();
}

void loop() {
  handleSerialCommands();

  // Continuously attempt stepping
  updateStepperEngine();

  unsigned long now = micros();
  if (now - lastLoopTime >= LOOP_TIME_US) {
    float dt = (now - lastLoopTime) / 1000000.0; 
    lastLoopTime = now;

    float currentRawAngle = getRawAngle();

    // Compute error relative to setpoint
    float error = currentRawAngle - upwardZeroAngle;
    while (error > 180.0)  error -= 360.0;
    while (error < -180.0) error += 360.0;

    // Plot output
    Serial.print("Angle:");
    Serial.print(currentRawAngle, 2);
    Serial.print(",");
    Serial.print("Setpoint:");
    Serial.print(upwardZeroAngle, 2);
    Serial.print(",");
    Serial.print("Error:");
    Serial.println(error, 2);

    // Dynamic balancing threshold set to +/- 20 degrees
    if (!pidEnabled || abs(error) > 30.0) {
      motorActive = false;
      errorSum = 0; 
      lastError = error;
      currentSpeedStepsPerSec = 0.0;
      stepIntervalUs = 0; 
      return;
    }

    motorActive = true;

    // --- PID Calculation ---
    errorSum += error * dt;
    errorSum = constrain(errorSum, -30.0, 30.0);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float targetSpeed = (Kp * error) + (Ki * errorSum) + (Kd * derivative);
    targetSpeed = constrain(targetSpeed, -3200.0, 3200.0);

    // Acceleration Slew
    float maxSpeedChange = MAX_ACCELERATION * dt;
    float speedDifference = targetSpeed - currentSpeedStepsPerSec;
    speedDifference = constrain(speedDifference, -maxSpeedChange, maxSpeedChange);
    currentSpeedStepsPerSec += speedDifference;

    // Direction update
    bool targetDir = (currentSpeedStepsPerSec >= 0.0);
    if (targetDir != currentDirection) {
      currentDirection = targetDir;
      digitalWrite(DIR_PIN, currentDirection ? HIGH : LOW);
    }

    float absSpeed = abs(currentSpeedStepsPerSec);
    if (absSpeed < 10.0) { 
      stepIntervalUs = 0; 
    } else {
      stepIntervalUs = (unsigned long)(1000000.0 / absSpeed);
    }
  }
}