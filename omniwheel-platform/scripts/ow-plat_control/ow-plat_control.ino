#include <AccelStepper.h>

// 200 steps/rev * 16 microsteps = 3200 steps per full revolution
#define STEPS_PER_REVOLUTION 3200

// Define pins for CNC Shield V3 (GRBL compatible pinout)
#define X_STEP_PIN 2
#define X_DIR_PIN  5

#define Y_STEP_PIN 3
#define Y_DIR_PIN  6

#define Z_STEP_PIN 4
#define Z_DIR_PIN  7

#define EN_PIN     8

// Target RPMs received from Python
float targetRPM_X = 0.0;
float targetRPM_Y = 0.0;
float targetRPM_Z = 0.0;

// Helper function to convert RPM to steps per second
float rpmToStepsPerSecond(float rpm) {
  return (rpm * STEPS_PER_REVOLUTION) / 60.0;
}

// Initialize AccelStepper for driver mode (1)
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

void setup() {
  Serial.begin(115200);

  // Enable drivers (Active LOW)
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  // Set max speed limits
  stepperX.setMaxSpeed(32000.0);
  stepperY.setMaxSpeed(32000.0);
  stepperZ.setMaxSpeed(32000.0);
}

void loop() {
  // Check if new RPM data has arrived from Python over Serial
  if (Serial.available() > 0) {
    float rX = Serial.parseFloat();
    float rY = Serial.parseFloat();
    float rZ = Serial.parseFloat();

    // Look for newline character to complete the packet
    if (Serial.read() == '\n') {
      targetRPM_X = rX;
      targetRPM_Y = rY;
      targetRPM_Z = rZ;

      // Apply the new speeds immediately
      stepperX.setSpeed(rpmToStepsPerSecond(targetRPM_X));
      stepperY.setSpeed(rpmToStepsPerSecond(targetRPM_Y));
      stepperZ.setSpeed(rpmToStepsPerSecond(targetRPM_Z));
    }
  }

  // Must be called continuously to maintain smooth movement
  stepperX.runSpeed();
  stepperY.runSpeed();
  stepperZ.runSpeed();
}