#include <AccelStepper.h>

// Pin Definitions for Gravity Dual Stepper Shield
#define M1_STEP_PIN 5
#define M1_DIR_PIN  4
#define M1_EN_PIN   12

#define M2_STEP_PIN 7
#define M2_DIR_PIN  6
#define M2_EN_PIN   8

// Configuration Parameters
const float MICROSTEPS = 16.0;       // Set to match DIP switches (1, 2, 4, 8, 16, 32)
const float STEPS_PER_REV = 200.0;  // Standard 1.8° stepper motor (17HS3401S)

// Driver setup (DRIVER mode: Pin 1 = STEP, Pin 2 = DIR)
AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// Helper function to set target RPM and calculated acceleration
void setMotorRPM(AccelStepper &motor, float targetRPM, float accelFactor = 1.0) {
  // Steps/sec = (RPM / 60) * (200 steps/rev * microsteps)
  float stepsPerSec = (targetRPM / 60.0) * (STEPS_PER_REV * MICROSTEPS);
  
  motor.setMaxSpeed(stepsPerSec);
  // Sets acceleration relative to top speed (1.0 = reaches top speed in 1 second)
  motor.setAcceleration(stepsPerSec * accelFactor); 
}

void setup() {
  // Enable DRV8825 drivers (LOW = Enabled)
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);

  // Set Motor 1 to 120 RPM (2 Revolutions Per Second)
  setMotorRPM(motor1, 50.0);

  // Set Motor 2 to 180 RPM (3 Revolutions Per Second)
  setMotorRPM(motor2, 25.0);
}

void loop() {
  // Maintain continuous rotation
  if (motor1.distanceToGo() == 0) {
    motor1.move(100000); // Queue up continuous steps
  }
  if (motor2.distanceToGo() == 0) {
    motor2.move(100000);
  }

  // Execute steps (must be called as fast as possible in loop)
  motor1.run();
  motor2.run();
}