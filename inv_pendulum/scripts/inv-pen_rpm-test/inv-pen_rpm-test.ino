#include <AccelStepper.h>

// CNC Shield v3 Pin Definitions for Y-Axis Slot
const int STEP_PIN   = 3; // Y.STEP
const int DIR_PIN    = 6; // Y.DIR
const int ENABLE_PIN = 8; // CNC Shield Enable Pin (Active LOW)

// Motor Configuration - Adjust these manually
const float TARGET_RPM  = 160.0;  // Target Speed
const int STEPS_PER_REV = 200;   // 1.8 degree stepper = 200 full steps
const int MICROSTEPS    = 16;    // Match your CNC shield Y-slot jumpers

// AccelStepper Driver Mode (DRIVER = 1 means external driver with Step & Dir pins)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable TMC2209 driver (Active LOW)

  // Calculate required steps per second for the target RPM
  float stepsPerSecond = (TARGET_RPM * STEPS_PER_REV * MICROSTEPS) / 60.0;

  // AccelStepper settings
  stepper.setMaxSpeed(stepsPerSecond);
  stepper.setAcceleration(stepsPerSecond * 2); // Accelerate to full speed in 0.5 seconds
  stepper.setSpeed(stepsPerSecond);
}

void loop() {
  // runSpeed() maintains continuous rotation at maxSpeed without target positions
  stepper.runSpeed(); 
}