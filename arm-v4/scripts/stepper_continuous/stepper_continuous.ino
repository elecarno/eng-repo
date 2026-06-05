#include <AccelStepper.h>

// Define shield pins
#define M1_STEP_PIN 5
#define M1_DIR_PIN  4
#define M1_EN_PIN   12

#define M2_STEP_PIN 7
#define M2_DIR_PIN  6
#define M2_EN_PIN   8

// Initialize the library using the standard DRIVER interface (Type 1)
AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

void setup() {
  // Shield requires us to manually handle the enable pins
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, LOW); // Turn on driver 1
  digitalWrite(M2_EN_PIN, LOW); // Turn on driver 2

  // Set maximum speeds (steps per second)
  motor1.setMaxSpeed(1000);
  motor2.setMaxSpeed(800);

  // Set constant running speeds (negative numbers will reverse direction)
  motor1.setSpeed(600); 
  motor2.setSpeed(400); 
}

void loop() {
  // runSpeed() tells the motors to constantly spin at the designated speed 
  motor1.runSpeed();
  motor2.runSpeed();
}