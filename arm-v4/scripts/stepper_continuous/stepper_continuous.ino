#include <AccelStepper.h>

#define M1_STEP_PIN 5
#define M1_DIR_PIN  4
#define M1_EN_PIN   12

#define M2_STEP_PIN 7
#define M2_DIR_PIN  6
#define M2_EN_PIN   8

AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

void setup() {
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);

  // If using 1/16 microstepping, multiply desired speed/accel by 16
  motor1.setMaxSpeed(3200);      
  motor1.setAcceleration(1600);

  motor2.setMaxSpeed(2400);
  motor2.setAcceleration(1200);
}

void loop() {
  // If the motor is getting close to its target position, keep moving it further out
  if (motor1.distanceToGo() == 0) {
    motor1.move(32000); // Add another 32,000 steps ahead
  }
  if (motor2.distanceToGo() == 0) {
    motor2.move(32000);
  }

  motor1.run();
  motor2.run();
}