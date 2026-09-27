#include <AccelStepper.h>

// --- Configuration Parameters ---
const float TARGET_RPM_1 = 15.0;    // Target speed for Motor 1 in RPM
const float TARGET_RPM_2 = 120.0;   // Target speed for Motor 2 in RPM

const int STEPS_PER_REV = 200;      // Standard 1.8-degree stepper
const int MICROSTEPS = 16;           // Change to match DIP switch setting (1, 2, 4, 8, 16, 32)

// --- Pin Definitions (Gravity Dual DRV8825 Shield) ---
#define M1_DIR_PIN   4
#define M1_STEP_PIN  5
#define M1_EN_PIN    6

#define M2_DIR_PIN   7
#define M2_STEP_PIN  8
#define M2_EN_PIN    12

// Initialize AccelStepper instances in DRIVER mode (Type 1: Step + Direction)
AccelStepper stepper1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// Function to convert RPM to steps/second
float rpmToStepsPerSec(float rpm, int stepsPerRev, int microsteps) {
  return (rpm * stepsPerRev * microsteps) / 60.0;
}

void setup() {
  // Configure Enable pins
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  
  // DRV8825 ENABLE pin is Active LOW (LOW = Enabled, HIGH = Disabled/Sleep)
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);

  // Calculate speeds
  float speed1 = rpmToStepsPerSec(TARGET_RPM_1, STEPS_PER_REV, MICROSTEPS);
  float speed2 = rpmToStepsPerSec(TARGET_RPM_2, STEPS_PER_REV, MICROSTEPS);

  // Set maximum limits and target constant speeds
  stepper1.setMaxSpeed(speed1);
  stepper1.setSpeed(speed1);

  stepper2.setMaxSpeed(speed2);
  stepper2.setSpeed(speed2);
}

void loop() {
  // Constant speed motion without acceleration blocking
  stepper1.runSpeed();
  stepper2.runSpeed();
}