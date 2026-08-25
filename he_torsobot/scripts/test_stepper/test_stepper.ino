#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

// --- Configuration ---
const float TARGET_RPM = 100.0; // Set your desired RPM here
const int MICROSTEPS = 16;      // Your driver jumper setting (1/16)
const int STEPS_PER_REV = 200;  // 1.8 degree stepper default

// --- CNC Shield V3 Pin Mapping (Y-Axis) ---
#define STEP_PIN 3
#define DIR_PIN 6
#define ENABLE_PIN 8

// --- TMC2209 UART Configuration ---
#define SW_UART_PIN 11      // Connect pin 11 to the TMC2209 UART/PDN pin
#define R_SENSE 0.11f       // Standard sense resistor value (R110)
#define DRIVER_ADDRESS 0b00 // Default MS1/MS2 address setting

// Setup SoftwareSerial on a single pin for hardware simplicity
SoftwareSerial tmcSerial(SW_UART_PIN, SW_UART_PIN); 
TMC2209Stepper driver(&tmcSerial, R_SENSE, DRIVER_ADDRESS);

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable drivers (Active LOW)

  // Initialize UART communication with TMC2209
  tmcSerial.begin(115200);
  driver.begin();
  
  // Set driver current (1040mA RMS is ~80% limit for a 1.3A peak motor)
  driver.rms_current(1040);
  driver.microsteps(MICROSTEPS);
  
  // --- Enable High-Torque Mode ---
  driver.en_spreadCycle(true); // true = SpreadCycle (High Torque), false = StealthChop (Quiet)
  driver.pwm_autoscale(true);  // Automatic current scaling
  driver.toff(5);              // Enable off-time to turn chopper on

  // Math: RPM -> Steps/Sec
  // (RPM / 60 sec) * (200 full steps * 16 microsteps)
  float stepsPerSec = (TARGET_RPM / 60.0) * (STEPS_PER_REV * MICROSTEPS);

  stepper.setMaxSpeed(stepsPerSec);
  stepper.setSpeed(stepsPerSec);
}

void loop() {
  stepper.runSpeed();
}