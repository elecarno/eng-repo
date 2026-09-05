#include <Arduino.h>

// --- Pin Definitions ---
// MT6701 Pin
const int ENCODER_PIN = 34; // ADC1 pin for analog angle readout

// TMC2209 Control Pins
const int STEP_PIN = 25;
const int DIR_PIN  = 26;
const int EN_PIN   = 27;

// --- Motion Settings ---
// Assumes standard 1.8° stepper motor at 1/16 microstepping:
// 10 degrees = ~5.55 full steps * 16 microsteps = 89 microsteps
const int STEPS_10_DEG      = 89;   
const int STEP_DELAY_MICROS = 800; // Step speed (lower = faster)
const int PAUSE_BETWEEN_ARC = 150; // Delay between wiggles in ms

// --- Encoder Helper Function ---
// Reads filtered angle in degrees from MT6701 analog pin
float getEncoderAngle() {
  uint32_t rawSum = 0;
  for (int i = 0; i < 16; i++) {
    rawSum += analogRead(ENCODER_PIN);
    delayMicroseconds(20);
  }
  return ((rawSum / 16.0) / 4095.0) * 360.0; // Convert 12-bit ADC (0-4095) to 0-360°
}

// --- Motor Step Helper Function ---
void moveSteps(bool dir, int steps) {
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_MICROS);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_MICROS);
  }
}

void setup() {
  Serial.begin(115200);

  // Setup ADC
  analogReadResolution(12);
  analogSetPinAttenuation(ENCODER_PIN, ADC_11db);

  // Setup TMC2209 Control Pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Enable Driver (Active LOW on TMC2209)
  digitalWrite(EN_PIN, LOW);

  Serial.println("TMC2209 Wiggle + MT6701 Encoder Reading Active...");
}

void loop() {
  // Read angle before moving
  float startAngle = getEncoderAngle();

  // 1. Move Forward ~10 Degrees
  moveSteps(true, STEPS_10_DEG);
  delay(PAUSE_BETWEEN_ARC);
  float forwardAngle = getEncoderAngle();

  // 2. Move Backward ~10 Degrees
  moveSteps(false, STEPS_10_DEG);
  delay(PAUSE_BETWEEN_ARC);
  float returnAngle = getEncoderAngle();

  // --- Output Angles to Serial Monitor ---
  Serial.print("Current Angle: ");
  Serial.print(returnAngle, 2);
  Serial.print("°  |  Last Arc Travel: ");
  Serial.print(abs(forwardAngle - startAngle), 2);
  Serial.println("°");
}