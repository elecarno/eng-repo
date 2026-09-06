#include <Arduino.h>
#include <FastAccelStepper.h>

// --- PINS -----------------------------------------------------------
// MT6701
const int PIN_ENCODER = 34; // pin for analog angle readout

// TMC2209
const int PIN_STEP = 25;
const int PIN_DIR  = 26;
const int PIN_EN   = 27;

// --- STEPPER CONFIG -------------------------------------------------
const int STEPS_PER_REV = 200; // 1.8 degree per step
const int MICROSTEPS    = 16;  // match with physical MS1/MS2 pin configuration
const int MICROSTEPS_PER_REV = STEPS_PER_REV * MICROSTEPS;

const float STEPS_TO_RADIANS = (2*PI) / MICROSTEPS_PER_REV;

/// --- PID -----------------------------------------------------------
const float kp = 0.0;
const float ki = 0.0;
const float kd = 0.0;

double dt, last_time;
double integral, previous, output = 0;

// --- GLOBALS --------------------------------------------------------
const int LOGIC_INTERVAL = 16; // in ms

// angle offset for pendulum hanging straight down
const float ENC_ZERO_DEGREES = 40.5;
const float ENC_ZERO_RADIANS = 0.7069;

// initialise FastAccelStepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// \/\/ TARGET ANGLE \/\/
const float TARGET_DEGREES = 176.5;
const float TARGET_RADIANS = 3.081;
// /\/\ TARGET ANGLE /\/\

int32_t stepperTargetPosition = 0;

// --- FUNCTIONS ------------------------------------------------------
float getEncoderDegrees() {
  uint32_t rawSum = 0;
  for (int i = 0; i < 16; i++) {
    rawSum += analogRead(PIN_ENCODER);
    delayMicroseconds(20);
  }

  // convert 12-bit ADC (0-4095) to 0-360°
  float rawDegrees = ( (rawSum/16.0) / 4095.0 ) * 360.0;
  float outputDegrees = 0;

  if (rawDegrees >= ENC_ZERO_DEGREES) {
    outputDegrees = rawDegrees - ENC_ZERO_DEGREES;
  } else {
    outputDegrees = 360.0 - ENC_ZERO_DEGREES + rawDegrees;
  }

  return outputDegrees;
}

float getEncoderRadians() {
  uint32_t rawSum = 0;
  for (int i = 0; i < 16; i++) {
    rawSum += analogRead(PIN_ENCODER);
    delayMicroseconds(20);
  }

  // convert 12-bit ADC (0-4095) to radians (0 to 2pi)
  float rawRadians = ( (rawSum/16.0) / 4095.0 ) * (2*PI);
  float outputRadians = 0;

  if (rawRadians >= ENC_ZERO_RADIANS) {
    outputRadians = rawRadians - ENC_ZERO_RADIANS;
  } else {
    outputRadians = (2*PI) - ENC_ZERO_RADIANS + rawRadians;
  }

  return outputRadians;
}

float getStepperAngleRadians() {
  if (stepper) {
    return stepper->getCurrentPosition() * STEPS_TO_RADIANS;
  }
  return 0.0;
}

// --- MAIN------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // MT6701 analog read
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ENCODER, ADC_11db);

  // TMC2209 with FastAccelStepper
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (stepper) {
    stepper->setDirectionPin(PIN_DIR);
    stepper->setEnablePin(PIN_EN, true); // Active Low
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz(20000);        // Max speed (steps/sec)
    stepper->setAcceleration(100000);    // High acceleration for rapid response
  }
}

void loop() {
  if (stepper && !stepper->isRunning()) {
    if (stepperTargetPosition == 400) {
      stepperTargetPosition = -400;
    } else {
      stepperTargetPosition = 400;
    }
    stepper->moveTo(stepperTargetPosition);
  }

  static unsigned long lastExecution = 0;

  if (millis() - lastExecution >= LOGIC_INTERVAL) {
    lastExecution = millis();

    float encoderAngle = getEncoderRadians();
    float stepperAngle = getStepperAngleRadians();

    // PID LOGIC
    // stepperTargetPosition = 0;

    // if (stepper) {
    //   stepper->moveTo(stepperTargetPosition);
    // }

    Serial.print("pen_angle:");
    Serial.print(encoderAngle, 2);
    Serial.print(",");
    Serial.print("stepper_angle:");
    Serial.println(stepperAngle, 2);
  }
}

