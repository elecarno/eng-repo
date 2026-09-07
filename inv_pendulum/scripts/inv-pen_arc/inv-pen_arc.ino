#include <Arduino.h>
#include <FastAccelStepper.h>

// ============================================================================
// PINS & HARDWARE CONFIGURATION
// ============================================================================
const int PIN_ENCODER = 34; // MT6701 Analog output pin
const int PIN_STEP    = 25; // Driver STEP pin
const int PIN_DIR     = 26; // Driver DIR pin
const int PIN_EN      = 27; // Driver EN pin (Active LOW)

const float TWO_PI_F = 6.28318530718f;

// Set your desired swing amplitude (half-arc angle from hanging center)
float TARGET_ARC_DEG = 30.0; // Oscillates between -30 deg and +30 deg

// Energy Pumping Motion Settings
const int32_t BASE_PULSE_STEPS = 150; // Steps to move per excitation pulse (~17 deg arm motion)
const float EMA_ALPHA          = 0.70; // Encoder filter coefficient

// Calibration / Reference
float ENC_ZERO_RADIANS = 0.0;

// ============================================================================
// STATE TRACKING
// ============================================================================
float filteredRadians  = -1.0;
float previousRadians  = 0.0;
float currentVelocity  = 0.0; // rad/s

float peakAngleDeg     = 0.0; // Stores peak angle reached in the current swing
int swingDirection     = 0;   // +1 = moving clockwise, -1 = counter-clockwise

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// ============================================================================
// SENSOR & CALIBRATION
// ============================================================================
void calibrateZero() {
  Serial.println(">> CALIBRATING: Leave pendulum stationary at bottom...");
  delay(1000);
  
  uint32_t adcSum = 0;
  for (int i = 0; i < 200; i++) {
    adcSum += analogRead(PIN_ENCODER);
    delay(5);
  }
  
  ENC_ZERO_RADIANS = ((float)adcSum / 200.0f / 4095.0f) * TWO_PI_F;
  Serial.print(">> ZERO CALIBRATED: "); Serial.println(ENC_ZERO_RADIANS, 4);
}

float getHangingAngleDeg() {
  uint32_t rawADC = analogRead(PIN_ENCODER); 
  float rawRad = (rawADC / 4095.0f) * TWO_PI_F;
  
  float normRad = rawRad - ENC_ZERO_RADIANS;
  if (normRad > PI)  normRad -= TWO_PI_F;
  if (normRad < -PI) normRad += TWO_PI_F;

  // Filter signal
  if (filteredRadians < -100.0f) {
    filteredRadians = normRad;
  } else {
    filteredRadians = (EMA_ALPHA * normRad) + ((1.0f - EMA_ALPHA) * filteredRadians);
  }

  // Convert to degrees relative to hanging bottom (0 deg = bottom)
  return filteredRadians * (180.0f / PI);
}

// ============================================================================
// LIVE SERIAL COMMAND PARSER
// ============================================================================
void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("Arc=") || input.startsWith("arc=")) {
      TARGET_ARC_DEG = input.substring(4).toFloat();
      Serial.print(">> UPDATED TARGET ARC: +/- ");
      Serial.print(TARGET_ARC_DEG, 1);
      Serial.println(" deg");
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ENCODER, ADC_11db);

  calibrateZero();

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (stepper) {
    stepper->setDirectionPin(PIN_DIR);
    stepper->setEnablePin(PIN_EN, true);
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz(35000);     
    stepper->setAcceleration(300000); 
  }

  Serial.println(">> PENDULUM DRIVER ACTIVE.");
  Serial.println(">> Type 'Arc=45' in Serial Monitor to set target arc angle.");
}

// ============================================================================
// MAIN LOOP (100 Hz State Engine)
// ============================================================================
void loop() {
  processSerialInput();

  static unsigned long lastMs = 0;
  unsigned long nowMs = millis();

  if (nowMs - lastMs >= 10) { // 100 Hz Loop
    float dt = (nowMs - lastMs) / 1000.0f;
    lastMs = nowMs;

    float currentAngleDeg = getHangingAngleDeg();
    
    // Estimate velocity (deg/s)
    currentVelocity = (currentAngleDeg - previousRadians) / dt;
    previousRadians = currentAngleDeg;

    // Track peak angle reached during current direction swing
    if (fabs(currentAngleDeg) > peakAngleDeg) {
      peakAngleDeg = fabs(currentAngleDeg);
    }

    // Determine current motion direction
    int currentDir = (currentVelocity > 5.0f) ? 1 : ((currentVelocity < -5.0f) ? -1 : 0);

    // Peak / Velocity Reversal Detection (Energy injection trigger point)
    if (currentDir != 0 && currentDir != swingDirection) {
      swingDirection = currentDir;

      // Energy Pumping Decision
      if (peakAngleDeg < TARGET_ARC_DEG) {
        // Calculate energy deficit factor (0.0 to 1.0)
        float deficit = (TARGET_ARC_DEG - peakAngleDeg) / TARGET_ARC_DEG;
        deficit = constrain(deficit, 0.2f, 1.0f);

        // Scale pulse size proportionally to how far we are from target
        int32_t stepOffset = (int32_t)(BASE_PULSE_STEPS * deficit);

        // Move pivot in the same direction pendulum is heading to push it
        if (stepper) {
          int32_t currentPos = stepper->getCurrentPosition();
          if (swingDirection > 0) {
            stepper->moveTo(currentPos + stepOffset);
          } else {
            stepper->moveTo(currentPos - stepOffset);
          }
        }
      }

      // Reset peak angle tracker for next direction swing
      peakAngleDeg = 0.0f;
    }

    // Telemetry output for Arduino Serial Plotter
    Serial.print("CurrentAngle:"); Serial.print(currentAngleDeg, 1); Serial.print(",");
    Serial.print("TargetArc:");    Serial.print(TARGET_ARC_DEG, 1);    Serial.print(",");
    Serial.print("NegativeTarget:"); Serial.println(-TARGET_ARC_DEG, 1);
  }
}