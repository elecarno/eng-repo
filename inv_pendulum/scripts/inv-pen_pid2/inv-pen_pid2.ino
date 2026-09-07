#include <Arduino.h>
#include <FastAccelStepper.h>

// ============================================================================
// PINS & HARDWARE CONFIGURATION
// ============================================================================
const int PIN_ENCODER = 34; // MT6701 Analog output pin
const int PIN_STEP    = 25; // Driver STEP pin
const int PIN_DIR     = 26; // Driver DIR pin
const int PIN_EN      = 27; // Driver EN pin (Active LOW)

// Stepper Hardware Setup (1/16 Microstepping -> MS1=3.3V, MS2=3.3V on TMC2209)
const int STEPS_PER_REV      = 200;
const int MICROSTEPS         = 16;
const int MICROSTEPS_PER_REV = STEPS_PER_REV * MICROSTEPS; // 3200 steps/rev

// Safety & Calibration Limits
const float SAFETY_CUTOFF_ANGLE = 0.8;    // ~45.8 degrees from upright
const float TARGET_RADIANS      = PI;     // Target upright equilibrium (~3.14159 rad)
float ENC_ZERO_RADIANS    = 0.0; // Analog offset at downward equilibrium

// ============================================================================
// SYSTEM TIMING & CONTROL TUNING (70mm / 70mm Setup)
// ============================================================================
const unsigned long CONTROL_INTERVAL_MS = 5; // 200 Hz loop execution
const float EMA_ALPHA = 0.65;                // Fast filter coefficient to minimize phase delay

// Base Gains (Live Tunable)
float Kp = 250.0;
float Ki = 0.0;
float Kd = 35.0;

// Deadband Compensator (Stiction Feedforward Offset)
float MIN_STEP_OFFSET = 12.0;

// ============================================================================
// STATE TRACKING
// ============================================================================
unsigned long last_micros = 0;
double integral = 0.0;
double previous_error = 0.0;
float filteredRadians = -1.0;

bool is_balancing = false;
unsigned long active_start_ms = 0;
float last_stable_duration_sec = 0.0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// ============================================================================
// ADC & SENSOR READOUT
// ============================================================================
float getEncoderRadians() {
  uint32_t rawADC = analogRead(PIN_ENCODER); 
  float rawRadians = (rawADC / 4095.0) * TWO_PI;
  
  float outputRadians = rawRadians - ENC_ZERO_RADIANS;
  if (outputRadians < 0.0) {
    outputRadians += TWO_PI;
  }
  
  // Single-pole Exponential Moving Average (EMA) filter
  if (filteredRadians < 0.0) {
    filteredRadians = outputRadians;
  } else {
    filteredRadians = (EMA_ALPHA * outputRadians) + ((1.0 - EMA_ALPHA) * filteredRadians);
  }
  
  return filteredRadians;
}

// ============================================================================
// PID & DEADBAND CONTROL CALCULATION
// ============================================================================
double calculateControlOutput(double error, double dt_sec) {
  if (dt_sec <= 0.0) return 0.0;

  // 1. Proportional Term
  double p_term = Kp * error;

  // 2. Integral Term (Clamped for Anti-Windup)
  integral += error * dt_sec;
  integral = constrain(integral, -2.0, 2.0); 
  double i_term = Ki * integral;

  // 3. Derivative Term
  double derivative = (error - previous_error) / dt_sec;
  previous_error = error;
  double d_term = Kd * derivative;

  double raw_output = p_term + i_term + d_term;

  // 4. Deadband Compensator (Bypasses motor/driver friction threshold)
  if (abs(raw_output) > 0.1) {
    if (raw_output > 0) {
      raw_output += MIN_STEP_OFFSET;
    } else {
      raw_output -= MIN_STEP_OFFSET;
    }
  }

  return raw_output;
}

// ============================================================================
// NON-BLOCKING SERIAL PARSER FOR RUNTIME TUNING
// ============================================================================
void processSerialInput() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("Kp=") || command.startsWith("kp=")) {
      Kp = command.substring(3).toFloat();
      Serial.print(">> Kp updated: "); Serial.println(Kp, 1);
    } 
    else if (command.startsWith("Kd=") || command.startsWith("kd=")) {
      Kd = command.substring(3).toFloat();
      Serial.print(">> Kd updated: "); Serial.println(Kd, 1);
    } 
    else if (command.startsWith("Ki=") || command.startsWith("ki=")) {
      Ki = command.substring(3).toFloat();
      Serial.print(">> Ki updated: "); Serial.println(Ki, 1);
    }
    else if (command.startsWith("Offset=") || command.startsWith("offset=")) {
      MIN_STEP_OFFSET = command.substring(7).toFloat();
      Serial.print(">> MIN_STEP_OFFSET updated: "); Serial.println(MIN_STEP_OFFSET, 1);
    }
  }
}


// Reads hanging downward equilibrium position at boot
void calibrateEncoderZero() {
  Serial.println(">> CALIBRATING: Let pendulum hang stationary...");
  delay(1000); // Allow physical oscillations to settle

  uint32_t adcSum = 0;
  const int SAMPLES = 200;

  for (int i = 0; i < SAMPLES; i++) {
    adcSum += analogRead(PIN_ENCODER);
    delay(5); // 200 samples * 5ms = 1.0 second sample window
  }

  float avgADC = (float)adcSum / SAMPLES;
  ENC_ZERO_RADIANS = (avgADC / 4095.0) * TWO_PI;

  Serial.print(">> CALIBRATION COMPLETE. Measured Offset: ");
  Serial.print(ENC_ZERO_RADIANS, 4);
  Serial.println(" rad");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ENCODER, ADC_11db);

  // Auto-zero the encoder reference offset
  calibrateEncoderZero();

  // Hardware Pulse Generation Engine initialization
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  
  if (stepper) {
    stepper->setDirectionPin(PIN_DIR);
    stepper->setEnablePin(PIN_EN, true);
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz(50000);     
    stepper->setAcceleration(450000); 
  }
  
  last_micros = micros();
  Serial.println(">> SYSTEM READY. Raise pendulum to vertical to engage.");
}

// ============================================================================
// MAIN EXECUTION LOOP
// ============================================================================
void loop() {
  processSerialInput();

  static unsigned long lastExecutionMs = 0;
  unsigned long currentMs = millis();

  // Enforce rigid 200 Hz loop timing
  if (currentMs - lastExecutionMs >= CONTROL_INTERVAL_MS) {
    lastExecutionMs = currentMs;

    // Time delta calculation using integer subtraction
    unsigned long now_micros = micros();
    double dt = (now_micros - last_micros) / 1000000.0;
    last_micros = now_micros;

    // Sensor readout & error calculation
    float alpha = getEncoderRadians();
    double alpha_error = TARGET_RADIANS - alpha; 

    // Safety Cutoff Trigger
    if (fabs(alpha_error) > SAFETY_CUTOFF_ANGLE) {
      if (stepper) stepper->stopMove();
      integral = 0.0;
      previous_error = 0.0;
      filteredRadians = -1.0; // Reset filter state to clear outdated history
      
      if (is_balancing) {
        last_stable_duration_sec = (currentMs - active_start_ms) / 1000.0f;
        is_balancing = false;
        
        Serial.print(">> SAFETY CUTOFF TRIGGERED. Duration: ");
        Serial.print(last_stable_duration_sec, 2);
        Serial.println(" s <<");
      }
      return;
    }

    // Balance Engagement State Transition
    if (!is_balancing) {
      is_balancing = true;
      active_start_ms = currentMs;
      previous_error = alpha_error; // Pre-load error to suppress derivative spikes
    }

    // Compute PID displacement
    double stepOffset = calculateControlOutput(alpha_error, dt);

    // Issue hardware-accelerated move command
    if (stepper) {
      int32_t currentPos = stepper->getCurrentPosition();
      int32_t targetStep = currentPos - (int32_t)stepOffset; 
      stepper->moveTo(targetStep);
    }

    // Telemetry output for Arduino Serial Plotter
    Serial.print("Angle:");            Serial.print(alpha, 3);                 Serial.print(",");
    Serial.print("Kp:");               Serial.print(Kp, 1);                    Serial.print(",");
    Serial.print("Kd:");               Serial.print(Kd, 1);                    Serial.print(",");
    Serial.print("Offset:");           Serial.print(MIN_STEP_OFFSET, 1);       Serial.print(",");
    Serial.print("LastStableSec:");   Serial.println(last_stable_duration_sec, 2);
  }
}