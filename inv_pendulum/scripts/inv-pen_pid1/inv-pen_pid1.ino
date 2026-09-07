#include <Arduino.h>
#include <FastAccelStepper.h>

// ============================================================================
// PINS & HARDWARE CONFIGURATION
// ============================================================================
// MT6701 Encoder (Pendulum Joint)
const int PIN_ENCODER = 34; // Analog readout pin

// TMC2209 Driver Pins
const int PIN_STEP = 25;
const int PIN_DIR  = 26;
const int PIN_EN   = 27;

// Stepper Configuration
const int STEPS_PER_REV        = 200; // Standard 1.8° motor
const int MICROSTEPS           = 16;  // Matches MS1/MS2 jumper setting
const int MICROSTEPS_PER_REV   = STEPS_PER_REV * MICROSTEPS; // 3200 steps/rev

const float SAFETY_CUTOFF_ANGLE = 0.8; // in radians

// ============================================================================
// SYSTEM PARAMETERS & PID GAINS
// ============================================================================
const int LOGIC_INTERVAL = 10; // Control loop interval: 10ms (100 Hz)

// Calibration Constants
const float ENC_ZERO_RADIANS = 0.7069; // Offset for pendulum pointing down
const float TARGET_RADIANS   = PI;     // Upright target (~3.14159 rad)

// PID Gains for Rotary System
// Note: Positive steps move the motor CLOCKWISE
float kp = 1200.0; // Converts radian error directly to target step offset
float ki = 2000.0;    // Kept at 0.0 during initial balance tuning
float kd = 50.0;  // Derivative dampening term

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================
double dt = 0.01;
double last_time = 0.0;
double integral = 0.0;
double previous_error = 0.0;

// Balance timing tracking variables
bool is_balancing = false;
unsigned long active_start_ms = 0;
float last_stable_duration_sec = 0.0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Fast single-sample ADC read (~10 µs execution time)
float getEncoderRadiansFast() {
  uint32_t rawADC = analogRead(PIN_ENCODER); 
  float rawRadians = (rawADC / 4095.0) * (2.0 * PI);
  
  float outputRadians = rawRadians - ENC_ZERO_RADIANS;
  if (outputRadians < 0) {
    outputRadians += (2.0 * PI);
  }
  
  return outputRadians;
}

// Compute PID output value
double computePID(double error, double dt_sec) {
  if (dt_sec <= 0) return 0.0;

  // 1. Proportional Term
  double p_term = kp * error;

  // 2. Integral Term with Clamping (Anti-Windup)
  integral += error * dt_sec;
  integral = constrain(integral, -2.0, 2.0); 
  double i_term = ki * integral;

  // 3. Derivative Term (Damps rapid velocity changes)
  double derivative = (error - previous_error) / dt_sec;
  previous_error = error;
  double d_term = kd * derivative;

  return p_term + i_term + d_term;
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Configure ESP32 ADC for MT6701 analog reading
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ENCODER, ADC_11db);

  // Initialize Hardware FastAccelStepper Engine
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  
  if (stepper) {
    stepper->setDirectionPin(PIN_DIR);
    stepper->setEnablePin(PIN_EN, true); // True = Active LOW logic
    stepper->setAutoEnable(true);

    // Dynamic speed/acceleration limits for fast balance corrections
    stepper->setSpeedInHz(40000);     // Max steps per second
    stepper->setAcceleration(300000); // Acceleration in steps/second^2
  }
  
  last_time = micros() / 1000000.0;
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================
void loop() {
  static unsigned long lastExecution = 0;

  // Fixed 100 Hz Control Loop
  if (millis() - lastExecution >= LOGIC_INTERVAL) {
    lastExecution = millis();

    // 1. Calculate precise dt in seconds
    double now = micros() / 1000000.0;
    dt = now - last_time;
    last_time = now;

    // 2. Fetch pendulum position from MT6701
    float alpha = getEncoderRadiansFast();
    
    // 3. Compute continuous angle error relative to vertical upright (PI)
    double alpha_error = TARGET_RADIANS - alpha; 

    // 4. Safety Cutoff & Timing Handling
    if (fabs(alpha_error) > SAFETY_CUTOFF_ANGLE) {
      if (stepper) stepper->stopMove();
      integral = 0.0;
      previous_error = 0.0;
      
      // If we were just actively balancing, stop the timer and calculate duration
      if (is_balancing) {
        last_stable_duration_sec = (millis() - active_start_ms) / 1000.0f;
        is_balancing = false;
        
        Serial.print(">> RUN ENDED. Last Stable Duration: ");
        Serial.print(last_stable_duration_sec, 2);
        Serial.println(" s <<");
      }

      // Output serial status when disabled
      Serial.print("Angle:");           Serial.print(alpha, 3);                 Serial.print(",");
      Serial.print("Setpoint:");        Serial.print(TARGET_RADIANS, 3);        Serial.print(",");
      Serial.print("LastStableSec:");  Serial.println(last_stable_duration_sec, 2);
      return;
    }

    // 5. Start timer if entering active balancing state
    if (!is_balancing) {
      is_balancing = true;
      active_start_ms = millis();
    }

    float current_stable_time = (millis() - active_start_ms) / 1000.0f;

    // 6. Calculate PID step displacement
    double pidStepOffset = computePID(alpha_error, dt);

    // 7. Apply Target Position (Sign inverted for Clockwise response)
    int32_t currentPos = stepper ? stepper->getCurrentPosition() : 0;
    int32_t targetStep = currentPos - (int32_t)pidStepOffset; 

    if (stepper) {
      stepper->moveTo(targetStep);
    }

    // 8. Telemetry (Compatible with Arduino Serial Plotter)
    Serial.print("Angle:");            Serial.print(alpha, 3);                 Serial.print(",");
    Serial.print("Setpoint:");         Serial.print(TARGET_RADIANS, 3);        Serial.print(",");
    // Serial.print("ActiveTimeSec:");    Serial.print(current_stable_time, 2);   Serial.print(",");
    Serial.print("LastStableSec:");   Serial.println(last_stable_duration_sec, 2);
  }
}