#include <AccelStepper.h>

// --- Configuration Parameters ---
<<<<<<< HEAD
const int STEPS_PER_REV = 200;           // Standard 1.8-degree stepper
const int MICROSTEPS = 16;               // Change to match DIP switch setting (1, 2, 4, 8, 16, 32)
const float GEAR_REDUCTION_RATIO = 30.0; // Set your gearbox ratio here 
const float CORRECTION_FACTOR = 0.5;     // Fine-tune multiplier (set to 0.5 since 90deg currently gave 180deg)
=======
const int STEPS_PER_REV = 200;          // Standard 1.8-degree stepper
const int MICROSTEPS = 16;              // Change to match DIP switch setting (1, 2, 4, 8, 16, 32)
const float GEAR_REDUCTION_RATIO = 30.0;// Set your gearbox ratio here 
const float CORRECTION_FACTOR = 0.5;    // Fine-tune multiplier (set to 0.5 since 90deg currently gave 180deg)
>>>>>>> 8c335fbecd843be54a1c72e85e4079fdeaa994db

// --- Correct Pin Definitions (Gravity Dual DRV8825 Shield - DFRobot DRI0023) ---
#define M1_DIR_PIN     7
#define M1_STEP_PIN    6
#define M1_EN_PIN      8

#define M2_DIR_PIN     4
#define M2_STEP_PIN    5
#define M2_EN_PIN      12

// Initialize AccelStepper instances in DRIVER mode (Type 1: Step + Direction)
AccelStepper stepper1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// Tracking variable to know when motors are actively moving
bool motorsActive = false;

void setup() {
  // Start Serial communication
  Serial.begin(115200);
  
  // Configure Enable pins
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  
  // Idle by default on boot: DRV8825 ENABLE is Active LOW, so HIGH disables/sleeps them
  digitalWrite(M1_EN_PIN, HIGH);
  digitalWrite(M2_EN_PIN, HIGH);

  // Set safe initial default limits to prevent sudden stalls on boot
  stepper1.setMaxSpeed(1500.0);
  stepper1.setAcceleration(1000.0);

  stepper2.setMaxSpeed(1500.0);
  stepper2.setAcceleration(1000.0);

  Serial.println("Gearbox Stepper Tester Ready (Motors idling by default).");
  Serial.println("Send move command: <motor> <angle_deg> <time_ms> (e.g., 2 90 3000)");
  Serial.println("Send stop command: stop <motor> (e.g., stop 1, or stop 0 for all)");
}

void loop() {
  // Check for incoming serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    parseCommand(input);
  }

  // Must be called continuously in loop to execute position moves
  stepper1.run();
  stepper2.run();

  // Auto-Idling Check: If motors were active and have finished all steps, disable them to prevent overheating
  if (motorsActive && stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
    // DRV8825 ENABLE is Active LOW. Setting HIGH puts drivers to sleep and cuts coil power.
    digitalWrite(M1_EN_PIN, HIGH);
    digitalWrite(M2_EN_PIN, HIGH);
    
    Serial.println("Moves complete. Motors disabled (idling) to prevent overheating.");
    motorsActive = false;
  }
}

// Function to parse serial input and trigger motor movement or stopping
void parseCommand(String command) {
  // Check if it's a stop command
  if (command.startsWith("stop") || command.startsWith("STOP")) {
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex == -1) {
      // Just "stop" entered -> stop both motors
      stepper1.stop();
      stepper2.stop();
      // Put them back to sleep when stopped manually
      digitalWrite(M1_EN_PIN, HIGH);
      digitalWrite(M2_EN_PIN, HIGH);
      motorsActive = false;
      Serial.println("Stopping all motors and setting to idle.");
      return;
    }
    
    int motorId = command.substring(spaceIndex + 1).toInt();
    if (motorId == 1) {
      stepper1.stop();
      digitalWrite(M1_EN_PIN, HIGH);
      Serial.println("Stopping Motor 1 and setting to idle.");
    } else if (motorId == 2) {
      stepper2.stop();
      digitalWrite(M2_EN_PIN, HIGH);
      Serial.println("Stopping Motor 2 and setting to idle.");
    } else if (motorId == 0) {
      stepper1.stop();
      stepper2.stop();
      digitalWrite(M1_EN_PIN, HIGH);
      digitalWrite(M2_EN_PIN, HIGH);
      motorsActive = false;
      Serial.println("Stopping all motors and setting to idle.");
    } else {
      Serial.println("Error: Invalid motor ID for stop command.");
    }
    return;
  }

  // Otherwise, parse standard move command: <motor> <angle> <time_ms>
  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);

  if (firstSpace == -1 || secondSpace == -1) {
    Serial.println("Error: Invalid format. Use <motor> <angle> <time_ms> or 'stop <motor>'");
    return;
  }

  int motorId = command.substring(0, firstSpace).toInt();
  float angle = command.substring(firstSpace + 1, secondSpace).toFloat();
  unsigned long timeMs = command.substring(secondSpace + 1).toInt();

  if (timeMs == 0) {
    Serial.println("Error: Time cannot be zero.");
    return;
  }

  // Calculate total steps required, factoring in microsteps, gearbox ratio, and correction factor
  float stepsPerDegreeOutput = (float)(STEPS_PER_REV * MICROSTEPS * GEAR_REDUCTION_RATIO) / 360.0;
  long targetSteps = round(angle * stepsPerDegreeOutput * CORRECTION_FACTOR);

  // Calculate required speed (steps/second) to finish within the requested time
  float timeSec = timeMs / 1000.0;
  float requiredSpeed = abs(targetSteps) / timeSec;

  // Safety ceiling: Cap max speed to prevent heavy gearboxes from instantly stalling/squealing
  if (requiredSpeed > 3000.0) {
    requiredSpeed = 3000.0;
  }

  // Wake up both drivers before issuing a movement command (Active LOW)
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  motorsActive = true;

  if (motorId == 1) {
    stepper1.setMaxSpeed(requiredSpeed);
    stepper1.setAcceleration(requiredSpeed * 2.0); // Smoother acceleration curve
    stepper1.move(targetSteps);
    
    Serial.print("Motor 1 -> Output Angle: "); Serial.print(angle);
    Serial.print(" deg, Time: "); Serial.print(timeMs); Serial.println(" ms");
  } 
  else if (motorId == 2) {
    stepper2.setMaxSpeed(requiredSpeed);
    stepper2.setAcceleration(requiredSpeed * 2.0);
    stepper2.move(targetSteps);
    
    Serial.print("Motor 2 -> Output Angle: "); Serial.print(angle);
    Serial.print(" deg, Time: "); Serial.print(timeMs); Serial.println(" ms");
  } 
  else {
    Serial.println("Error: Invalid motor ID. Use 1 or 2.");
  }
}