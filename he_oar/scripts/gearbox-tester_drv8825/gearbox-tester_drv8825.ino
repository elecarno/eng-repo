#include <AccelStepper.h>

// --- Configuration Parameters ---
const int STEPS_PER_REV = 200;          // Standard 1.8-degree stepper
const int MICROSTEPS = 16;              // Change to match DIP switch setting (1, 2, 4, 8, 16, 32)

// --- Independent Gear Ratios & Correction Factors per Motor ---
const float GEAR_REDUCTION_RATIO_1 = 35.0; // Gearbox ratio for Motor 1
const float CORRECTION_FACTOR_1    = 0.5;  // Fine-tune multiplier for Motor 1

const float GEAR_REDUCTION_RATIO_2 = 30.0; // Gearbox ratio for Motor 2
const float CORRECTION_FACTOR_2    = 0.5;  // Fine-tune multiplier for Motor 2

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

// Tracking variables
bool motorsActive = false;

// --- Preprogrammed Test Sequence Configuration ---
struct TestStep {
  int motorId;              // 1 or 2
  float angle;              // Target angle in degrees (+ or -)
  unsigned long moveTimeMs; // Time allocated for the move
  unsigned long pauseMs;    // Pause duration after the move completes
};

// Define your looping test profile here:
TestStep testSequence[] = {
  {1, -90.0, 5000, 1000},   // Motor 1: -90 deg in 5.0s, pause 1.0s
  {1,  90.0, 5000, 1000},   // Motor 1: +90 deg in 5.0s, pause 1.0s
  {2,  90.0, 5000, 1000},    // Motor 2: +90 deg in 5.0s, pause 0.1s
  {2, -90.0, 5000, 1000}    // Motor 2: -90 deg in 5.0s, pause 1.0s
};
const int SEQUENCE_LENGTH = sizeof(testSequence) / sizeof(testSequence[0]);

// Sequence State Machine Variables
bool sequenceRunning = false; // Start paused on boot
int currentStepIndex = 0;
enum TestState { STATE_IDLE, STATE_MOVING, STATE_PAUSING };
TestState testState = STATE_IDLE;
unsigned long pauseStartTime = 0;
int loopCount = 0;
const int TARGET_LOOPS = -1; // Set number of loops (-1 for infinite continuous looping)

void setup() {
  Serial.begin(115200);
  
  pinMode(M1_EN_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);
  
  // Idle by default on boot
  digitalWrite(M1_EN_PIN, HIGH);
  digitalWrite(M2_EN_PIN, HIGH);

  stepper1.setMaxSpeed(1500.0);
  stepper1.setAcceleration(1000.0);

  stepper2.setMaxSpeed(1500.0);
  stepper2.setAcceleration(1000.0);

  Serial.println("Gearbox Stepper Tester Initialized.");
  Serial.println("Motors are idling. Send 'start' to begin the sequence.");
  Serial.println("Send 'stop' via Serial Monitor to abort testing at any time.");
}

void loop() {
  // Check for incoming serial commands ('start' or 'stop')
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    parseCommand(input);
  }

  // Must be called continuously in loop to execute position moves
  stepper1.run();
  stepper2.run();

  // Handle automated preprogrammed test sequence if active
  if (sequenceRunning) {
    updateTestSequence();
  }
}

// --- Movement Execution Helper ---
void executeMove(int motorId, float angle, unsigned long timeMs) {
  if (timeMs == 0) timeMs = 1;
  float timeSec = timeMs / 1000.0;
  long targetSteps = 0;
  float requiredSpeed = 0;

  // Wake up drivers (Active LOW)
  digitalWrite(M1_EN_PIN, LOW);
  digitalWrite(M2_EN_PIN, LOW);
  motorsActive = true;

  if (motorId == 1) {
    float stepsPerDegreeOutput = (float)(STEPS_PER_REV * MICROSTEPS * GEAR_REDUCTION_RATIO_1) / 360.0;
    targetSteps = round(angle * stepsPerDegreeOutput * CORRECTION_FACTOR_1);
    requiredSpeed = abs(targetSteps) / timeSec;
    if (requiredSpeed > 3000.0) requiredSpeed = 3000.0;

    stepper1.setMaxSpeed(requiredSpeed);
    stepper1.setAcceleration(requiredSpeed * 2.0);
    stepper1.move(targetSteps);
  } 
  else if (motorId == 2) {
    float stepsPerDegreeOutput = (float)(STEPS_PER_REV * MICROSTEPS * GEAR_REDUCTION_RATIO_2) / 360.0;
    targetSteps = round(angle * stepsPerDegreeOutput * CORRECTION_FACTOR_2);
    requiredSpeed = abs(targetSteps) / timeSec;
    if (requiredSpeed > 3000.0) requiredSpeed = 3000.0;

    stepper2.setMaxSpeed(requiredSpeed);
    stepper2.setAcceleration(requiredSpeed * 2.0);
    stepper2.move(targetSteps);
  }
}

// --- Test Sequence State Machine ---
void updateTestSequence() {
  switch (testState) {
    case STATE_IDLE:
      if (currentStepIndex < SEQUENCE_LENGTH) {
        TestStep step = testSequence[currentStepIndex];
        Serial.print("[Test Loop "); Serial.print(loopCount + 1);
        Serial.print("] Step "); Serial.print(currentStepIndex + 1);
        Serial.print(": Motor "); Serial.print(step.motorId);
        Serial.print(" -> "); Serial.print(step.angle); 
        Serial.print(" deg in "); Serial.print(step.moveTimeMs); Serial.println(" ms");
        
        executeMove(step.motorId, step.angle, step.moveTimeMs);
        testState = STATE_MOVING;
      } else {
        // Completed one full pass of the sequence array
        loopCount++;
        currentStepIndex = 0;
        
        if (TARGET_LOOPS > 0 && loopCount >= TARGET_LOOPS) {
          sequenceRunning = false;
          testState = STATE_IDLE;
          digitalWrite(M1_EN_PIN, HIGH);
          digitalWrite(M2_EN_PIN, HIGH);
          motorsActive = false;
          Serial.println("========================================");
          Serial.println(" Test Sequence Completed All Loops!     ");
          Serial.println("========================================");
          return;
        }
        Serial.println("--- Starting Next Loop Iteration ---");
      }
      break;

    case STATE_MOVING:
      // Wait until the motors finish stepping
      if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
        TestStep step = testSequence[currentStepIndex];
        if (step.pauseMs > 0) {
          pauseStartTime = millis();
          testState = STATE_PAUSING;
        } else {
          currentStepIndex++;
          testState = STATE_IDLE;
        }
      }
      break;

    case STATE_PAUSING:
      {
        TestStep step = testSequence[currentStepIndex];
        if (millis() - pauseStartTime >= step.pauseMs) {
          currentStepIndex++;
          testState = STATE_IDLE;
        }
      }
      break;
  }
}

// --- Serial Command Parser ('start' and 'stop') ---
void parseCommand(String command) {
  if (command.equalsIgnoreCase("start")) {
    if (!sequenceRunning) {
      sequenceRunning = true;
      currentStepIndex = 0;
      loopCount = 0;
      testState = STATE_IDLE;
      Serial.println("-> Test sequence started!");
    } else {
      Serial.println("-> Test sequence is already running.");
    }
  } 
  else if (command.equalsIgnoreCase("stop")) {
    sequenceRunning = false;
    testState = STATE_IDLE;
    stepper1.stop();
    stepper2.stop();
    digitalWrite(M1_EN_PIN, HIGH);
    digitalWrite(M2_EN_PIN, HIGH);
    motorsActive = false;
    Serial.println("-> Test sequence aborted. Motors stopped and put into idle state.");
  } 
  else {
    Serial.println("-> Unknown command. Use 'start' or 'stop'.");
  }
}