// DESCRIPTION ----------------------------------------------------------------------------------------------------------------
// This is the main hardware script for the Steven robot
// It has four functions:
// 1. Servo control
// 2. Sensor control & data processing
// 3. Matrix display control
// 4. Buzzer control


// INCLUDES -------------------------------------------------------------------------------------------------------------------
// MAX7219 8x8 Matrix Displays
#include <MD_MAX72xx.h>
#include <SPI.h>
// PCA9685 Servo Driver
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// GLOBALS & FUNCTIONS --------------------------------------------------------------------------------------------------------
enum DisplaySide { D_LEFT = 0, D_RIGHT = 1 }; // define display side enum at top to prevent compiler error

// COOPERATIVE SCHEDULER ----------------------------------------------
// function pointer type for scheduler tasks
typedef void (*TaskFunction)();

struct Task {
    TaskFunction run;           // function to execute
    unsigned long interval;     // how often to run (in milliseconds)
    unsigned long lastRun;      // timestamp of last execution
};

// predefine task functions
void taskServos();
void taskSensors();
void taskDisplays();
void taskBuzzer();

Task tasks[] = {
  {taskServos,    20,   0},
  {taskSensors,   30,   0},
  {taskDisplays,  300,  0},
  {taskBuzzer,    500,  0}
};

const int tasksCount = sizeof(tasks) / sizeof(Task);


// SERVOS -------------------------------------------------------------
// TO-DO
void taskServos () {
  
}


// SENSORS ------------------------------------------------------------
const int TRIG_PIN = 12;
const int ECHO_PINS[] = {14, 27, 26}; 
const char* SENSOR_LABELS[] = {"Left", "Middle", "Right"};
const int SENSOR_COUNT = 3;

int currentSensorIndex = 0;

void taskSensors() {
  // 1. Trigger a clean pulse on the shared TRIG pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2); // Microsecond delays are okay! They don't hurt the scheduler.
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 2. Read the specific echo pin for the CURRENT sensor
  // We dropped the timeout to 10000 microseconds (~170 cm max distance) 
  // so pulseIn() won't hang the CPU if there's no obstacle.
  long duration = pulseIn(ECHO_PINS[currentSensorIndex], HIGH, 10000); 
  float distance = (duration * 0.0343) / 2;
  
  // 3. Print the data out
  Serial.print("Sensor ");
  Serial.print(SENSOR_LABELS[currentSensorIndex]); 
  Serial.print(": ");
  
  if (distance == 0) {
    Serial.println("Out of range");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  // 4. Cycle to the NEXT sensor index for the next time this task runs
  currentSensorIndex++;
  if (currentSensorIndex >= SENSOR_COUNT) {
    currentSensorIndex = 0; // Loop back to the Left sensor
  }
  
  // Notice: ZERO delay() calls here! The 30ms scheduler interval 
  // completely replaces the old 80ms delay, letting acoustic echoes fade safely.
}


// MATRIX DISPLAYS ----------------------------------------------------
#define dHARDWARE_TYPE MD_MAX72XX::FC16_HW
const int DISPLAYS_MAX_DEVICES = 2;

// ESP32 hardware VSPI pins
const int DISPLAYS_CLK_PIN  = 18;
const int DISPLAYS_DATA_PIN = 23;
const int DISPLAYS_CS_PIN   = 5;
// initialize using hardware SPI
MD_MAX72XX mx = MD_MAX72XX(dHARDWARE_TYPE, DISPLAYS_CS_PIN, DISPLAYS_MAX_DEVICES);

// patterns - make using https://xantorohara.github.io/led-matrix-editor/
const uint8_t patternLeft[8] = {
  0b00000000, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00111100, 0b00000000
};

const uint8_t patternRight[8] = {
  0b00000000, 0b00011100, 0b00100100, 0b00100100, 0b00011100, 0b00100100, 0b00100100, 0b00000000
};

const uint8_t patternSmiley[8] = {
  0b00111100, 0b01000010, 0b10100101, 0b10000001, 0b10100101, 0b10011001, 0b01000010, 0b00111100
};

const uint8_t patternHeart[8] = {
  0b00000000, 0b01100110,  0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000, 0b00000000       
};

const uint8_t patternFull[8] = {
  0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111
};

const uint8_t patternBored[8] = {
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111      
};

const uint8_t patternHappy[8] = {
  0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11000011, 0b10000001, 0b00000000, 0b00000000     
};

void displayDrawMatrixPattern(enum DisplaySide side, const uint8_t pattern[]) {
  uint8_t deviceIndex = (uint8_t)side; // 0 for left, 1 for right

  // disable auto-wrap update for this specific hardware module
  mx.control(deviceIndex, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  
  for (uint8_t col = 0; col < 8; col++) {
    // correct library format: setColumn(device, column_within_device, value)
    mx.setColumn(deviceIndex, col, pattern[col]); 
  }
  
  mx.control(deviceIndex, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void taskDisplays () {
  displayDrawMatrixPattern(D_LEFT, patternFull);
  displayDrawMatrixPattern(D_RIGHT, patternHeart);
}


// BUZZER -------------------------------------------------------------
const int BUZZER_PIN = 25;
unsigned long buzzerTargetTime = 0;
bool buzzerPlaying = false;

void requestBuzzerTone(double frequency, int duration) {
  if (frequency > 0) {
    ledcWriteTone(BUZZER_PIN, frequency);
    buzzerTargetTime = millis() + duration;
    buzzerPlaying = true;
  }
}

void taskBuzzer() {
  // check if a running tone has finished duration
  if (buzzerPlaying && millis() >= buzzerTargetTime) {
    ledcWriteTone(BUZZER_PIN, 0); // Stop tone safely without delaying
    buzzerPlaying = false;
  }
  
  // add "melodies" here
}


// SETUP -----------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // SENSORS ----------------------------------------------------------
  pinMode(TRIG_PIN, OUTPUT);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(ECHO_PINS[i], INPUT);
  }

  // MATRIX DISPLAYS --------------------------------------------------
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2); // Set brightness low (0-15)
  mx.clear();

  // --- STARTUP TEST ---
  displayDrawMatrixPattern(D_LEFT, patternLeft);
  delay(500);
  displayDrawMatrixPattern(D_RIGHT, patternRight);
  delay(500);

  mx.clear(); // clear everything for main loop
  delay(500);

  // BUZZER -----------------------------------------------------------
  ledcAttach(BUZZER_PIN, 2000, 8);

  // SERVOS -----------------------------------------------------------
  delay(500);
}

// MASTER LOOP ----------------------------------------------------------------------------------------------------------------
void loop() {
  unsigned long timeCurrent = millis();

  // cooperative scheduler Loop
  for (int i = 0; i < tasksCount; i++) {
      // check if required interval has passed
      if (timeCurrent - tasks[i].lastRun >= tasks[i].interval) {
          tasks[i].run();                 // run task
          tasks[i].lastRun = timeCurrent; // update the execution timestamp
      }
  }
}
