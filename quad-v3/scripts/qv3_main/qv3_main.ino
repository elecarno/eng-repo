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

enum dDisplaySide { D_LEFT = 0, D_RIGHT = 1 };

// SERVOS (s-) --------------------------------------------------------
// driver
Adafruit_PWMServoDriver sPWM = Adafruit_PWMServoDriver();

// servo definitions
const int sSERVO_FREQ = 50;   // analog servo ~50 Hz updates
// MG996R limits (channels 0 - 11)
const int sUSMIN_MG996R = 500;   // minimum safe limit for MG996R
const int sUSMAX_MG996R = 2500;  // maximum safe limit for MG996R
// MG90S limits (channels 12 & 13)
const int sUSMIN_MG90S = 600;   // minimum safe limit for MG90S
const int sUSMAX_MG90S = 2400;  // maximum safe limit for MG90S

// rest position definition
const uint16_t sRestPose[14] = {
  1000,  // channel 0  FRONT LEFT LEG   hip
  1764,  // channel 1                   knee
  1735,  // channel 2                   ankle

  2000,  // channel 3  FRONT RIGHT LEG  hip
  1235,  // channel 4,                  knee
  1264,  // channel 5,                  ankle

  2000,  // channel 6, BACK LEFT LEG    hip
  1235,  // channel 7,                  knee
  1264,  // channel 8,                  ankle

  1000,  // channel 9, BACK RIGHT LEG   hip
  1764,  // channel 10,                 knee
  1735,  // channel 11                  ankle

  sUSMIN_MG90S,   // channel 12 - LEFT  ANTENNA  (MG90S)
  sUSMAX_MG90S    // channel 13 - RIGHT ANTENNA  (MG90S)
};

// array to track current configuration in memory
uint16_t sCurrentPose[14];

void sParseAndMove(String data) {
  int idx0  = data.indexOf("C0:");
  int idx1  = data.indexOf("C1:");
  int idx2  = data.indexOf("C2:");
  int idx3  = data.indexOf("C3:");
  int idx4  = data.indexOf("C4:");
  int idx5  = data.indexOf("C5:");
  int idx6  = data.indexOf("C6:");
  int idx7  = data.indexOf("C7:");
  int idx8  = data.indexOf("C8:");
  int idx9  = data.indexOf("C9:");
  int idx10 = data.indexOf("C10:");
  int idx11 = data.indexOf("C11:");
  int idx12 = data.indexOf("C12:");
  int idx13 = data.indexOf("C13:");

  if (
    idx0  != -1 &&
    idx1  != -1 &&
    idx2  != -1 &&
    idx3  != -1 &&
    idx4  != -1 &&
    idx5  != -1 &&
    idx6  != -1 &&
    idx7  != -1 &&
    idx8  != -1 &&
    idx9  != -1 &&
    idx10 != -1 &&
    idx11 != -1 &&
    idx12 != -1 &&
    idx13 != -1
    ) {
    
    int val0  = data.substring(idx0  + 3, idx1 ).toInt();
    int val1  = data.substring(idx1  + 3, idx2 ).toInt();
    int val2  = data.substring(idx2  + 3, idx3 ).toInt();
    int val3  = data.substring(idx3  + 3, idx4 ).toInt();
    int val4  = data.substring(idx4  + 3, idx5 ).toInt();
    int val5  = data.substring(idx5  + 3, idx6 ).toInt();
    int val6  = data.substring(idx6  + 3, idx7 ).toInt();
    int val7  = data.substring(idx7  + 3, idx8 ).toInt();
    int val8  = data.substring(idx8  + 3, idx9 ).toInt();
    int val9  = data.substring(idx9  + 3, idx10).toInt();
    int val10 = data.substring(idx10 + 4, idx11).toInt();
    int val11 = data.substring(idx11 + 4, idx12).toInt();
    int val12 = data.substring(idx12 + 4, idx13).toInt();
    int val13 = data.substring(idx13 + 4       ).toInt();

    // constrain MG996R legs
    sCurrentPose[0]  = constrain(val0,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[1]  = constrain(val1,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[2]  = constrain(val2,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[3]  = constrain(val3,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[4]  = constrain(val4,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[5]  = constrain(val5,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[6]  = constrain(val6,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[7]  = constrain(val7,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[8]  = constrain(val8,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[9]  = constrain(val9,  sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[10] = constrain(val10, sUSMIN_MG996R, sUSMAX_MG996R);
    sCurrentPose[11] = constrain(val11, sUSMIN_MG996R, sUSMAX_MG996R);
    
    // constrain MG90S antennae
    sCurrentPose[12] = constrain(val12, sUSMIN_MG90S, sUSMAX_MG90S);
    sCurrentPose[13] = constrain(val13, sUSMIN_MG90S, sUSMAX_MG90S);
  }
}


// SENSORS (e- )-------------------------------------------------------
const int ePIN_TRIG = 12;
const int ePIN_ECHO[] = {14, 27, 26}; 
const char* eSENSOR_LABELS[] = {"right", "middle", "left"};
const int eSENSOR_COUNT = 3;


// MATRIX DISPLAYS (d- )-----------------------------------------------
#define dHARDWARE_TYPE MD_MAX72XX::FC16_HW
const int dMAX_DEVICES = 2;

// ESP32 hardware VSPI pins
const int dCLK_PIN  = 18;
const int dDATA_PIN = 23;
const int dCS_PIN   = 5;
// initialize using hardware SPI
MD_MAX72XX mx = MD_MAX72XX(dHARDWARE_TYPE, dCS_PIN, dMAX_DEVICES);

// patterns - make using https://xantorohara.github.io/led-matrix-editor/
const uint8_t dpLeft[8] = {
  0b00000000, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00111100, 0b00000000
};

const uint8_t dpRight[8] = {
  0b00000000, 0b00011100, 0b00100100, 0b00100100, 0b00011100, 0b00100100, 0b00100100, 0b00000000
};

const uint8_t dpSmiley[8] = {
  0b00111100, 0b01000010, 0b10100101, 0b10000001, 0b10100101, 0b10011001, 0b01000010, 0b00111100
};

const uint8_t dpHeart[8] = {
  0b00000000, 0b01100110,  0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000, 0b00000000       
};

const uint8_t dpFull[8] = {
  0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111
};

const uint8_t dpBored[8] = {
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111      
};

const uint8_t dpHappy[8] = {
  0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11000011, 0b10000001, 0b00000000, 0b00000000     
};

void dDrawMatrixPattern(enum dDisplaySide side, const uint8_t pattern[]) {
  uint8_t startCol = side * 8; // set offset needed

  mx.control((uint8_t)side, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (uint8_t col = 0; col < 8; col++) {
    mx.setColumn(startCol + col, pattern[col]); 
  }
  mx.control((uint8_t)side, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}


// BUZZER (b- )--------------------------------------------------------
const int bPIN_BUZZER = 25;

void bPlayTone(double frequency, int duration) {
  if (frequency == 0) {
    ledcWriteTone(bPIN_BUZZER, 0);         // stop tone
  } else {
    ledcWriteTone(bPIN_BUZZER, frequency); // start tone directly on PIN
  }
  delay(duration);                        // hold tone
  ledcWriteTone(bPIN_BUZZER, 0);           // stop tone
  delay(50);                              // pause between tones
}


// SETUP -----------------------------------------------------------------------------------------------------------------------
void setup() {

  // SERVOS -----------------------------------------------------------
  Serial.begin(115200); 
  delay(1000); 

  Serial.println("Initializing PCA9685 on ESP32...");
  
  // pass ESP32 standard I2C pins 
  Wire.begin(21, 22);

  sPWM.begin();
  sPWM.setOscillatorFrequency(25000000);
  sPWM.setPWMFreq(sSERVO_FREQ);  
  delay(10);

  Serial.println("Moving robot to its custom resting pose...");
  
  for (uint8_t i = 0; i < 14; i++) {
    sCurrentPose[i] = sRestPose[i];
    sPWM.writeMicroseconds(i, sCurrentPose[i]);
  }

  // SENSORS ----------------------------------------------------------
  pinMode(ePIN_TRIG, OUTPUT);
  for (int i = 0; i < eSENSOR_COUNT; i++) {
    pinMode(ePIN_ECHO[i], INPUT);
  }

  // MATRIX DISPLAYS --------------------------------------------------
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2); // Set brightness low (0-15)
  mx.clear();

  // --- STARTUP TEST ---
  dDrawMatrixPattern(D_LEFT, dpLeft);
  delay(1000);
  dDrawMatrixPattern(D_RIGHT, dpRight);
  delay(1000);

  mx.clear(); // clear everything for main loop
  delay(1000);

  // BUZZER -----------------------------------------------------------
  ledcAttach(bPIN_BUZZER, 2000, 8);
}

// MASTER LOOP ----------------------------------------------------------------------------------------------------------------
void loop() {

  // SERVOS -----------------------------------------------------------
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    sParseAndMove(data);
  }
  
  for (uint8_t i = 0; i < 14; i++) {
    sPWM.writeMicroseconds(i, sCurrentPose[i]);
  }

  // SENSORS ----------------------------------------------------------
  for (int i = 0; i < eSENSOR_COUNT; i++) {
    digitalWrite(ePIN_TRIG, LOW);
    delayMicroseconds(5); // Increased stability delay
    
    digitalWrite(ePIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ePIN_TRIG, LOW);
    
    long duration = pulseIn(ePIN_ECHO[i], HIGH, 25000); 
    float distance = (duration * 0.0343) / 2;
    
    Serial.print("Sensor ");
    Serial.print(eSENSOR_LABELS[i]); 
    Serial.print(": ");
    
    // if (distance == 0) {
    //   Serial.println("Out of range");
    // } else {
    //   Serial.print(distance);
    //   Serial.println(" cm");
    // }
    
    // Give a generous 80ms for the power to stabilize and sound to fade
    // delay(80); 
  }

  // MATRIX DISPLAYS --------------------------------------------------
  dDrawMatrixPattern(D_LEFT,  dpFull);
  dDrawMatrixPattern(D_RIGHT, dpFull);

  // BUZZER -----------------------------------------------------------
  // bPlayTone(500, 50);
  // bPlayTone(800, 50);
  // bPlayTone(600, 50);
  // bPlayTone(500, 50);
  // bPlayTone(700, 50);

  // Serial.println("-----------------------");
  delay(20);
}
