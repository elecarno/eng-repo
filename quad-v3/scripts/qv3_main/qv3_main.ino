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
  sUSMIN_MG996R,  // channel 0  FRONT LEFT LEG   hip
  sUSMAX_MG996R,  // channel 1                   knee
  sUSMAX_MG996R,  // channel 2                   ankle
  sUSMIN_MG996R,  // channel 3  FRONT RIGHT LEG  hip
  sUSMAX_MG996R,  // channel 4,                  knee
  sUSMAX_MG996R,  // channel 5,                  ankle
  sUSMIN_MG996R,  // channel 6, BACK LEFT LEG    hip
  sUSMAX_MG996R,  // channel 7,                  knee
  sUSMAX_MG996R,  // channel 8,                  ankle
  sUSMIN_MG996R,  // channel 9, BACK RIGHT LEG   hip
  sUSMAX_MG996R,  // channel 10,                 knee
  sUSMAX_MG996R,  // channel 11                  ankle
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
const char* eSENSOR_LABELS[] = {"left", "middle", "right"};
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

enum dDisplaySide { D_LEFT = 0, D_RIGHT = 1 };


// BUZZER (b- )--------------------------------------------------------
const int bBuzzerPin = 25;

void bPlayTone(double frequency, int duration) {
  if (frequency == 0) {
    ledcWriteTone(bBuzzerPin, 0);         // stop tone
  } else {
    ledcWriteTone(bBuzzerPin, frequency); // start tone directly on PIN
  }
  delay(duration);                        // hold tone
  ledcWriteTone(bBuzzerPin, 0);           // stop tone
  delay(50);                              // pause between tones
}


// SETUP -----------------------------------------------------------------------------------------------------------------------
void setup() {

  // SERVOS (----------------------------------------------------------
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
}

// MASTER LOOP ----------------------------------------------------------------------------------------------------------------
void loop() {

}
