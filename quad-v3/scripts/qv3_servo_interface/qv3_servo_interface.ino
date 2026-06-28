// STEVEN SERVO INTERFACE CODE
// This is the code that is uploaded to Steven's ESP32 microcontroller. The code allows for
// Steven to take in serial inputs in a specified format and set each servo to the positions
// specified in that input.

// PCA9685 to ESP32 wiring
// SDA -> GPIO 21
// SCL -> GPIO 22


// --- INCLUDES -------------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// --- GLOBALS --------------------------------------------------------------------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo definitions
#define SERVO_FREQ      50   // Analog servo ~50 Hz updates

// MG996R Limits (Channels 0 - 11)
#define USMIN_MG996R    500  // Minimum safe limit for MG996R
#define USMAX_MG996R   2500  // Maximum safe limit for MG996R

// MG90S Limits (Channels 12 & 13)
#define USMIN_MG90S     600  // Minimum safe limit for MG90S
#define USMAX_MG90S    2400  // Maximum safe limit for MG90S

// Robot rest position definition
const uint16_t robotRestPose[14] = {
  USMIN_MG996R,  // channel 0  FRONT LEFT LEG   hip
  USMAX_MG996R,  // channel 1                   knee
  USMAX_MG996R,  // channel 2                   ankle

  USMAX_MG996R,  // channel 3  FRONT RIGHT LEG  hip
  USMIN_MG996R,  // channel 4,                  knee
  USMIN_MG996R,  // channel 5,                  ankle

  USMAX_MG996R,  // channel 6, BACK LEFT LEG    hip
  USMIN_MG996R,  // channel 7,                  knee
  USMIN_MG996R,  // channel 8,                  ankle

  USMIN_MG996R,  // channel 9, BACK RIGHT LEG   hip
  USMAX_MG996R,  // channel 10,                 knee
  USMAX_MG996R,  // channel 11                  ankle

  USMIN_MG90S,   // channel 12 - LEFT  ANTENNA  (MG90S)
  USMAX_MG90S    // channel 13 - RIGHT ANTENNA  (MG90S)
};

// Array to track current configuration in memory
uint16_t currentPose[14];


// --- CODE -----------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); 
  delay(1000); 

  Serial.println("Initializing PCA9685 on ESP32...");
  
  // Explicitly passing ESP32 standard I2C pins 
  Wire.begin(21, 22);

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(10);

  Serial.println("Moving robot to its custom resting pose...");
  
  for (uint8_t i = 0; i < 14; i++) {
    currentPose[i] = robotRestPose[i];
    pwm.writeMicroseconds(i, currentPose[i]);
  }
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseAndMove(data);
  }
  
  for (uint8_t i = 0; i < 14; i++) {
    pwm.writeMicroseconds(i, currentPose[i]);
  }
  delay(20); 
}

void parseAndMove(String data) {
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
    currentPose[0]  = constrain(val0,  USMIN_MG996R, USMAX_MG996R);
    currentPose[1]  = constrain(val1,  USMIN_MG996R, USMAX_MG996R);
    currentPose[2]  = constrain(val2,  USMIN_MG996R, USMAX_MG996R);
    currentPose[3]  = constrain(val3,  USMIN_MG996R, USMAX_MG996R);
    currentPose[4]  = constrain(val4,  USMIN_MG996R, USMAX_MG996R);
    currentPose[5]  = constrain(val5,  USMIN_MG996R, USMAX_MG996R);
    currentPose[6]  = constrain(val6,  USMIN_MG996R, USMAX_MG996R);
    currentPose[7]  = constrain(val7,  USMIN_MG996R, USMAX_MG996R);
    currentPose[8]  = constrain(val8,  USMIN_MG996R, USMAX_MG996R);
    currentPose[9]  = constrain(val9,  USMIN_MG996R, USMAX_MG996R);
    currentPose[10] = constrain(val10, USMIN_MG996R, USMAX_MG996R);
    currentPose[11] = constrain(val11, USMIN_MG996R, USMAX_MG996R);
    
    // constrain MG90S antennae
    currentPose[12] = constrain(val12, USMIN_MG90S, USMAX_MG90S);
    currentPose[13] = constrain(val13, USMIN_MG90S, USMAX_MG90S);
  }
}