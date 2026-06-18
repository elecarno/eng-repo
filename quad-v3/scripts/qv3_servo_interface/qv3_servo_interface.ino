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

// servo definitions
#define SERVO_FREQ  50   // analog servo ~50 Hz updates
#define USMIN       500  // minimum safe limit
#define USMAX      2500  // maximum safe limit
#define USMID      1500  // 90 degrees midpoint

// robot rest position definition
// SEE BELOW FOR MG996R TO PCA9685 WIRING \/\/\/
const uint16_t robotRestPose[12] = {
  USMIN,  // channel 0  FRONT LEFT LEG   hip
  USMAX,  // channel 1                   knee
  USMAX,  // channel 2                   ankle
  USMIN,  // channel 3  FRONT RIGHT LEG  hip
  USMAX,  // channel 4,                  knee
  USMAX,  // channel 5,                  ankle
  USMIN,  // channel 6, BACK LEFT LEG    hip
  USMAX,  // channel 7,                  knee
  USMAX,  // channel 8,                  ankle
  USMIN,  // channel 9, BACK RIGHT LEG   hip
  USMAX,  // channel 10,                 knee
  USMAX   // channel 11                  ankle
};

// array to track current configuration in memory
uint16_t currentPose[12];


// --- CODE -----------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); 
  delay(1000); 

  Serial.println("Initializing PCA9685 on ESP32...");
  Wire.begin();

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(10);

  Serial.println("Moving robot to its custom resting pose...");
  
  for (uint8_t i = 0; i < 12; i++) {
    currentPose[i] = robotRestPose[i];
    pwm.writeMicroseconds(i, currentPose[i]);
  }
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseAndMove(data);
  }
  
  for (uint8_t i = 0; i < 12; i++) {
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
    idx11 != -1
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
    int val10 = data.substring(idx10 + 3, idx11).toInt();
    int val11 = data.substring(idx11 + 3       ).toInt();

    currentPose[0]  = constrain(val0,  USMIN, USMAX);
    currentPose[1]  = constrain(val1,  USMIN, USMAX);
    currentPose[2]  = constrain(val2,  USMIN, USMAX);
    currentPose[3]  = constrain(val3,  USMIN, USMAX);
    currentPose[4]  = constrain(val4,  USMIN, USMAX);
    currentPose[5]  = constrain(val5,  USMIN, USMAX);
    currentPose[6]  = constrain(val6,  USMIN, USMAX);
    currentPose[7]  = constrain(val7,  USMIN, USMAX);
    currentPose[8]  = constrain(val8,  USMIN, USMAX);
    currentPose[9]  = constrain(val9,  USMIN, USMAX);
    currentPose[10] = constrain(val10, USMIN, USMAX);
    currentPose[11] = constrain(val11, USMIN, USMAX);
  }
}