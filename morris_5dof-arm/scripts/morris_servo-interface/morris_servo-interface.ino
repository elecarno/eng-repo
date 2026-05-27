// MORRIS SERVO INTERFACE CODE
// This is the code that is uploaded to Morris' ESP32 microcontroller. The code allows for
// Morris to take in serial inputs in a specified format and set each servo to the positions
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
const uint16_t robotRestPose[5] = {
  USMID,  // channel 0 (joint 1 / base)
  USMIN,  // channel 1 (joint 2 / shoulder)
  USMIN,  // channel 2 (joint 3 / elbow)
  USMID,  // channel 3 (joint 4 / wrist)
  USMID   // channel 4 (joint 5 / cuff)
};

// array to track current configuration in memory
uint16_t currentPose[5];


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
  
  for (uint8_t i = 0; i < 5; i++) {
    currentPose[i] = robotRestPose[i];
    pwm.writeMicroseconds(i, currentPose[i]);
  }
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseAndMove(data);
  }
  
  for (uint8_t i = 0; i < 5; i++) {
    pwm.writeMicroseconds(i, currentPose[i]);
  }
  delay(20); 
}

void parseAndMove(String data) {
  int idx0 = data.indexOf("C0:");
  int idx1 = data.indexOf("C1:");
  int idx2 = data.indexOf("C2:");
  int idx3 = data.indexOf("C3:");
  int idx4 = data.indexOf("C4:");

  if (idx0 != -1 && idx1 != -1 && idx2 != -1 && idx3 != -1 && idx4 != -1) {
    
    int val0 = data.substring(idx0 + 3, idx1).toInt();
    int val1 = data.substring(idx1 + 3, idx2).toInt();
    int val2 = data.substring(idx2 + 3, idx3).toInt();
    int val3 = data.substring(idx3 + 3, idx4).toInt();
    int val4 = data.substring(idx4 + 3).toInt();

    currentPose[0] = constrain(val0, USMIN, USMAX);
    currentPose[1] = constrain(val1, USMIN, USMAX);
    currentPose[2] = constrain(val2, USMIN, USMAX);
    currentPose[3] = constrain(val3, USMIN, USMAX);
    currentPose[4] = constrain(val4, USMIN, USMAX);
  }
}