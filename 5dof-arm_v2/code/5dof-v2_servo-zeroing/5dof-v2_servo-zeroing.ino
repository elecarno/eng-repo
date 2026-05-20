// MG996R CUSTOM REST CONFIGURATION CODE

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// MG996R Calibration Limits
#define SERVOMIN   102   // 'minimum' pulse length count (out of 4096)
#define SERVOMAX   512   // 'maximum' pulse length count (out of 4096)
#define USMIN      500   // 0 degrees
#define USMID     1500   // 90 degrees
#define USMAX     2500   // 180 degrees
#define SERVO_FREQ  50   // Analog servos run at ~50 Hz updates

// --- ROBOT REST CONFIGURATION ---
// Change these to USMIN, USMID, or USMAX to set the unique rest state for each servo channel (0 to 5)
const uint16_t robotRestPose[6] = {
  USMID,  // Channel 0 (joint 1 / base)
  USMAX,  // Channel 1 (joint 2 / shoulder)
  USMIN,  // Channel 2 (joint 3 / elbow1)
  USMAX,  // Channel 3 (joint 4 / elbow2)
  USMID,  // Channel 4 (joint 5 / wrist1)
  USMID   // Channel 5 (joint 6 / wrist2)
};

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
  
  // Set your 6 active servos to their designated starting positions
  for (uint8_t i = 0; i < 6; i++) {
    pwm.writeMicroseconds(i, robotRestPose[i]);
  }
}

void loop() {
  // Hold the 6 active servos securely in their rest positions
  for (uint8_t i = 0; i < 6; i++) {
    pwm.writeMicroseconds(i, robotRestPose[i]);
  }
  delay(500); 
}