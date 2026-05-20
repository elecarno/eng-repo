// MG996R 6DOF PC SERIAL CONTROLLER
// SDA -> GPIO 21
// SCL -> GPIO 22

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ  50   // Analog servos run at ~50 Hz updates
#define USMIN       500  // Absolute minimum safe limit
#define USMAX      2500  // Absolute maximum safe limit

#define USMID      1500  // 90 degrees reference

// --- ROBOT REST CONFIGURATION ---
const uint16_t robotRestPose[6] = {
  USMID,  // Channel 0 (joint 1 / base)
  USMIN,  // Channel 1 (joint 2 / shoulder)
  USMIN,  // Channel 2 (joint 3 / elbow1)
  USMAX,  // Channel 3 (joint 4 / elbow2)
  USMID,  // Channel 4 (joint 5 / wrist1)
  USMID   // Channel 5 (joint 6 / wrist2)
};

// Array to track current positions in memory
uint16_t currentPose[6];

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
  
  // Set initial positions to your home configuration
  for (uint8_t i = 0; i < 6; i++) {
    currentPose[i] = robotRestPose[i];
    pwm.writeMicroseconds(i, currentPose[i]);
  }
}

void loop() {
  // Check if PC sent a new position string
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseAndMove(data);
  }
  
  // Continuously refresh the driver with current targets to maintain holding torque
  for (uint8_t i = 0; i < 6; i++) {
    pwm.writeMicroseconds(i, currentPose[i]);
  }
  delay(20); // Small delay to prevent spamming the I2C bus too violently
}

void parseAndMove(String data) {
  // Expected format: "C0:1500,C1:2500,C2:500,C3:2500,C4:1500,C5:1500"
  int idx0 = data.indexOf("C0:");
  int idx1 = data.indexOf("C1:");
  int idx2 = data.indexOf("C2:");
  int idx3 = data.indexOf("C3:");
  int idx4 = data.indexOf("C4:");
  int idx5 = data.indexOf("C5:");

  if (idx0 != -1 && idx1 != -1 && idx2 != -1 && idx3 != -1 && idx4 != -1 && idx5 != -1) {
    // Extract strings and parse integers
    int val0 = data.substring(idx0 + 3, idx1).toInt();
    int val1 = data.substring(idx1 + 3, idx2).toInt();
    int val2 = data.substring(idx2 + 3, idx3).toInt();
    int val3 = data.substring(idx3 + 3, idx4).toInt();
    int val4 = data.substring(idx4 + 3, idx5).toInt();
    int val5 = data.substring(idx5 + 3).toInt();

    // Constrain to safe physical ranges and update tracking array
    currentPose[0] = constrain(val0, USMIN, USMAX);
    currentPose[1] = constrain(val1, USMIN, USMAX);
    currentPose[2] = constrain(val2, USMIN, USMAX);
    currentPose[3] = constrain(val3, USMIN, USMAX);
    currentPose[4] = constrain(val4, USMIN, USMAX);
    currentPose[5] = constrain(val5, USMIN, USMAX);
  }
}