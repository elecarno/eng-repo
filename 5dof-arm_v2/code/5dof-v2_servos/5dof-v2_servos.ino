// MG996R ZEROING CODE

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// MG996R Settings
#define SERVOMIN  102 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  512 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2500 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define USMID  1500
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685...");

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);  

  delay(10);
  
  Serial.println("Zeroing all 16 servo channels to 90 degrees (1500us)...");
  
  // Loop through all 16 available channels on the PCA9685
  for (uint8_t i = 0; i < 16; i++) {
    pwm.writeMicroseconds(i, USMID);
  }

  Serial.println("All servos centered! Safe to attach servo horns now.");
}

void loop() {
  // We keep sending the center signal in the loop to hold the servos firmly in place 
  // while you physically work on your project and screw the horns down.
  for (uint8_t i = 0; i < 16; i++) {
    pwm.writeMicroseconds(i, USMID);
  }
  delay(500); 
}