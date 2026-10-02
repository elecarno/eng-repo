#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Standard 50Hz refresh rate for MG996R

// Calibrated MG996R Pulse Width Specifications (in microseconds)
#define USMIN  500   // Microseconds for 0°
#define USMID  1450  // Microseconds for 90° (Midpoint between 500 and 2400)
#define USMAX  2400  // Microseconds for 180° (Max supported limit)

/**
 * Converts microsecond pulse width to PCA9685 12-bit PWM ticks (0-4095).
 * PCA9685 period at 50 Hz = 20,000 microseconds across 4096 counts.
 */
uint16_t usToTicks(uint16_t microseconds) {
  return (uint16_t)(((uint32_t)microseconds * 4096) / 20000);
}

/**
 * Sets an MG996R channel to 0, 90, or 180 degrees using calibrated limits.
 * @param channel PCA9685 channel (0 to 5)
 * @param angle Target angle (0, 90, or 180)
 */
void setServoPosition(uint8_t channel, uint16_t angle) {
  if (channel > 5) {
    Serial.println("Error: Channel out of range (0-5 only).");
    return;
  }

  uint16_t pulseUS;

  switch (angle) {
    case 0:
      pulseUS = USMIN;
      break;
    case 90:
      pulseUS = USMID;
      break;
    case 180:
      pulseUS = USMAX;
      break;
    default:
      Serial.println("Invalid angle! Allowed angles: 0, 90, or 180.");
      return;
  }

  uint16_t ticks = usToTicks(pulseUS);
  pwm.setPWM(channel, 0, ticks);

  Serial.print("MG996R Ch ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.print("° (");
  Serial.print(pulseUS);
  Serial.print(" µs / ");
  Serial.print(ticks);
  Serial.println(" ticks)");
}

void setup() {
  Serial.begin(9600);
  Serial.println("--- Calibrated MG996R 6-Servo Controller ---");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  // Initialize channels 0 through 5 to 90 degrees
  Serial.println("Setting channels 0-5 to center (90 degrees)...");
  for (uint8_t ch = 0; ch < 6; ch++) {
    setServoPosition(ch, 90);
  }

  Serial.println("\nCommands format: <channel> <angle>");
  Serial.println("Example inputs:");
  Serial.println("  0 180   -> Move Servo 0 to 180°");
  Serial.println("  3 0     -> Move Servo 3 to 0°");
  Serial.println("  5 90    -> Move Servo 5 to 90°\n");
}

void loop() {
  if (Serial.available() > 0) {
    int channel = Serial.parseInt();
    int angle = Serial.parseInt();

    // Clear any leftover characters in serial buffer
    while (Serial.available() > 0 && Serial.peek() < '0') {
      Serial.read();
    }

    setServoPosition(channel, angle);
  }
}