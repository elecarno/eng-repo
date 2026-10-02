#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Standard 50Hz refresh rate

// -------------------------------------------------------------------
// Servo Calibration Limits (in microseconds)
// -------------------------------------------------------------------

// Miuzei 20kg Digital Servo Specs (Channels 0 & 1) - 270° Range
#define MIUZEI_MIN  500   // 0° pulse width
#define MIUZEI_90   1166  // 90° pulse width  (500 + (2000 * 90 / 270))
#define MIUZEI_180  1833  // 180° pulse width (500 + (2000 * 180 / 270))
#define MIUZEI_MAX  2500  // 270° max pulse width

// MG996R Analog Servo Specs (Channels 2 to 6) - 180° Range
#define MG996R_MIN  500   // 0° pulse width
#define MG996R_MID  1450  // 90° center pulse width
#define MG996R_MAX  2400  // 180° max pulse width

/**
 * Converts microsecond pulse width to PCA9685 12-bit PWM ticks (0-4095).
 */
uint16_t usToTicks(uint16_t microseconds) {
  return (uint16_t)(((uint32_t)microseconds * 4096) / 20000);
}

/**
 * Sets a target servo channel (0 to 6) to its respective allowed angles.
 */
void setServoPosition(uint8_t channel, uint16_t angle) {
  if (channel > 6) {
    Serial.println("Error: Channel out of range (0-6 only).");
    return;
  }

  uint16_t pulseUS = 0;
  const char* typeStr = "";

  if (channel <= 1) {
    // Channels 0 & 1: Miuzei 20kg (270° Range)
    typeStr = "Miuzei 270°";
    switch (angle) {
      case 0:   pulseUS = MIUZEI_MIN; break;
      case 90:  pulseUS = MIUZEI_90;  break;
      case 180: pulseUS = MIUZEI_180; break;
      case 270: pulseUS = MIUZEI_MAX; break;
      default:
        Serial.println("Invalid angle for Miuzei! Allowed: 0, 90, 180, or 270.");
        return;
    }
  } else {
    // Channels 2 to 6: MG996R (180° Range)
    typeStr = "MG996R 180°";
    switch (angle) {
      case 0:   pulseUS = MG996R_MIN; break;
      case 90:  pulseUS = MG996R_MID; break;
      case 180: pulseUS = MG996R_MAX; break;
      default:
        Serial.println("Invalid angle for MG996R! Allowed: 0, 90, or 180.");
        return;
    }
  }

  uint16_t ticks = usToTicks(pulseUS);
  pwm.setPWM(channel, 0, ticks);

  Serial.print("Ch ");
  Serial.print(channel);
  Serial.print(" [");
  Serial.print(typeStr);
  Serial.print("] -> ");
  Serial.print(angle);
  Serial.print("° (");
  Serial.print(pulseUS);
  Serial.print(" µs / ");
  Serial.print(ticks);
  Serial.println(" ticks)");
}

void setup() {
  Serial.begin(9600);
  Serial.println("==========================================");
  Serial.println("   7-Axis Arm Interactive Control Setup   ");
  Serial.println("==========================================");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  // Initialize all channels to neutral position
  Serial.println("Initializing channels to neutral centers...");
  setServoPosition(0, 90);
  setServoPosition(1, 90);
  for (uint8_t ch = 2; ch <= 6; ch++) {
    setServoPosition(ch, 90);
  }

  printInstructions();
}

void loop() {
  if (Serial.available() > 0) {
    int channel = Serial.parseInt();
    int angle = Serial.parseInt();

    while (Serial.available() > 0 && Serial.peek() < '0') {
      Serial.read();
    }

    setServoPosition(channel, angle);
  }
}

void printInstructions() {
  Serial.println("\n--- Serial Monitor Controls ---");
  Serial.println("Format: <channel> <angle>\n");
  Serial.println("Channels 0 & 1 (Miuzei 270°):");
  Serial.println("  Allowed Angles: 0, 90, 180, or 270");
  Serial.println("  Examples: '0 270', '1 90'\n");
  Serial.println("Channels 2 to 6 (MG996R 180°):");
  Serial.println("  Allowed Angles: 0, 90, or 180");
  Serial.println("  Examples: '2 180', '6 0'\n");
}