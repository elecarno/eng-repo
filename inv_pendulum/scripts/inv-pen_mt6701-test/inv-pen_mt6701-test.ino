#include <Arduino.h>

const int ANALOG_PIN = 34; // Connect to MT6701 'Analog / PWM' pin

void setup() {
  Serial.begin(115200);
  
  // Set ADC resolution to 12-bit (0 - 4095)
  analogReadResolution(12);
  
  // Attenuation for 0V - 3.3V range
  analogSetPinAttenuation(ANALOG_PIN, ADC_11db); 
  
  Serial.println("MT6701 Analog Angle Reader Started...");
}

void loop() {
  // Take 16 readings to average out breadboard noise
  uint32_t rawSum = 0;
  for (int i = 0; i < 16; i++) {
    rawSum += analogRead(ANALOG_PIN);
    delayMicroseconds(100);
  }
  float rawAverage = rawSum / 16.0;

  // Convert 12-bit ADC value (0-4095) to Degrees (0 - 360)
  float angleDegrees = (rawAverage / 4095.0) * 360.0;
  
  // Convert 12-bit ADC value to Voltage
  float voltage = (rawAverage / 4095.0) * 3.3;

  // Output to Serial
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.print("V  |  Angle: ");
  Serial.print(angleDegrees, 0);
  Serial.println("°");

  delay(100); // 10Hz update rate
}