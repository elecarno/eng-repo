// If this still brownouts, we will need to give each sensor its own trigger pin.
const int TRIG_PIN = 12;
const int ECHO_PINS[] = {14, 27, 26}; 
const char* SENSOR_LABELS[] = {"Left", "Middle", "Right"};
const int SENSOR_COUNT = 3;

void setup() {
  Serial.begin(115200); 
  pinMode(TRIG_PIN, OUTPUT);
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(ECHO_PINS[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5); // Increased stability delay
    
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PINS[i], HIGH, 25000); 
    float distance = (duration * 0.0343) / 2;
    
    Serial.print("Sensor ");
    Serial.print(SENSOR_LABELS[i]); 
    Serial.print(": ");
    
    if (distance == 0) {
      Serial.println("Out of range");
    } else {
      Serial.print(distance);
      Serial.println(" cm");
    }
    
    // Give a generous 80ms for the power to stabilize and sound to fade
    delay(80); 
  }
  
  Serial.println("-----------------------");
  delay(300); 
}