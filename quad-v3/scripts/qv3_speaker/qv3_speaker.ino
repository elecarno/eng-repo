// Choose a free pin (e.g., 25)
const int piezoPin = 25;

void setup() {
  // New v3.0+ syntax: ledcAttach(pin, frequency, resolution)
  // We attach the pin directly at a starting frequency of 2000Hz and 8-bit resolution.
  ledcAttach(piezoPin, 2000, 8);
}

void loop() {
  // Play a short chime sequence
  playTone(500, 50);
  playTone(800, 50);
  playTone(600, 50);
  playTone(500, 50);
  playTone(700, 50);

  // Wait 5 seconds before playing again
  delay(5000);
}

// Custom function adjusted for ESP32 Core v3.0+
void playTone(double frequency, int duration) {
  if (frequency == 0) {
    ledcWriteTone(piezoPin, 0); // Stop the tone
  } else {
    ledcWriteTone(piezoPin, frequency); // Start the tone directly on the PIN, not a channel
  }
  delay(duration);                       // Hold it
  ledcWriteTone(piezoPin, 0);           // Stop the tone
  delay(50);                             // Brief pause between notes
}