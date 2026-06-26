#include <MD_MAX72xx.h>
#include <SPI.h>

// AZDelivery modules are usually FC16_HW. 
// If your display looks upside down or reversed, change this to GENERIC_HW.
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 2

// ESP32 Hardware VSPI Pins
#define CLK_PIN   18  
#define DATA_PIN  23  
#define CS_PIN    5   

// Initialize using hardware SPI
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

void setup() {
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2); // Set brightness low (0-15) so it doesn't blind you
  mx.clear();

  // --- STARTUP TEST ---
  // Step 1: Light up the first matrix (Device 0)
  mx.control(0, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (uint8_t row = 0; row < 8; row++) {
    mx.setRow(0, row, 0xFF); // Turn on all 8 LEDs in this row
  }
  mx.control(0, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  delay(1000);

  // Step 2: Light up the second matrix (Device 1)
  mx.control(1, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (uint8_t row = 0; row < 8; row++) {
    mx.setRow(1, row, 0xFF);
  }
  mx.control(1, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
  delay(1000);

  mx.clear(); // Clear everything for the main loop
}

void loop() {
  // --- BOUNCING PIXEL ANIMATION ---
  // Total width of two matrices is 16 columns (0 to 15)
  // Row 3 is just a nice middle row to watch the pixel move
  int targetRow = 3; 

  // Move the pixel from left to right (Col 0 to 15)
  for (int col = 0; col < 16; col++) {
    mx.setPoint(targetRow, col, true);  // Turn LED on
    delay(50);                          // Speed of the pixel
    mx.setPoint(targetRow, col, false); // Turn LED off
  }

  // Move the pixel from right to left (Col 15 down to 0)
  for (int col = 15; col >= 0; col--) {
    mx.setPoint(targetRow, col, true);
    delay(50);
    mx.setPoint(targetRow, col, false);
  }
}