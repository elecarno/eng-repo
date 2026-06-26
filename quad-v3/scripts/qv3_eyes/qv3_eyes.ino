#include <MD_MAX72xx.h>
#include <SPI.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 2

// ESP32 Hardware VSPI Pins
#define CLK_PIN   18  
#define DATA_PIN  23  
#define CS_PIN    5   

// Initialize using hardware SPI
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// PATTERNS - make using https://xantorohara.github.io/led-matrix-editor/
const uint8_t pLeft[8] = {
  0b00000000,
  0b00000100,
  0b00000100,
  0b00000100,
  0b00000100,
  0b00000100,
  0b00111100,
  0b00000000
};

const uint8_t pRight[8] = {
  0b00000000,
  0b00011100,
  0b00100100,
  0b00100100,
  0b00011100,
  0b00100100,
  0b00100100,
  0b00000000
};

const uint8_t pSmiley[8] = {
  0b00111100,
  0b01000010,
  0b10100101,
  0b10000001,
  0b10100101,
  0b10011001,
  0b01000010,
  0b00111100
};

const uint8_t pHeart[8] = {
  0b00000000,
  0b01100110, 
  0b11111111,
  0b11111111,
  0b01111110, 
  0b00111100, 
  0b00011000, 
  0b00000000       
};

const uint8_t pFull[8] = {
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111      
};

const uint8_t pBored[8] = {
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111      
};

const uint8_t pHappy[8] = {
  0b11111111,
  0b11111111,
  0b11111111,
  0b11111111,
  0b11000011,
  0b10000001,
  0b00000000,
  0b00000000     
};

enum DisplaySide { D_LEFT = 0, D_RIGHT = 1 };

void drawMatrixPattern(enum DisplaySide side, const uint8_t pattern[]) {
  uint8_t startCol = side * 8; // set offset needed

  mx.control((uint8_t)side, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (uint8_t col = 0; col < 8; col++) {
    mx.setColumn(startCol + col, pattern[col]); 
  }
  mx.control((uint8_t)side, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void setup() {
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2); // Set brightness low (0-15)
  mx.clear();

  // --- STARTUP TEST ---
  drawMatrixPattern(D_LEFT, pLeft);
  delay(1000);
  drawMatrixPattern(D_RIGHT, pRight);
  delay(1000);

  mx.clear(); // clear everything for main loop
  delay(1000);
}

void loop() {
  drawMatrixPattern(D_LEFT,  pHappy);
  drawMatrixPattern(D_RIGHT, pHappy);

  delay(100);
}