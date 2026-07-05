// DESCRIPTION ----------------------------------------------------------------------------------------------------------------
// This is the main hardware script for the Steven robot
// It has four functions:
// 1. Servo control
// 2. Sensor control & data processing
// 3. Matrix display control
// 4. Buzzer control


// INCLUDES -------------------------------------------------------------------------------------------------------------------
// MAX7219 8x8 Matrix Displays
#include <MD_MAX72xx.h>
#include <SPI.h>
// PCA9685 Servo Driver
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// GLOBALS & FUNCTIONS --------------------------------------------------------------------------------------------------------

enum dDisplaySide { D_LEFT = 0, D_RIGHT = 1 };

// SERVOS (s-) --------------------------------------------------------
// --- CONFIGURATION ----------------------------------------------------------
#define SERVO_FREQ      50
#define ENABLE_DIAGNOSTICS  false

// Servo limits
#define USMIN_MG996R    500  
#define USMAX_MG996R   2500  
#define USMIN_MG90S     600  
#define USMAX_MG90S    2400  

// Rest position definition
const uint16_t robotRestPose[14] = {
  1000, 1764, 1735,  // Front Left
  2000, 1235, 1264,  // Front Right
  2000, 1235, 1264,  // Back Left
  1000, 1764, 1735,  // Back Right
  USMIN_MG90S, USMAX_MG90S  // Antennas
};

// Runtime state tracking
uint16_t currentPose[14];
bool servoHealthy[14];       
unsigned long servoLastUpdate[14];

Adafruit_PWMServoDriver pwm;


// ============================================================================
// HELPER FUNCTIONS - FIXED STRING HANDLING
// ============================================================================

void printDiagnostic(const char* msg) {
#if ENABLE_DIAGNOSTICS
  Serial.print("[DIAG] ");
  Serial.println(msg);
#endif
}

// Use sprintf instead of String concatenation for diagnostics
void markServoUnhealthy(uint8_t channel) {
  if (!servoHealthy[channel]) {
    char diagBuf[32];
    snprintf(diagBuf, sizeof(diagBuf), "SERVO %d UNHEALTHY", channel);
    printDiagnostic(diagBuf);
    servoHealthy[channel] = false;
  }
}

void markServoHealthy(uint8_t channel) {
  if (!servoHealthy[channel]) {
    char diagBuf[32];
    snprintf(diagBuf, sizeof(diagBuf), "SERVO %d RECOVERED", channel);
    printDiagnostic(diagBuf);
    servoHealthy[channel] = true;
  }
  servoLastUpdate[channel] = millis();
}

bool safeWriteMicroseconds(uint8_t channel, uint16_t us, uint8_t retries = 3) {
  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    pwm.writeMicroseconds(channel, us);
    delay(1);
    
    // Simple success assumption - can't truly verify output on PCA9685
    markServoHealthy(channel);
    return true;
  }
  
  char diagBuf[32];
  snprintf(diagBuf, sizeof(diagBuf), "SERVO WRITE FAILED: CH %d", channel);
  printDiagnostic(diagBuf);
  markServoUnhealthy(channel);
  return false;
}

// ============================================================================
// COMMAND PARSING - FIXED STRING TRIMMING
// ============================================================================

void parseAndMove(String data) {
  int idx0  = data.indexOf("C0:");
  int idx1  = data.indexOf("C1:");
  int idx2  = data.indexOf("C2:");
  int idx3  = data.indexOf("C3:");
  int idx4  = data.indexOf("C4:");
  int idx5  = data.indexOf("C5:");
  int idx6  = data.indexOf("C6:");
  int idx7  = data.indexOf("C7:");
  int idx8  = data.indexOf("C8:");
  int idx9  = data.indexOf("C9:");
  int idx10 = data.indexOf("C10:");
  int idx11 = data.indexOf("C11:");
  int idx12 = data.indexOf("C12:");
  int idx13 = data.indexOf("C13:");
  
  if (idx0 == -1 || idx1 == -1 || idx2 == -1 || idx3 == -1 ||
      idx4 == -1 || idx5 == -1 || idx6 == -1 || idx7 == -1 ||
      idx8 == -1 || idx9 == -1 || idx10 == -1 || idx11 == -1 ||
      idx12 == -1 || idx13 == -1) {
    Serial.println("ERROR: Malformed command - missing markers");
    return;
  }
  
  // FIX: Parse substring into temp String FIRST, THEN trim, THEN convert
  int vals[14];
  String tmpStr;
  
  tmpStr = data.substring(idx0  + 3, idx1 ); tmpStr.trim(); vals[0] = tmpStr.toInt();
  tmpStr = data.substring(idx1  + 3, idx2 ); tmpStr.trim(); vals[1] = tmpStr.toInt();
  tmpStr = data.substring(idx2  + 3, idx3 ); tmpStr.trim(); vals[2] = tmpStr.toInt();
  tmpStr = data.substring(idx3  + 3, idx4 ); tmpStr.trim(); vals[3] = tmpStr.toInt();
  tmpStr = data.substring(idx4  + 3, idx5 ); tmpStr.trim(); vals[4] = tmpStr.toInt();
  tmpStr = data.substring(idx5  + 3, idx6 ); tmpStr.trim(); vals[5] = tmpStr.toInt();
  tmpStr = data.substring(idx6  + 3, idx7 ); tmpStr.trim(); vals[6] = tmpStr.toInt();
  tmpStr = data.substring(idx7  + 3, idx8 ); tmpStr.trim(); vals[7] = tmpStr.toInt();
  tmpStr = data.substring(idx8  + 3, idx9 ); tmpStr.trim(); vals[8] = tmpStr.toInt();
  tmpStr = data.substring(idx9  + 3, idx10); tmpStr.trim(); vals[9] = tmpStr.toInt();
  tmpStr = data.substring(idx10 + 4, idx11); tmpStr.trim(); vals[10] = tmpStr.toInt();
  tmpStr = data.substring(idx11 + 4, idx12); tmpStr.trim(); vals[11] = tmpStr.toInt();
  tmpStr = data.substring(idx12 + 4, idx13); tmpStr.trim(); vals[12] = tmpStr.toInt();
  tmpStr = data.substring(idx13 + 4       ); tmpStr.trim(); vals[13] = tmpStr.toInt();
  
  // Apply with bounds checking
  bool movementWarning = false;
  for (uint8_t i = 0; i < 14; i++) {
    uint16_t minVal = (i < 12) ? USMIN_MG996R : USMIN_MG90S;
    uint16_t maxVal = (i < 12) ? USMAX_MG996R : USMAX_MG90S;
    
    int constrained = constrain(vals[i], minVal, maxVal);
    int delta = abs(constrained - currentPose[i]);
    
    if (delta > 500) {
      Serial.printf("WARN: Large jump CH %d (%d -> %d)\n", i, currentPose[i], constrained);
      movementWarning = true;
    }
    
    // Only reject extreme jumps if safety threshold exceeded
    if (delta < 800) {  // Removed broken comment syntax
      currentPose[i] = constrained;
    }
  }
  
  if (movementWarning) {
    Serial.println("(Movement warnings logged - positions adjusted)");
  }
}


// SENSORS (e- )-------------------------------------------------------
const int TRIG_PIN = 12;
const int ECHO_PINS[] = {14, 27, 26}; 
const char* SENSOR_LABELS[] = {"Left", "Middle", "Right"};
const int SENSOR_COUNT = 3;


// MATRIX DISPLAYS (d- )-----------------------------------------------
#define dHARDWARE_TYPE MD_MAX72XX::FC16_HW
const int dMAX_DEVICES = 2;

// ESP32 hardware VSPI pins
const int dCLK_PIN  = 18;
const int dDATA_PIN = 23;
const int dCS_PIN   = 5;
// initialize using hardware SPI
MD_MAX72XX mx = MD_MAX72XX(dHARDWARE_TYPE, dCS_PIN, dMAX_DEVICES);

// patterns - make using https://xantorohara.github.io/led-matrix-editor/
const uint8_t dpLeft[8] = {
  0b00000000, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00000100, 0b00111100, 0b00000000
};

const uint8_t dpRight[8] = {
  0b00000000, 0b00011100, 0b00100100, 0b00100100, 0b00011100, 0b00100100, 0b00100100, 0b00000000
};

const uint8_t dpSmiley[8] = {
  0b00111100, 0b01000010, 0b10100101, 0b10000001, 0b10100101, 0b10011001, 0b01000010, 0b00111100
};

const uint8_t dpHeart[8] = {
  0b00000000, 0b01100110,  0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000, 0b00000000       
};

const uint8_t dpFull[8] = {
  0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11111111
};

const uint8_t dpBored[8] = {
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b11111111, 0b11111111, 0b11111111, 0b11111111      
};

const uint8_t dpHappy[8] = {
  0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b11000011, 0b10000001, 0b00000000, 0b00000000     
};

void dDrawMatrixPattern(enum dDisplaySide side, const uint8_t pattern[]) {
  uint8_t startCol = side * 8; // set offset needed

  mx.control((uint8_t)side, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  for (uint8_t col = 0; col < 8; col++) {
    mx.setColumn(startCol + col, pattern[col]); 
  }
  mx.control((uint8_t)side, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}


// BUZZER (b- )--------------------------------------------------------
const int bPIN_BUZZER = 25;

void bPlayTone(double frequency, int duration) {
  if (frequency == 0) {
    ledcWriteTone(bPIN_BUZZER, 0);         // stop tone
  } else {
    ledcWriteTone(bPIN_BUZZER, frequency); // start tone directly on PIN
  }
  delay(duration);                        // hold tone
  ledcWriteTone(bPIN_BUZZER, 0);           // stop tone
  delay(50);                              // pause between tones
}


// SETUP -----------------------------------------------------------------------------------------------------------------------
void setup() {
  // SENSORS ----------------------------------------------------------
  Serial.begin(115200); 
  // pinMode(TRIG_PIN, OUTPUT);
  // for (int i = 0; i < SENSOR_COUNT; i++) {
  //   pinMode(ECHO_PINS[i], INPUT);
  // }

  // MATRIX DISPLAYS --------------------------------------------------
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 2); // Set brightness low (0-15)
  mx.clear();

  // --- STARTUP TEST ---
  dDrawMatrixPattern(D_LEFT, dpLeft);
  delay(1000);
  dDrawMatrixPattern(D_RIGHT, dpRight);
  delay(1000);

  mx.clear(); // clear everything for main loop
  delay(1000);

  // BUZZER -----------------------------------------------------------
  ledcAttach(bPIN_BUZZER, 2000, 8);

  // SERVOS -----------------------------------------------------------
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("    QUADRUPED SERVOS INITIALIZATION v3.1  ");
  Serial.println("========================================\n");
  
  Wire.begin(21, 22);
  Wire.setClock(400000);
  
  Serial.println("[STEP 1] Initializing PCA9685...");
  if (!pwm.begin()) {
    Serial.println("FATAL ERROR: PCA9685 not responding!");
    while (true) delay(1000);
  }
  
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(200);
  
  Serial.println("\n[STEP 2] Initializing Rest Poses (Isolated Knee Rails)...");
  
  // Revised grouping: We split the high-load joints so they NEVER activate together
  const uint8_t legGroups[4][4] = {
    {0, 1, 2, 3},     // Phase 1: Front Left completely + Front Right hip
    {5, 6, 7, 8},     // Phase 2: Front Right ankle + Back Left completely
    {9, 11, 255, 255},// Phase 3: Back Right hip and ankle only
    {4, 10, 255, 255} // Phase 4: CRITICAL KNEES ONLY - Woken up last when chassis is supported
  };
  
  for (uint8_t g = 0; g < 4; g++) {
    Serial.printf("Spooling up Leg Group %d...\n", g);
    
    for (uint8_t chIdx = 0; chIdx < 4; chIdx++) {
      uint8_t channel = legGroups[g][chIdx];
      if (channel == 255) continue; 
      
      // SOFT-START RAMP FOR THE PROBLEM KNEE JOINTS (4 and 10)
      if (channel == 4 || channel == 10) {
        Serial.printf("  [KNEE SOFT-START] Easing CH %d into load...\n", channel);
        
        uint16_t startPulse = 1500; // Assume standard neutral midpoint to initialize internal IC
        uint16_t targetPulse = robotRestPose[channel];
        int step = (targetPulse > startPulse) ? 20 : -20;
        
        // Wake the chip up at neutral
        pwm.writeMicroseconds(channel, startPulse);
        delay(50);
        
        // Smoothly step toward target rest pose to avoid high stall-current spikes
        for (uint16_t p = startPulse; abs(p - targetPulse) > 20; p += step) {
          pwm.writeMicroseconds(channel, p);
          delay(25); // 25ms increments gives the 4A supply time to breathe
        }
      }
      
      // Standard direct power-up for non-problem channels
      Serial.printf("  Waking up CH %d -> %d µs\n", channel, robotRestPose[channel]);
      safeWriteMicroseconds(channel, robotRestPose[channel]);
      
      currentPose[channel] = robotRestPose[channel];
      servoHealthy[channel] = true;
      servoLastUpdate[channel] = millis();
      
      delay(50); 
    }
    
    delay(300); // Extended pause between groups to clear trace heating/sag
    Serial.println("  Group Stabilized.");
  }

  // Initialize the antennas separately at the very end
  Serial.println("\n[STEP 3] Initializing Antennas...");
  for (uint8_t i = 12; i < 14; i++) {
    safeWriteMicroseconds(i, robotRestPose[i]);
    currentPose[i] = robotRestPose[i];
    servoHealthy[i] = true;
    servoLastUpdate[i] = millis();
    delay(50);
  }
  
  Serial.println("\n========================================");
  Serial.println("    STARTUP COMPLETE - ALL RAILS LOCKED   ");
  Serial.println("========================================");
  
  delay(1000);
}

// MASTER LOOP ----------------------------------------------------------------------------------------------------------------
void loop() {
  // MATRIX DISPLAYS --------------------------------------------------
  dDrawMatrixPattern(D_LEFT,  dpFull);
  dDrawMatrixPattern(D_RIGHT, dpFull);

  // BUZZER -----------------------------------------------------------
  // bPlayTone(500, 50);
  // bPlayTone(800, 50);
  // bPlayTone(600, 50);
  // bPlayTone(500, 50);
  // bPlayTone(700, 50);

  // Serial.println("-----------------------");
  
  // SERVOS -----------------------------------------------------------
  static unsigned long lastSystemCheck = 0;
  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseAndMove(data);
    delay(5);
  }
  
  // Update all servos at controlled rate
  for (uint8_t i = 0; i < 14; i++) {
    if (servoHealthy[i]) {
      pwm.writeMicroseconds(i, currentPose[i]);
    } else {
      pwm.writeMicroseconds(i, 1500);  // Safe center position
    }
  }
  
  // Periodic diagnostics every 5 seconds
  if (millis() - lastSystemCheck > 5000) {
    lastSystemCheck = millis();
    
    #if ENABLE_DIAGNOSTICS
    uint8_t offline = 0;
    for (uint8_t i = 0; i < 14; i++) {
      if ((millis() - servoLastUpdate[i]) > 2000) offline++;
    }
    if (offline > 0) {
      Serial.printf("[%lu] STATUS: %d servos inactive\n", millis(), offline);
    }
    #endif
  }
  
  // SENSORS ----------------------------------------------------------
  // for (int i = 0; i < SENSOR_COUNT; i++) {
  //   digitalWrite(TRIG_PIN, LOW);
  //   delayMicroseconds(5); // Increased stability delay
    
  //   digitalWrite(TRIG_PIN, HIGH);
  //   delayMicroseconds(10);
  //   digitalWrite(TRIG_PIN, LOW);
    
  //   long duration = pulseIn(ECHO_PINS[i], HIGH, 25000); 
  //   float distance = (duration * 0.0343) / 2;
    
  //   Serial.print("Sensor ");
  //   Serial.print(SENSOR_LABELS[i]); 
  //   Serial.print(": ");
    
  //   if (distance == 0) {
  //     Serial.println("Out of range");
  //   } else {
  //     Serial.print(distance);
  //     Serial.println(" cm");
  //   }
    
  //   // Give a generous 80ms for the power to stabilize and sound to fade
  //   // delay(80); 
  // }
  
  // Serial.println("-----------------------");
  // delay(300); 

  delay(25);  // 40Hz refresh cycle
}
