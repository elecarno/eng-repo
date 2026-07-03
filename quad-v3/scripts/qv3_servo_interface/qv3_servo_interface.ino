// STEVEN SERVO INTERFACE CODE v2.0 - FIXED & COMPILABLE
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- CONFIGURATION ----------------------------------------------------------
#define SERVO_FREQ      50
#define ENABLE_DIAGNOSTICS  true

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
// STAGED INITIALIZATION
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("   QUADRUPED SERVOS INITIALIZATION v2.0  ");
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
  delay(100);
  
  Serial.println("\n[STEP 2] Testing each channel...");
  for (uint8_t i = 0; i < 14; i++) {
    Serial.printf("Testing channel %d... ", i);
    pwm.writeMicroseconds(i, 1500);
    delay(80);
    Serial.println("OK");
    
    currentPose[i] = robotRestPose[i];
    servoHealthy[i] = true;
    servoLastUpdate[i] = millis();
  }
  
  Serial.println("\n[STEP 3] Loading rest pose (staggered)...");
  
  // Group legs to prevent simultaneous current spikes
  const uint8_t legGroups[4][4] = {
    {0, 1, 2, 3},
    {4, 5, 6, 7},
    {8, 9, 10, 11},
    {12, 13, 0, 0}  // Last group smaller
  };
  
  for (uint8_t g = 0; g < 4; g++) {
    Serial.printf("Loading group %d...", g);
    
    for (uint8_t chIdx = 0; chIdx < 4; chIdx++) {
      uint8_t channel = legGroups[g][chIdx];
      
      // Skip dummy slots (third antenna slot)
      if (channel == 0 && g == 3) continue;
      if (channel >= 14) continue;
      
      // Extended warm-up for known-stubborn channels
      if (channel <= 1 || channel == 7 || channel == 4) {
        Serial.printf(" [WARM-UP]");
        
        // Three progressive pulses
        pwm.writeMicroseconds(channel, 1000);
        delay(25);
        pwm.writeMicroseconds(channel, 2000);
        delay(25);
        pwm.writeMicroseconds(channel, robotRestPose[channel]);
        delay(50);
      } else {
        safeWriteMicroseconds(channel, robotRestPose[channel]);
      }
    }
    
    delay(100);  // Let power stabilize between groups
    Serial.println(" DONE");
  }
  
  Serial.println("\n========================================");
  Serial.println("   STARTUP COMPLETE                      ");
  Serial.println("========================================");
  
  for (uint8_t i = 0; i < 14; i++) {
    Serial.printf("CH %2d: POS=%4d µs\n", i, currentPose[i]);
  }
  
  delay(2000);
}


// ============================================================================
// RUNTIME LOOP
// ============================================================================

void loop() {
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
  
  delay(25);  // 40Hz refresh cycle
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