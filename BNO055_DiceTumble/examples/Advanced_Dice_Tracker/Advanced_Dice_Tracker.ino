/*
  Advanced_Dice_Tracker.ino
  
  Advanced dice rolling detection with statistics tracking.
  
  Features:
  - Continuous angle monitoring during rolls
  - Roll statistics (count, max angle, duration)
  - Face landing frequency tracking
  - Average roll metrics
  - Real-time calibration monitoring
  
  Perfect for analyzing dice rolling behavior and sensor performance.
*/

#include "BNO055_Motion.h"

BNO055_Motion sensor;

// State machine
enum RollState {
  IDLE,
  ROLLING,
  LANDED
};

RollState state = IDLE;

// Configuration
const float TUMBLE_THRESHOLD = 0.707;  // 45 degrees
const int UPDATE_INTERVAL = 50;        // ms (faster for better tracking)

// Statistics
struct RollStats {
  int rollCount;
  float maxAngle;
  unsigned long startTime;
  unsigned long duration;
  float totalRotation;
  int samples;
} currentRoll, totalStats;

// Face tracking (Z+, Z-, X+, X-, Y+, Y-)
int faceCount[6] = {0, 0, 0, 0, 0, 0};
const char* faceNames[6] = {"Z+", "Z-", "X+", "X-", "Y+", "Y-"};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  printHeader();
  
  // Initialize sensor
  if (!sensor.init(true)) {
    Serial.println("❌ SENSOR INIT FAILED!");
    while (1) delay(100);
  }
  
  sensor.setTumbleThreshold(TUMBLE_THRESHOLD);
  
  // Wait for stability
  Serial.print("\nStabilizing");
  int stableCount = 0;
  while (stableCount < 20) {
    sensor.update();
    stableCount = sensor.stable() ? stableCount + 1 : 0;
    if (stableCount % 5 == 0) Serial.print(".");
    delay(50);
  }
  Serial.println(" Ready!\n");
  
  // Initialize statistics
  resetTotalStats();
  
  // Start monitoring
  sensor.resetTumbleDetection();
  state = IDLE;
  
  printInstructions();
}

void loop() {
  sensor.update();
  
  switch (state) {
    case IDLE:
      handleIdleState();
      break;
      
    case ROLLING:
      handleRollingState();
      break;
      
    case LANDED:
      handleLandedState();
      break;
  }
  
  delay(UPDATE_INTERVAL);
}

// ============================================
// STATE HANDLERS
// ============================================

void handleIdleState() {
  if (sensor.tumbled()) {
    // Roll started!
    state = ROLLING;
    initializeRollStats();
    
    Serial.println("\n╔═══════════════════════════════════════════╗");
    Serial.print("║  ROLL #");
    Serial.print(totalStats.rollCount + 1);
    Serial.println(" STARTED                        ║");
    Serial.println("╚═══════════════════════════════════════════╝\n");
  }
}

void handleRollingState() {
  // Track angle during roll
  float angle = sensor.getTumbleAngle();
  
  // Update statistics
  currentRoll.samples++;
  currentRoll.totalRotation += angle;
  if (angle > currentRoll.maxAngle) {
    currentRoll.maxAngle = angle;
  }
  
  // Output angle with visual bar
  Serial.print("  ");
  Serial.print(angle, 1);
  Serial.print("° ");
  printAngleBar(angle);
  Serial.println();
  
  // Check if stopped moving
  if (!sensor.moving()) {
    state = LANDED;
    Serial.println("\n  ⏸  Stopped moving, checking stability...");
  }
}

void handleLandedState() {
  // Check if stable and flat
  if (sensor.stable() && sensor.on_table()) {
    // Roll complete!
    currentRoll.duration = millis() - currentRoll.startTime;
    
    // Get landing orientation
    BNO055_Orientation orientation = sensor.orientation();
    int faceIndex = orientationToFaceIndex(orientation);
    if (faceIndex >= 0) {
      faceCount[faceIndex]++;
    }
    
    // Update total statistics
    updateTotalStats();
    
    // Print results
    printRollResults(orientation);
    
    // Wait before next roll
    delay(1500);
    
    // Reset for next roll
    sensor.resetTumbleDetection();
    state = IDLE;
    
    Serial.println("\n✓ Ready for next roll!\n");
    
  } else if (sensor.moving()) {
    // Started moving again
    Serial.println("  ⚠  Moving again...\n");
    state = ROLLING;
  }
}

// ============================================
// STATISTICS FUNCTIONS
// ============================================

void initializeRollStats() {
  currentRoll.maxAngle = 0;
  currentRoll.startTime = millis();
  currentRoll.duration = 0;
  currentRoll.totalRotation = 0;
  currentRoll.samples = 0;
}

void updateTotalStats() {
  totalStats.rollCount++;
  if (currentRoll.maxAngle > totalStats.maxAngle) {
    totalStats.maxAngle = currentRoll.maxAngle;
  }
  totalStats.totalRotation += currentRoll.maxAngle;
  totalStats.duration += currentRoll.duration;
  totalStats.samples += currentRoll.samples;
}

void resetTotalStats() {
  totalStats.rollCount = 0;
  totalStats.maxAngle = 0;
  totalStats.totalRotation = 0;
  totalStats.duration = 0;
  totalStats.samples = 0;
  for (int i = 0; i < 6; i++) {
    faceCount[i] = 0;
  }
}

int orientationToFaceIndex(BNO055_Orientation orientation) {
  switch (orientation) {
    case ORIENTATION_Z_UP:   return 0;
    case ORIENTATION_Z_DOWN: return 1;
    case ORIENTATION_X_UP:   return 2;
    case ORIENTATION_X_DOWN: return 3;
    case ORIENTATION_Y_UP:   return 4;
    case ORIENTATION_Y_DOWN: return 5;
    default: return -1;
  }
}

// ============================================
// DISPLAY FUNCTIONS
// ============================================

void printHeader() {
  Serial.println("\n╔═══════════════════════════════════════════╗");
  Serial.println("║     ADVANCED DICE ROLL TRACKER            ║");
  Serial.println("║     Statistics & Analysis Mode            ║");
  Serial.println("╚═══════════════════════════════════════════╝");
}

void printInstructions() {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("  Ready to track dice rolls!");
  Serial.println("  Pick up dice and roll to begin...");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

void printAngleBar(float angle) {
  int bars = (int)(angle / 10);
  for (int i = 0; i < bars && i < 18; i++) {
    Serial.print("█");
  }
}

void printRollResults(BNO055_Orientation orientation) {
  Serial.println("\n┌───────────────────────────────────────────┐");
  Serial.println("│           ROLL COMPLETE                   │");
  Serial.println("└───────────────────────────────────────────┘");
  
  // This roll
  Serial.println("\n  This Roll:");
  Serial.print("    Max rotation: ");
  Serial.print(currentRoll.maxAngle, 1);
  Serial.println("°");
  
  Serial.print("    Duration: ");
  Serial.print(currentRoll.duration / 1000.0, 2);
  Serial.println(" seconds");
  
  Serial.print("    Samples: ");
  Serial.println(currentRoll.samples);
  
  Serial.print("    Landed on: ");
  Serial.println(sensor.getOrientationString());
  
  // Calibration
  uint8_t sys, gyro, accel, mag;
  sensor.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("    Calibration: S:");
  Serial.print(sys);
  Serial.print(" G:");
  Serial.print(gyro);
  Serial.print(" A:");
  Serial.print(accel);
  Serial.print(" M:");
  Serial.println(mag);
  
  // Statistics
  Serial.println("\n  Session Statistics:");
  Serial.print("    Total rolls: ");
  Serial.println(totalStats.rollCount);
  
  Serial.print("    Average rotation: ");
  Serial.print(totalStats.totalRotation / totalStats.rollCount, 1);
  Serial.println("°");
  
  Serial.print("    Max rotation ever: ");
  Serial.print(totalStats.maxAngle, 1);
  Serial.println("°");
  
  Serial.print("    Total time rolling: ");
  Serial.print(totalStats.duration / 1000.0, 1);
  Serial.println(" sec");
  
  // Face distribution
  Serial.println("\n  Face Distribution:");
  for (int i = 0; i < 6; i++) {
    Serial.print("    ");
    Serial.print(faceNames[i]);
    Serial.print(": ");
    Serial.print(faceCount[i]);
    Serial.print(" (");
    Serial.print((faceCount[i] * 100.0) / totalStats.rollCount, 1);
    Serial.println("%)");
  }
  
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}
