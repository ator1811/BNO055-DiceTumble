/*
  Dice_Roll_Detection.ino
  
  Complete dice rolling detection example using BNO055_Motion library.
  
  Workflow:
  1. Wait for tumble to start
  2. Output rotation angle during tumbling
  3. Detect when dice lands flat and stable
  4. Display final orientation
  5. Reset and repeat
  
  This demonstrates the full cycle of detecting a dice roll.
*/

#include "BNO055_Motion.h"

// Create sensor instance
BNO055_Motion sensor;

// State machine for dice roll detection
enum DiceState {
  WAITING,      // Waiting for tumble to start
  TUMBLING,     // Actively tumbling
  SETTLING,     // Stopped moving but not yet stable
  STABLE        // Flat and stable on table
};

DiceState currentState = WAITING;

// Configuration
const float TUMBLE_THRESHOLD = 0.707;  // 45 degrees (cos(45°))
const int UPDATE_INTERVAL = 100;       // ms between updates

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║  Dice Roll Detection Example          ║");
  Serial.println("║  BNO055 Rotation Matrix Method         ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // Initialize sensor
  Serial.println("Initializing sensor...");
  if (!sensor.init(true)) {
    Serial.println("❌ Sensor initialization failed!");
    Serial.println("Check connections and reset.");
    while (1) delay(100);
  }
  
  // Set tumble threshold
  sensor.setTumbleThreshold(TUMBLE_THRESHOLD);
  
  // Wait for initial stability
  Serial.println("\nWaiting for sensor to stabilize...");
  waitForStability();
  
  // Ready to start
  Serial.println("\n✓ Initialization complete!");
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  Ready to detect dice rolls!          ║");
  Serial.println("║  Place dice on table to start...      ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // Initialize for first roll
  sensor.resetTumbleDetection();
  currentState = WAITING;
}

void loop() {
  // Update sensor readings
  sensor.update();
  
  // State machine
  switch (currentState) {
    
    case WAITING:
      handleWaitingState();
      break;
      
    case TUMBLING:
      handleTumblingState();
      break;
      
    case SETTLING:
      handleSettlingState();
      break;
      
    case STABLE:
      handleStableState();
      break;
  }
  
  delay(UPDATE_INTERVAL);
}

// ============================================
// STATE HANDLERS
// ============================================

void handleWaitingState() {
  // Check if tumble has started
  if (sensor.tumbled()) {
    Serial.println("\n🎲 TUMBLE DETECTED! Roll in progress...");
    Serial.println("─────────────────────────────────────");
    currentState = TUMBLING;
  }
}

void handleTumblingState() {
  // Output rotation angle during tumble
  float angle = sensor.getTumbleAngle();
  
  // Print angle with visual indicator
  Serial.print("Rotation: ");
  Serial.print(angle, 1);
  Serial.print("° ");
  
  // Visual bar graph (each ▓ = 10 degrees)
  int bars = (int)(angle / 10);
  for (int i = 0; i < bars && i < 18; i++) {
    Serial.print("▓");
  }
  Serial.println();
  
  // Check if dice has stopped moving
  if (!sensor.moving()) {
    Serial.println("\n⏸  Motion stopped, waiting for stability...");
    currentState = SETTLING;
  }
}

void handleSettlingState() {
  // Wait for dice to become stable
  if (sensor.stable() && sensor.on_table()) {
    Serial.println("✓ Stable and flat detected!");
    currentState = STABLE;
  }
  else if (sensor.moving()) {
    // Started moving again, back to tumbling
    Serial.println("⚠  Movement detected, continuing...");
    currentState = TUMBLING;
  }
}

void handleStableState() {
  // Dice has landed - show results
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║          ROLL COMPLETE!                ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Display final rotation
  float finalAngle = sensor.getTumbleAngle();
  Serial.print("\nTotal rotation: ");
  Serial.print(finalAngle, 1);
  Serial.println("°");
  
  // Display orientation (which face is up)
  Serial.print("Landed on: ");
  Serial.println(sensor.getOrientationString());
  
  // Display calibration status
  uint8_t sys, gyro, accel, mag;
  sensor.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("Calibration: Sys:");
  Serial.print(sys);
  Serial.print(" G:");
  Serial.print(gyro);
  Serial.print(" A:");
  Serial.print(accel);
  Serial.print(" M:");
  Serial.println(mag);
  
  // Wait a moment before next roll
  Serial.println("\n─────────────────────────────────────");
  Serial.println("Waiting 2 seconds before next roll...");
  Serial.println("─────────────────────────────────────\n");
  delay(2000);
  
  // Reset for next roll
  sensor.resetTumbleDetection();
  currentState = WAITING;
  
  Serial.println("✓ Ready for next roll!");
  Serial.println("  Pick up dice and roll again...\n");
}

// ============================================
// HELPER FUNCTIONS
// ============================================

void waitForStability() {
  int stableCount = 0;
  const int requiredStableCount = 10;
  
  while (stableCount < requiredStableCount) {
    sensor.update();
    
    if (sensor.stable()) {
      stableCount++;
      Serial.print(".");
    } else {
      stableCount = 0;
    }
    
    delay(100);
  }
  Serial.println(" OK!");
}
