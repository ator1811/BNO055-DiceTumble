/*
  Simple_Tumble_Monitor.ino
  
  Simplified example that continuously outputs rotation angle.
  Shows angle whenever tumbling is detected, resets when stable.
  
  Perfect for testing and debugging tumble detection.
*/

#include "BNO055_Motion.h"

BNO055_Motion sensor;

// Configuration
const float TUMBLE_THRESHOLD = 0.707;  // 45 degrees
const int UPDATE_INTERVAL = 100;       // ms

bool isTumbling = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n=== Simple Tumble Monitor ===\n");
  
  // Initialize sensor
  if (!sensor.init(true)) {
    Serial.println("ERROR: Sensor init failed!");
    while (1) delay(100);
  }
  
  sensor.setTumbleThreshold(TUMBLE_THRESHOLD);
  
  // Wait for stability
  Serial.print("Stabilizing");
  for (int i = 0; i < 10; i++) {
    sensor.update();
    if (!sensor.stable()) i = 0;
    Serial.print(".");
    delay(100);
  }
  Serial.println(" Ready!\n");
  
  // Reset and start monitoring
  sensor.resetTumbleDetection();
  Serial.println("Monitoring for tumbles...\n");
}

void loop() {
  sensor.update();
  
  // Check tumble status
  if (sensor.tumbled()) {
    if (!isTumbling) {
      // Just started tumbling
      Serial.println("\n>>> TUMBLE STARTED <<<");
      isTumbling = true;
    }
    
    // Output current angle
    float angle = sensor.getTumbleAngle();
    Serial.print("Angle: ");
    Serial.print(angle, 1);
    Serial.println("°");
  }
  
  // Check if stable and flat
  if (isTumbling && sensor.stable() && sensor.on_table()) {
    // Tumble finished
    Serial.println(">>> STABLE & FLAT <<<");
    Serial.print("Final angle: ");
    Serial.print(sensor.getTumbleAngle(), 1);
    Serial.println("°");
    Serial.print("Orientation: ");
    Serial.println(sensor.getOrientationString());
    Serial.println("\n--- Resetting for next tumble ---\n");
    
    // Reset for next detection
    delay(1000);
    sensor.resetTumbleDetection();
    isTumbling = false;
    
    Serial.println("Ready...\n");
  }
  
  delay(UPDATE_INTERVAL);
}
