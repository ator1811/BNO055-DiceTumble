/*
  BNO055_Motion.h - Wrapper library for BNO055 motion and orientation detection
  
  Features:
  - Easy initialization with custom axis remapping
  - Motion and stability detection
  - Orientation detection (which face is down)
  - Gyroscope and accelerometer access
  - Rotation matrix-based tumble detection (insensitive to vertical axis rotation)
  - Tunable thresholds
  
  Usage:
    BNO055_Motion sensor;
    
    void setup() {
      Serial.begin(115200);
      sensor.init();
    }
    
    void loop() {
      sensor.update();  // Call regularly (every 100ms recommended)
      
      if (sensor.moving()) {
        Serial.println("Motion detected!");
      }
      
      if (sensor.stable()) {
        Serial.println("Sensor is stable");
      }
      
      Serial.print("Orientation: ");
      Serial.println(sensor.getOrientationString());
    }
*/

#ifndef BNO055_MOTION_H
#define BNO055_MOTION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Orientation enumeration
enum BNO055_Orientation {
  ORIENTATION_UNKNOWN,
  ORIENTATION_Z_UP,      // Normal vertical position (X+ physically up)
  ORIENTATION_Z_DOWN,    // Upside down
  ORIENTATION_X_UP,      // Tilted
  ORIENTATION_X_DOWN,    // Tilted opposite
  ORIENTATION_Y_UP,      // Tilted sideways
  ORIENTATION_Y_DOWN,    // Tilted opposite sideways
  ORIENTATION_TILTED     // Not aligned with any axis
};

class BNO055_Motion {
public:
  // Constructor
  BNO055_Motion();
  
  // ============================================
  // CORE FUNCTIONS (as requested)
  // ============================================
  
  // Initialize sensor with custom axis remapping
  // Returns true if successful, false if sensor not detected
  // Set verbose=true to print initialization progress to Serial
  bool init(bool verbose = false);
  
  // Check if sensor is currently moving
  boolean moving();
  
  // Check if sensor is stable (not moving)
  boolean stable();
  
  // Check if sensor is flat on one of its faces (on table)
  // Returns true if orientation is Z_UP, Z_DOWN, X_UP, X_DOWN, Y_UP, or Y_DOWN
  boolean on_table();
  
  // Get current orientation
  BNO055_Orientation orientation();
  
  // Get gyroscope readings in deg/s (as output by BNO055)
  // NOTE: These return raw sensor values in degrees per second
  // Conversion to rad/s happens internally in updateUpVector()
  float gyroX();
  float gyroY();
  float gyroZ();
  
  // ============================================
  // ADDITIONAL USEFUL FUNCTIONS
  // ============================================
  
  // Must be called regularly (every 100ms recommended) to update sensor state
  void update();
  
  // Get accelerometer readings in m/s²
  float accelX();
  float accelY();
  float accelZ();
  
  // Get total acceleration magnitude
  float getAccelMagnitude();
  
  // Get acceleration change since last update
  float getAccelChange();
  
  // Get orientation as human-readable string
  String getOrientationString();
  
  // Get calibration status (0-3 for each, 3 = fully calibrated)
  void getCalibration(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
  
  // Check if sensor is calibrated (all values >= 2)
  boolean isCalibrated();
  
  // ============================================
  // TUNABLE PARAMETERS
  // ============================================
  
  // Set motion detection threshold (default: 0.5 m/s²)
  // Lower = more sensitive to motion
  void setMotionThreshold(float threshold);
  
  // Set stability threshold (default: 0.15 m/s²)
  // Lower = stricter stability requirement
  void setStableThreshold(float threshold);
  
  // Set number of stable samples required (default: 5)
  // Higher = longer wait before declaring stable
  void setStableCount(int count);
  
  // Set orientation detection thresholds
  void setOrientationThresholds(float minGravity, float maxGravity, float maxOtherAxis);
  
  // ============================================
  // AXIS REMAPPING CONFIGURATION
  // ============================================
  
  // Set custom axis remapping (use carefully!)
  // config: AXIS_MAP_CONFIG register value
  // sign: AXIS_MAP_SIGN register value
  void setAxisRemap(uint8_t config, uint8_t sign);
  
  // Get current axis remap configuration
  void getAxisRemap(uint8_t* config, uint8_t* sign);
  
  // ============================================
  // TUMBLE DETECTION (Rotation Matrix-based)
  // ============================================
  
  // Reset tumble detection - captures current "up" direction as reference
  // Uses gravity vector to determine initial orientation
  // Can be called with sensor in ANY orientation
  void resetTumbleDetection();
  
  // Check if sensor has tumbled beyond threshold since last reset
  // Uses rotation matrices to track orientation change
  // Insensitive to rotation around the vertical axis (spinning on table)
  // Only detects actual rolling motion
  boolean tumbled();
  
  // Get the rotation angle in degrees since reference was set
  // Calculated from dot product between current and initial up vectors
  float getTumbleAngle();
  
  // Set tumble detection threshold (cosine of angle, default: 0.707 = 45°)
  // Common values: 0.866 (30°), 0.707 (45°), 0.5 (60°), 0.0 (90°)
  // Threshold represents cos(angle), where angle is the rotation threshold
  void setTumbleThreshold(float threshold);
  
  // ============================================
  // DEBUG FUNCTIONS
  // ============================================
  
  // Get the current dot product (for debugging)
  // Returns value between -1.0 and 1.0
  float getDebugDotProduct();
  
  // Get the current "up" vector (for debugging)
  void getDebugUpVector(float* x, float* y, float* z);
  
  // Get the reference "up" vector (for debugging)
  void getDebugUpStart(float* x, float* y, float* z);
  
  // Print comprehensive debug info to Serial
  void printDebugInfo();

private:
  Adafruit_BNO055 bno;
  
  // Sensor readings
  imu::Vector<3> _accel;
  imu::Vector<3> _gyro;
  
  // Motion detection state
  float _prevAccelMag;
  float _currentAccelMag;
  float _accelChange;
  bool _isMoving;
  int _stableCounter;
  
  // Orientation state
  BNO055_Orientation _currentOrientation;
  
  // Thresholds (tunable)
  float _motionThreshold;
  float _stableThreshold;
  int _stableCountRequired;
  float _flatGravityMin;
  float _flatGravityMax;
  float _flatOtherAxisMax;
  
  // Axis remap configuration
  uint8_t _axisRemapConfig;
  uint8_t _axisRemapSign;
  
  // Tumble detection (rotation matrix-based)
  float _xUp;              // Current "up" vector (updated by rotation matrices)
  float _yUp;
  float _zUp;
  float _xUpStart;         // Initial reference "up" vector
  float _yUpStart;
  float _zUpStart;
  unsigned long _prevMicros;  // For delta time calculation
  float _tumbleThreshold;
  bool _tumbleDetected;
  bool _tumbleReferenceSet;
  bool _firstUpdateAfterReset;  // Flag to skip first update with bad deltaTime
  
  // BNO055 Register addresses
  static const uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
  static const uint8_t BNO055_AXIS_MAP_CONFIG_ADDR = 0x41;
  static const uint8_t BNO055_AXIS_MAP_SIGN_ADDR = 0x42;
  
  // Internal helper functions
  BNO055_Orientation detectOrientation();
  void applyAxisRemap();
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  void updateUpVector(float deltaTime);  // Apply rotation matrices
};

#endif // BNO055_MOTION_H