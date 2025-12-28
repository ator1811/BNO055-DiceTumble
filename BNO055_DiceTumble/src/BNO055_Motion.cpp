/*
  BNO055_Motion.cpp - Implementation of BNO055 motion detection wrapper
*/

#include "BNO055_Motion.h"

// Constructor
BNO055_Motion::BNO055_Motion() : bno(55) {
  // Initialize default thresholds
  _motionThreshold = 0.5;        // m/s²
  _stableThreshold = 0.15;       // m/s²
  _stableCountRequired = 5;      // samples
  _flatGravityMin = 9.0;         // m/s²
  _flatGravityMax = 10.5;        // m/s²
  _flatOtherAxisMax = 2.0;       // m/s²
  
  // Default axis remap: X↔Z swap with Y unchanged
  _axisRemapConfig = 0x06;       // Z gets X, Y gets Y, X gets Z
  _axisRemapSign = 0x06;         // Sign configuration
  
  // Initialize tumble detection (rotation matrix-based)
  _xUp = 0.0;
  _yUp = 0.0;
  _zUp = 1.0;                    // Default to Z-up
  _xUpStart = 0.0;
  _yUpStart = 0.0;
  _zUpStart = 1.0;
  _prevMicros = 0;
  _tumbleThreshold = 0.707;      // cos(45°) by default
  _tumbleDetected = false;
  _tumbleReferenceSet = false;
  _firstUpdateAfterReset = false;
  
  // Initialize state
  _prevAccelMag = 0;
  _currentAccelMag = 0;
  _accelChange = 0;
  _isMoving = false;
  _stableCounter = 0;
  _currentOrientation = ORIENTATION_UNKNOWN;
}

// ============================================
// CORE FUNCTIONS
// ============================================

bool BNO055_Motion::init(bool verbose) {
  // Initialize I2C and BNO055
  if (verbose) Serial.print("Initializing BNO055... ");
  
  if (!bno.begin()) {
    if (verbose) Serial.println("FAILED! Sensor not detected.");
    return false;  // Sensor not detected
  }
  
  if (verbose) Serial.println("detected.");
  delay(100);
  
  // Apply custom axis remapping
  if (verbose) Serial.print("Applying axis remapping... ");
  applyAxisRemap();
  if (verbose) Serial.println("done.");
  
  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);
  
  delay(100);
  
  // Wait for sensor to produce sensible readings
  // This is especially important with ESP32 and I2C initialization
  if (verbose) Serial.print("Waiting for stable readings... ");
  
  unsigned long startTime = millis();
  const unsigned long timeout = 5000;  // 5 second timeout
  bool sensibleReading = false;
  int attempts = 0;
  
  while (!sensibleReading && (millis() - startTime < timeout)) {
    // Read acceleration
    _accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float mag = sqrt(_accel.x()*_accel.x() + _accel.y()*_accel.y() + _accel.z()*_accel.z());
    
    // Check if reading is sensible (close to gravity, not zero or wildly off)
    // Valid range: 7-12 m/s² (allows for some movement during init)
    if (mag > 7.0 && mag < 12.0) {
      sensibleReading = true;
      _prevAccelMag = mag;
      _currentAccelMag = mag;
      if (verbose) {
        Serial.print("OK (");
        Serial.print(mag, 2);
        Serial.print(" m/s² after ");
        Serial.print(attempts);
        Serial.println(" attempts)");
      }
    } else {
      attempts++;
      if (verbose && attempts % 10 == 0) Serial.print(".");
      delay(50);  // Wait a bit before next reading
    }
  }
  
  if (!sensibleReading) {
    if (verbose) {
      Serial.println("\nFAILED! Timeout - sensor not producing valid readings.");
      Serial.println("Check connections and try again.");
    }
    return false;  // Timeout - sensor not producing valid readings
  }
  
  // Do a few more updates to stabilize the baseline
  if (verbose) Serial.print("Stabilizing baseline... ");
  for (int i = 0; i < 5; i++) {
    _accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    _gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    _currentAccelMag = sqrt(_accel.x()*_accel.x() + _accel.y()*_accel.y() + _accel.z()*_accel.z());
    _prevAccelMag = _currentAccelMag;
    delay(20);
  }
  if (verbose) Serial.println("done.");
  
  // Initialize tumble detection with current orientation
  if (verbose) Serial.print("Initializing tumble detection... ");
  resetTumbleDetection();
  if (verbose) Serial.println("done.");
  
  if (verbose) Serial.println("✓ BNO055 initialization complete!");
  
  return true;  // Success
}

void BNO055_Motion::update() {
  // Calculate delta time
  unsigned long currentMicros = micros();
  float deltaTime = (currentMicros - _prevMicros) * 1e-6;  // Convert to seconds
  _prevMicros = currentMicros;
  
  // Read sensor data
  _accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  _gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // Calculate acceleration magnitude
  _currentAccelMag = sqrt(_accel.x()*_accel.x() + _accel.y()*_accel.y() + _accel.z()*_accel.z());
  _accelChange = abs(_currentAccelMag - _prevAccelMag);
  
  // Motion detection logic
  if (_accelChange > _motionThreshold) {
    // Significant change = motion detected
    _isMoving = true;
    _stableCounter = 0;
  } 
  else if (_accelChange < _stableThreshold) {
    // Very little change = potentially stable
    _stableCounter++;
    
    if (_isMoving && _stableCounter >= _stableCountRequired) {
      // Been stable long enough
      _isMoving = false;
    }
  } 
  else {
    // In between - small movements
    if (_stableCounter > 0) {
      _stableCounter--;  // Slowly decay stable counter
    }
  }
  
  // Detect orientation
  _currentOrientation = detectOrientation();
  
  // Update up vector using rotation matrices (if reference is set)
  if (_tumbleReferenceSet) {
    // Skip first update after reset to avoid bad deltaTime
    if (_firstUpdateAfterReset) {
      _firstUpdateAfterReset = false;
      _prevMicros = currentMicros;  // Reset timing
    }
    else if (deltaTime > 0.0 && deltaTime < 1.0) {
      updateUpVector(deltaTime);
      
      // Check for tumble by comparing current up vector with initial reference
      float dotProduct = _xUp * _xUpStart + _yUp * _yUpStart + _zUp * _zUpStart;
      
      // Clamp to valid range for acos
      dotProduct = constrain(dotProduct, -1.0, 1.0);
      
      // Check if tumbled beyond threshold
      if (dotProduct < _tumbleThreshold) {
        _tumbleDetected = true;
      }
    }
  }
  
  // Update previous magnitude for next iteration
  _prevAccelMag = _currentAccelMag;
}

boolean BNO055_Motion::moving() {
  return _isMoving;
}

boolean BNO055_Motion::stable() {
  return !_isMoving && (_stableCounter >= _stableCountRequired);
}

boolean BNO055_Motion::on_table() {
  // Check if orientation is one of the flat positions (not tilted or unknown)
  return (_currentOrientation != ORIENTATION_UNKNOWN && 
          _currentOrientation != ORIENTATION_TILTED);
}

BNO055_Orientation BNO055_Motion::orientation() {
  return _currentOrientation;
}

float BNO055_Motion::gyroX() {
  return _gyro.x();
}

float BNO055_Motion::gyroY() {
  return _gyro.y();
}

float BNO055_Motion::gyroZ() {
  return _gyro.z();
}

// ============================================
// ADDITIONAL USEFUL FUNCTIONS
// ============================================

float BNO055_Motion::accelX() {
  return _accel.x();
}

float BNO055_Motion::accelY() {
  return _accel.y();
}

float BNO055_Motion::accelZ() {
  return _accel.z();
}

float BNO055_Motion::getAccelMagnitude() {
  return _currentAccelMag;
}

float BNO055_Motion::getAccelChange() {
  return _accelChange;
}

String BNO055_Motion::getOrientationString() {
  switch (_currentOrientation) {
    case ORIENTATION_Z_UP:
      return "Z+ UP (Vertical - Normal)";
    case ORIENTATION_Z_DOWN:
      return "Z- UP (Vertical - Inverted)";
    case ORIENTATION_X_UP:
      return "X+ UP";
    case ORIENTATION_X_DOWN:
      return "X- UP";
    case ORIENTATION_Y_UP:
      return "Y+ UP";
    case ORIENTATION_Y_DOWN:
      return "Y- UP";
    case ORIENTATION_TILTED:
      return "TILTED (not aligned)";
    default:
      return "UNKNOWN";
  }
}

void BNO055_Motion::getCalibration(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  bno.getCalibration(system, gyro, accel, mag);
}

boolean BNO055_Motion::isCalibrated() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  return (system >= 2 && gyro >= 2 && accel >= 2 && mag >= 2);
}

// ============================================
// TUNABLE PARAMETERS
// ============================================

void BNO055_Motion::setMotionThreshold(float threshold) {
  _motionThreshold = threshold;
}

void BNO055_Motion::setStableThreshold(float threshold) {
  _stableThreshold = threshold;
}

void BNO055_Motion::setStableCount(int count) {
  _stableCountRequired = count;
}

void BNO055_Motion::setOrientationThresholds(float minGravity, float maxGravity, float maxOtherAxis) {
  _flatGravityMin = minGravity;
  _flatGravityMax = maxGravity;
  _flatOtherAxisMax = maxOtherAxis;
}

// ============================================
// AXIS REMAPPING
// ============================================

void BNO055_Motion::setAxisRemap(uint8_t config, uint8_t sign) {
  _axisRemapConfig = config;
  _axisRemapSign = sign;
  applyAxisRemap();
}

void BNO055_Motion::getAxisRemap(uint8_t* config, uint8_t* sign) {
  *config = readRegister(BNO055_AXIS_MAP_CONFIG_ADDR);
  *sign = readRegister(BNO055_AXIS_MAP_SIGN_ADDR);
}

// ============================================
// TUMBLE DETECTION FUNCTIONS (Rotation Matrix-based)
// ============================================

void BNO055_Motion::resetTumbleDetection() {
  // Get current acceleration (gravity) vector
  _accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  // Calculate magnitude
  float mag = sqrt(_accel.x()*_accel.x() + _accel.y()*_accel.y() + _accel.z()*_accel.z());
  
  #ifdef DEBUG_TUMBLE
  Serial.print("RESET: Accel:(");
  Serial.print(_accel.x(), 4);
  Serial.print(",");
  Serial.print(_accel.y(), 4);
  Serial.print(",");
  Serial.print(_accel.z(), 4);
  Serial.print(") Mag:");
  Serial.print(mag, 4);
  #endif
  
  // Normalize to unit vector (invert because gravity points down, we want "up")
  // Avoid division by zero
  if (mag > 0.1) {
    _xUpStart = -_accel.x() / mag;
    _yUpStart = -_accel.y() / mag;
    _zUpStart = -_accel.z() / mag;
    
    #ifdef DEBUG_TUMBLE
    Serial.print(" | UpStart:(");
    Serial.print(_xUpStart, 4);
    Serial.print(",");
    Serial.print(_yUpStart, 4);
    Serial.print(",");
    Serial.print(_zUpStart, 4);
    Serial.println(")");
    #endif
    
    // Initialize current up vector to same as start
    _xUp = _xUpStart;
    _yUp = _yUpStart;
    _zUp = _zUpStart;
    
    // Reset timing
    _prevMicros = micros();
    
    // Clear detection flags
    _tumbleDetected = false;
    _tumbleReferenceSet = true;
    _firstUpdateAfterReset = true;  // Skip first update to avoid bad deltaTime
  }
  #ifdef DEBUG_TUMBLE
  else {
    Serial.println(" | ERROR: Magnitude too low!");
  }
  #endif
}

boolean BNO055_Motion::tumbled() {
  return _tumbleDetected;
}

float BNO055_Motion::getTumbleAngle() {
  if (!_tumbleReferenceSet) {
    return 0.0;  // No reference set
  }
  
  // Calculate dot product between current and initial up vectors
  float dotProduct = _xUp * _xUpStart + _yUp * _yUpStart + _zUp * _zUpStart;
  
  // Clamp to valid range for acos
  dotProduct = constrain(dotProduct, -1.0, 1.0);
  
  // Convert to angle in degrees
  float angleRadians = acos(dotProduct);
  float angleDegrees = angleRadians * 57.2958;  // 180/PI
  
  return angleDegrees;
}

void BNO055_Motion::setTumbleThreshold(float threshold) {
  _tumbleThreshold = threshold;
  // Reset tumble detection when threshold changes
  _tumbleDetected = false;
}

// ============================================
// DEBUG FUNCTIONS
// ============================================

float BNO055_Motion::getDebugDotProduct() {
  if (!_tumbleReferenceSet) {
    return 1.0;  // No reference set, return 1.0
  }
  
  float dotProduct = _xUp * _xUpStart + _yUp * _yUpStart + _zUp * _zUpStart;
  return constrain(dotProduct, -1.0, 1.0);
}

void BNO055_Motion::getDebugUpVector(float* x, float* y, float* z) {
  *x = _xUp;
  *y = _yUp;
  *z = _zUp;
}

void BNO055_Motion::getDebugUpStart(float* x, float* y, float* z) {
  *x = _xUpStart;
  *y = _yUpStart;
  *z = _zUpStart;
}

void BNO055_Motion::printDebugInfo() {
  Serial.print("UpStart:(");
  Serial.print(_xUpStart, 4);
  Serial.print(", ");
  Serial.print(_yUpStart, 4);
  Serial.print(", ");
  Serial.print(_zUpStart, 4);
  Serial.print(") | Up:(");
  Serial.print(_xUp, 4);
  Serial.print(", ");
  Serial.print(_yUp, 4);
  Serial.print(", ");
  Serial.print(_zUp, 4);
  Serial.print(") | Gyro:(");
  Serial.print(_gyro.x(), 4);
  Serial.print(", ");
  Serial.print(_gyro.y(), 4);
  Serial.print(", ");
  Serial.print(_gyro.z(), 4);
  Serial.print(") | Dot:");
  Serial.print(getDebugDotProduct(), 4);
  Serial.print(" | Angle:");
  Serial.print(getTumbleAngle(), 2);
  Serial.println("°");
}

// ============================================
// PRIVATE HELPER FUNCTIONS
// ============================================

void BNO055_Motion::updateUpVector(float deltaTime) {
  // BNO055 outputs gyroscope in DEGREES per second, not radians!
  // Convert to radians per second, then calculate rotation angles

  
  float xRot = _gyro.x() * DEG_TO_RAD * deltaTime;
  float yRot = _gyro.y() * DEG_TO_RAD * deltaTime;
  float zRot = _gyro.z() * DEG_TO_RAD * deltaTime;
  
  // Debug: Print rotation info
  #ifdef DEBUG_TUMBLE
  Serial.print("dt=");
  Serial.print(deltaTime, 6);
  Serial.print(" | Gyro:(");
  Serial.print(_gyro.x(), 4);
  Serial.print(",");
  Serial.print(_gyro.y(), 4);
  Serial.print(",");
  Serial.print(_gyro.z(), 4);
  Serial.print(") | Rot:(");
  Serial.print(xRot, 6);
  Serial.print(",");
  Serial.print(yRot, 6);
  Serial.print(",");
  Serial.print(zRot, 6);
  Serial.print(")");
  #endif
  
  // Save pre-rotation up vector for debugging
  #ifdef DEBUG_TUMBLE
  float preX = _xUp, preY = _yUp, preZ = _zUp;
  #endif
  
  // Apply rotation matrices sequentially: X, then Y, then Z
  // This updates the current "up" vector based on the rotation
  
  // X-axis rotation matrix
  // Rotates around X-axis, affects Y and Z components
  float xUp = _xUp;
  float yUp = _yUp * cos(xRot) - _zUp * sin(xRot);
  float zUp = _yUp * sin(xRot) + _zUp * cos(xRot);
  
  _xUp = xUp;
  _yUp = yUp;
  _zUp = zUp;
  
  // Y-axis rotation matrix
  // Rotates around Y-axis, affects X and Z components
  xUp = _xUp * cos(yRot) + _zUp * sin(yRot);
  yUp = _yUp;
  zUp = -_xUp * sin(yRot) + _zUp * cos(yRot);
  
  _xUp = xUp;
  _yUp = yUp;
  _zUp = zUp;
  
  // Z-axis rotation matrix
  // Rotates around Z-axis, affects X and Y components
  xUp = _xUp * cos(zRot) - _yUp * sin(zRot);
  yUp = _xUp * sin(zRot) + _yUp * cos(zRot);
  zUp = _zUp;
  
  _xUp = xUp;
  _yUp = yUp;
  _zUp = zUp;
  
  // Calculate magnitude before normalization
  float magnitude = sqrt(_xUp * _xUp + _yUp * _yUp + _zUp * _zUp);
  
  #ifdef DEBUG_TUMBLE
  Serial.print(" | Pre:(");
  Serial.print(preX, 4);
  Serial.print(",");
  Serial.print(preY, 4);
  Serial.print(",");
  Serial.print(preZ, 4);
  Serial.print(") | Post:(");
  Serial.print(_xUp, 4);
  Serial.print(",");
  Serial.print(_yUp, 4);
  Serial.print(",");
  Serial.print(_zUp, 4);
  Serial.print(") | Mag:");
  Serial.print(magnitude, 6);
  #endif
  
  // CRITICAL: Renormalize the up vector to prevent drift
  // Floating-point errors accumulate, causing magnitude to drift from 1.0
  // This would make dot product calculations unreliable
  if (magnitude > 0.01) {  // Avoid division by zero
    _xUp /= magnitude;
    _yUp /= magnitude;
    _zUp /= magnitude;
  }
  
  #ifdef DEBUG_TUMBLE
  // Calculate dot product for debugging
  float dot = _xUp * _xUpStart + _yUp * _yUpStart + _zUp * _zUpStart;
  Serial.print(" | Dot:");
  Serial.println(dot, 4);
  #endif
}

BNO055_Orientation BNO055_Motion::detectOrientation() {
  float x = _accel.x();
  float y = _accel.y();
  float z = _accel.z();
  
  // Note: Accelerometer reads NEGATIVE when axis points UP (gravity pulls down)
  // and POSITIVE when axis points DOWN (accelerating toward ground)
  
  // Check which axis is aligned with gravity
  bool xDown = (abs(x) > _flatGravityMin && abs(x) < _flatGravityMax);
  bool yDown = (abs(y) > _flatGravityMin && abs(y) < _flatGravityMax);
  bool zDown = (abs(z) > _flatGravityMin && abs(z) < _flatGravityMax);
  
  // Z-axis aligned (physical X+ up = normal vertical)
  if (zDown && abs(x) < _flatOtherAxisMax && abs(y) < _flatOtherAxisMax) {
    return (z > 0) ? ORIENTATION_Z_UP : ORIENTATION_Z_DOWN;
  }
  
  // X-axis aligned (tilted toward physical Z direction)
  if (xDown && abs(y) < _flatOtherAxisMax && abs(z) < _flatOtherAxisMax) {
    return (x > 0) ? ORIENTATION_X_UP : ORIENTATION_X_DOWN;
  }
  
  // Y-axis aligned (tilted sideways)
  if (yDown && abs(x) < _flatOtherAxisMax && abs(z) < _flatOtherAxisMax) {
    return (y > 0) ? ORIENTATION_Y_UP : ORIENTATION_Y_DOWN;
  }
  
  // Not aligned with any axis
  return ORIENTATION_TILTED;
}

void BNO055_Motion::applyAxisRemap() {
  // Must be in CONFIG mode to change axis remap
  writeRegister(BNO055_OPR_MODE_ADDR, 0x00);
  delay(25);
  
  // Write custom axis remap configuration
  writeRegister(BNO055_AXIS_MAP_CONFIG_ADDR, _axisRemapConfig);
  delay(10);
  
  // Write custom axis sign configuration
  writeRegister(BNO055_AXIS_MAP_SIGN_ADDR, _axisRemapSign);
  delay(10);
  
  // Switch to NDOF mode (all sensors + fusion)
  writeRegister(BNO055_OPR_MODE_ADDR, 0x0C);
  delay(25);
}

uint8_t BNO055_Motion::readRegister(uint8_t reg) {
  uint8_t value;
  Wire.beginTransmission(BNO055_ADDRESS_A);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS_A, (uint8_t)1);
  value = Wire.read();
  return value;
}

void BNO055_Motion::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDRESS_A);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  delay(2);
}