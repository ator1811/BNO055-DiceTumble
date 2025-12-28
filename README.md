# BNO055_DiceTumble Library

Advanced motion detection library for BNO055 IMU sensor featuring gyroscope-based tumble detection that's insensitive to vertical axis rotation. Designed specifically for dice rolling applications and sophisticated orientation sensing.

## 🎯 Key Features

- **Rotation Matrix-Based Tumble Detection** - Accurately tracks 3D orientation changes using gyroscope integration
- **Vertical Rotation Insensitive** - Ignores spinning on a table, only detects actual rolling motion
- **Comprehensive Motion Detection** - Moving/stable state detection with tunable thresholds
- **Orientation Sensing** - Detect which face is down (6 orientations + tilted state)
- **Easy Calibration** - Built-in calibration status monitoring
- **Customizable Axis Remapping** - Configure sensor orientation to match your mounting
- **Production Ready** - Clean API with extensive debugging capabilities

## 📋 Requirements

### Hardware
- BNO055 9-DOF Absolute Orientation IMU
- ESP32, Arduino, or compatible microcontroller
- I2C connection to BNO055

### Software Dependencies
- [Adafruit BNO055 Library](https://github.com/adafruit/Adafruit_BNO055)
- [Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor)

## 🚀 Quick Start

### Installation

1. **Arduino Library Manager** (recommended)
   - Open Arduino IDE
   - Go to Sketch → Include Library → Manage Libraries
   - Search for "BNO055_DiceTumble"
   - Click Install

2. **Manual Installation**
   - Download this repository as ZIP
   - In Arduino IDE: Sketch → Include Library → Add .ZIP Library
   - Select the downloaded ZIP file

### Basic Usage

```cpp
#include <BNO055_Motion.h>

BNO055_Motion sensor;

void setup() {
  Serial.begin(115200);
  
  // Initialize sensor
  if (!sensor.init(true)) {  // true = verbose output
    Serial.println("Failed to initialize BNO055!");
    while (1) delay(10);
  }
  
  Serial.println("BNO055 initialized successfully!");
}

void loop() {
  // Update sensor readings (call every 50-100ms)
  sensor.update();
  
  // Check for motion
  if (sensor.moving()) {
    Serial.println("Motion detected!");
  }
  
  // Check for stability
  if (sensor.stable() && sensor.on_table()) {
    Serial.print("Stable on: ");
    Serial.println(sensor.getOrientationString());
  }
  
  delay(100);
}
```

### Dice Roll Detection

```cpp
#include <BNO055_Motion.h>

BNO055_Motion sensor;

void setup() {
  Serial.begin(115200);
  sensor.init(true);
  sensor.setTumbleThreshold(0.707);  // 45 degrees
  sensor.resetTumbleDetection();
}

void loop() {
  sensor.update();
  
  // Detect when dice is rolled
  if (sensor.tumbled()) {
    Serial.print("Rolling... ");
    Serial.print(sensor.getTumbleAngle());
    Serial.println("°");
  }
  
  // Detect when roll is complete
  if (sensor.stable() && sensor.on_table()) {
    Serial.print("Landed on: ");
    Serial.println(sensor.getOrientationString());
    sensor.resetTumbleDetection();  // Ready for next roll
    delay(1000);  // Debounce
  }
  
  delay(100);
}
```

## 📚 API Reference

### Core Functions

#### Initialization
```cpp
bool init(bool verbose = false)
```
Initialize the BNO055 sensor with custom axis remapping. Returns `true` if successful.

#### Sensor Updates
```cpp
void update()
```
Update all sensor readings. **Must be called regularly** (every 50-100ms recommended).

#### Motion Detection
```cpp
boolean moving()        // Returns true if sensor is moving
boolean stable()        // Returns true if sensor is stable (not moving)
boolean on_table()      // Returns true if sensor is flat on a face
```

#### Orientation Detection
```cpp
BNO055_Orientation orientation()     // Get current orientation enum
String getOrientationString()        // Get orientation as readable string
```

Possible orientations:
- `ORIENTATION_Z_UP` - Normal vertical position
- `ORIENTATION_Z_DOWN` - Upside down
- `ORIENTATION_X_UP` - Tilted
- `ORIENTATION_X_DOWN` - Tilted opposite
- `ORIENTATION_Y_UP` - Tilted sideways
- `ORIENTATION_Y_DOWN` - Tilted opposite sideways
- `ORIENTATION_TILTED` - Not aligned with any axis

#### Gyroscope & Accelerometer
```cpp
float gyroX(), gyroY(), gyroZ()      // Gyroscope in deg/s
float accelX(), accelY(), accelZ()   // Accelerometer in m/s²
float getAccelMagnitude()            // Total acceleration magnitude
```

### Tumble Detection

#### Basic Usage
```cpp
void resetTumbleDetection()          // Capture current orientation as reference
boolean tumbled()                    // Check if tumbled beyond threshold
float getTumbleAngle()               // Get rotation angle in degrees
```

#### Configuration
```cpp
void setTumbleThreshold(float threshold)
```
Set tumble detection threshold (cosine of angle):
- `0.866` = 30° (very sensitive)
- `0.707` = 45° (balanced) **← RECOMMENDED**
- `0.500` = 60° (less sensitive)
- `0.000` = 90° (only quarter turns)

### Tunable Parameters

```cpp
void setMotionThreshold(float threshold)     // Default: 0.5 m/s²
void setStableThreshold(float threshold)     // Default: 0.15 m/s²
void setStableCount(int count)               // Default: 5 samples
void setOrientationThresholds(float minG, float maxG, float maxOther)
```

### Calibration

```cpp
void getCalibration(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
boolean isCalibrated()               // Returns true if all ≥ 2
```

### Debug Functions

```cpp
void printDebugInfo()                        // Print comprehensive debug info
float getDebugDotProduct()                   // Get current dot product
void getDebugUpVector(float* x, float* y, float* z)
void getDebugUpStart(float* x, float* y, float* z)
```

## 📖 Examples

The library includes three comprehensive example sketches:

### 1. Dice_Roll_Detection ⭐ **RECOMMENDED**
Complete dice rolling system with state machine architecture, visual feedback, and automatic reset.

**Best for:** Production applications, complete dice systems

### 2. Simple_Tumble_Monitor
Minimal code demonstrating basic tumble detection with continuous angle output.

**Best for:** Learning, testing, quick prototyping

### 3. Advanced_Dice_Tracker
Detailed statistics tracking with roll analysis, face distribution, and session monitoring.

**Best for:** Analysis, quality assurance, research

See `examples/` folder for complete code and detailed documentation.

## 🔧 Configuration Tips

### Sensor Mounting
The library assumes **vertical mounting** (sensor standing on edge). The axis remapping automatically swaps X and Z axes for this configuration.

### Calibration
For best results, calibrate the sensor by moving it in a figure-8 pattern until all calibration values reach 3:
```cpp
uint8_t system, gyro, accel, mag;
sensor.getCalibration(&system, &gyro, &accel, &mag);
// All should be 3 for optimal performance
```

### Update Rate
Call `sensor.update()` regularly:
- 50ms (20 Hz) - Fast tracking
- 100ms (10 Hz) - Balanced **← RECOMMENDED**
- 200ms (5 Hz) - Battery saving

## 🎲 How It Works

### Rotation Matrix Method
The library uses rotation matrices to track the "up" vector through 3D space:

1. **Reference Capture** - `resetTumbleDetection()` saves the current gravity vector as the initial "up" direction
2. **Gyroscope Integration** - Each update applies rotation based on gyroscope readings
3. **Dot Product Comparison** - Compares current vs. initial up vectors using dot product
4. **Angle Calculation** - `arccos(dot_product)` gives the total rotation angle

### Why Gyroscope Instead of Accelerometer?
- **Shaking produces:** High acceleration, low angular velocity
- **Rolling produces:** Measurable rotation, varying acceleration
- Gyroscope-based detection eliminates false positives from shaking

### Vertical Rotation Insensitivity
The dot product only measures the **angle between vectors**, not rotation around them. Spinning on a table rotates around the up vector but doesn't change it, so the dot product remains ~1.0 and no tumble is detected.

## 🐛 Troubleshooting

### No tumble detected when rolling
- Lower threshold: `sensor.setTumbleThreshold(0.866);` (30°)
- Check calibration status
- Verify sensor mounting orientation

### False tumbles when stationary
- Raise threshold: `sensor.setTumbleThreshold(0.5);` (60°)
- Check for vibration in mounting
- Verify gyroscope readings are near zero when still

### Never reaches stable state
- Ensure sensor is flat on a face (`on_table()` must be true)
- Verify accelerometer shows gravity (~9.8 m/s²)
- May need to adjust orientation thresholds

### Inconsistent results
- Fully calibrate sensor (all values should be 3)
- Ensure stable power supply
- Verify solid I2C connections

## 📊 Performance

- **Update Rate:** 10-20 Hz (50-100ms intervals)
- **Angle Resolution:** ~1-2 degrees
- **Tumble Detection Latency:** 100-200ms
- **Stability Detection:** 0.5-1.0 seconds

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## 📄 License

MIT License - see LICENSE file for details

## 🙏 Acknowledgments

- Built on top of the excellent [Adafruit BNO055 library](https://github.com/adafruit/Adafruit_BNO055)
- Inspired by the need for reliable dice roll detection in quantum dice projects

## 📞 Support

For issues, questions, or feature requests, please use the GitHub issue tracker.

---

**Happy rolling!** 🎲
