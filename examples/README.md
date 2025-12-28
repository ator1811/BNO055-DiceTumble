# Dice Roll Detection Examples

Three example sketches demonstrating different approaches to dice roll detection using the BNO055_Motion library.

---

## 📁 Example Sketches

### 1. `Dice_Roll_Detection.ino` ⭐ RECOMMENDED

**Best for:** Production use, complete dice rolling system

**Features:**
- Clean state machine architecture (WAITING → TUMBLING → SETTLING → STABLE)
- Visual angle display with progress bars during rolling
- Automatic reset after each roll
- Displays final orientation and calibration status
- Easy to integrate into larger projects

**Output Example:**
```
🎲 TUMBLE DETECTED! Roll in progress...
─────────────────────────────────────
Rotation: 12.3° ▓
Rotation: 45.7° ▓▓▓▓
Rotation: 78.2° ▓▓▓▓▓▓▓

⏸  Motion stopped, waiting for stability...
✓ Stable and flat detected!

╔════════════════════════════════════════╗
║          ROLL COMPLETE!                ║
╚════════════════════════════════════════╝

Total rotation: 78.2°
Landed on: X+ UP
```

**When to use:**
- Building a dice rolling application
- Need clean state transitions
- Want visual feedback during rolls
- Production/deployment code

---

### 2. `Simple_Tumble_Monitor.ino`

**Best for:** Testing, debugging, simple applications

**Features:**
- Minimal code, easy to understand
- Continuous angle output during tumbling
- Simple tumble start/stop detection
- Quick testing of tumble threshold settings

**Output Example:**
```
>>> TUMBLE STARTED <<<
Angle: 15.3°
Angle: 34.8°
Angle: 52.1°
Angle: 67.4°
>>> STABLE & FLAT <<<
Final angle: 67.4°
Orientation: Y+ UP

--- Resetting for next tumble ---
```

**When to use:**
- Learning how the library works
- Testing tumble detection thresholds
- Debugging sensor behavior
- Quick prototyping

---

### 3. `Advanced_Dice_Tracker.ino`

**Best for:** Analysis, testing, quality assurance

**Features:**
- Detailed roll statistics tracking
- Max angle per roll and overall session
- Roll duration measurement
- Face landing frequency distribution
- Average rotation calculations
- Real-time calibration monitoring
- Multiple roll session tracking

**Output Example:**
```
┌───────────────────────────────────────────┐
│           ROLL COMPLETE                   │
└───────────────────────────────────────────┘

  This Roll:
    Max rotation: 94.3°
    Duration: 1.45 seconds
    Samples: 29
    Landed on: Z+ UP (Vertical - Normal)
    Calibration: S:3 G:3 A:3 M:3

  Session Statistics:
    Total rolls: 12
    Average rotation: 78.4°
    Max rotation ever: 142.7°
    Total time rolling: 18.3 sec

  Face Distribution:
    Z+: 3 (25.0%)
    Z-: 1 (8.3%)
    X+: 2 (16.7%)
    X-: 2 (16.7%)
    Y+: 3 (25.0%)
    Y-: 1 (8.3%)
```

**When to use:**
- Analyzing dice rolling behavior
- Testing sensor accuracy over time
- Quality assurance / validation
- Research and development
- Checking for dice bias

---

## 🎯 Quick Start Guide

### 1. Choose Your Example

| Need | Use This |
|------|----------|
| Production dice rolling system | `Dice_Roll_Detection.ino` |
| Quick test/prototype | `Simple_Tumble_Monitor.ino` |
| Analysis/statistics | `Advanced_Dice_Tracker.ino` |

### 2. Upload and Run

1. Open the sketch in Arduino IDE
2. Select your board (ESP32, Arduino, etc.)
3. Select the correct COM port
4. Upload
5. Open Serial Monitor (115200 baud)

### 3. Basic Usage

**All examples follow the same pattern:**
1. Place dice/sensor on table
2. Pick it up and roll/tilt
3. Place back on table
4. Wait for stable detection
5. Repeat!

---

## ⚙️ Configuration Options

### Tumble Threshold

Adjust sensitivity in all examples:

```cpp
const float TUMBLE_THRESHOLD = 0.707;  // 45 degrees

// Common values:
// 0.866 = 30° (very sensitive)
// 0.707 = 45° (balanced - RECOMMENDED)
// 0.500 = 60° (less sensitive)
// 0.000 = 90° (only quarter turns)
```

### Update Interval

Control how often readings are taken:

```cpp
const int UPDATE_INTERVAL = 100;  // milliseconds

// Recommendations:
// 50ms  = Fast (20 updates/sec) - good for tracking
// 100ms = Normal (10 updates/sec) - balanced
// 200ms = Slow (5 updates/sec) - battery saving
```

---

## 🔍 Understanding the State Flow

### State Machine (Dice_Roll_Detection)

```
     ┌──────────┐
     │ WAITING  │  ← Reset point
     └────┬─────┘
          │ tumbled()
          ▼
     ┌──────────┐
     │ TUMBLING │  ← Output angles here
     └────┬─────┘
          │ !moving()
          ▼
     ┌──────────┐
     │ SETTLING │  ← Waiting for stability
     └────┬─────┘
          │ stable() && on_table()
          ▼
     ┌──────────┐
     │  STABLE  │  ← Show results, then reset
     └──────────┘
```

### Key Detection Points

1. **Tumble Start:** `sensor.tumbled()` returns true
   - Rotation angle exceeds threshold
   - Indicates significant orientation change

2. **Motion Stop:** `!sensor.moving()` returns true
   - Acceleration change below threshold
   - Dice has stopped rolling

3. **Stable Landing:** `sensor.stable() && sensor.on_table()` returns true
   - Stable for required number of samples
   - Aligned with one of the axes (flat on table)

---

## 🐛 Troubleshooting

### Problem: No tumble detected when rolling

**Solutions:**
- Lower the threshold: `sensor.setTumbleThreshold(0.866);` (30°)
- Check sensor calibration status
- Ensure sensor is properly mounted

### Problem: False tumbles when stationary

**Solutions:**
- Raise the threshold: `sensor.setTumbleThreshold(0.5);` (60°)
- Check for vibration/movement in mounting
- Verify gyroscope readings are near zero when still

### Problem: Never reaches stable state

**Solutions:**
- Check `sensor.on_table()` - must be flat on a face
- Verify accelerometer readings show gravity (~9.8 m/s²)
- May need to adjust orientation thresholds

### Problem: Inconsistent results

**Solutions:**
- Calibrate sensor thoroughly (move in figure-8 pattern)
- Check calibration status: all values should be 3
- Ensure stable power supply
- Verify I2C connections are solid

---

## 📊 Expected Behavior

### Normal Dice Roll

```
State     | Angle  | Moving | Stable | On Table
----------|--------|--------|--------|----------
WAITING   | 0.0°   | No     | Yes    | Yes
TUMBLING  | 15.3°  | Yes    | No     | No
TUMBLING  | 45.7°  | Yes    | No     | No
TUMBLING  | 72.1°  | Yes    | No     | No
SETTLING  | 72.1°  | No     | No     | No
SETTLING  | 72.1°  | No     | No     | Yes
STABLE    | 72.1°  | No     | Yes    | Yes
```

### Typical Angle Ranges

- **Light tilt:** 20-40°
- **Normal roll:** 60-120°
- **Vigorous roll:** 120-180°
- **Multiple tumbles:** 180°+

---

## 🎨 Customization Ideas

### Add Sound Effects
```cpp
if (sensor.tumbled()) {
    tone(BUZZER_PIN, 1000, 100);  // Beep on tumble start
}
```

### Add LED Indicators
```cpp
if (state == TUMBLING) {
    digitalWrite(LED_PIN, HIGH);  // LED on during roll
} else {
    digitalWrite(LED_PIN, LOW);
}
```

### Log to SD Card
```cpp
if (state == STABLE) {
    logFile.print(millis());
    logFile.print(",");
    logFile.print(currentRoll.maxAngle);
    logFile.print(",");
    logFile.println(sensor.getOrientationString());
}
```

### Wireless Reporting
```cpp
if (state == STABLE) {
    // Send via WiFi/Bluetooth
    sendRollData(currentRoll.maxAngle, orientation);
}
```

---

## 📚 Related Documentation

- **ROTATION_MATRIX_EXPLANATION.md** - How the math works
- **TROUBLESHOOTING.md** - Detailed problem solving
- **NORMALIZATION_VERIFICATION.md** - Technical accuracy details
- **FIX_SUMMARY.md** - Gyroscope unit conversion explanation

---

## 💡 Tips for Best Results

1. **Calibrate thoroughly** - Move sensor in figure-8 pattern until all calibration values are 3
2. **Mount securely** - Vibration causes false triggers
3. **Choose appropriate threshold** - 45° (0.707) works for most dice
4. **Update regularly** - Call `sensor.update()` every 50-100ms
5. **Test extensively** - Try different roll styles to verify behavior

---

## 🎲 Ready to Roll!

Pick an example, upload it, and start detecting dice rolls with precision!
