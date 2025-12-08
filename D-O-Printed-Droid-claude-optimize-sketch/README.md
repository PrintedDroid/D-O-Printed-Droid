# D-O Self-Balancing Droid - Universal Controller

![Version](https://img.shields.io/badge/version-3.2.2-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![License](https://img.shields.io/badge/license-MIT-orange.svg)

**Universal control system for self-balancing D-O droid with flexible input configuration**

This optimized Arduino sketch provides complete control over a self-balancing D-O droid (from Star Wars) with two simple setup modes for maximum flexibility in remote control configuration.

## 📋 Table of Contents

- [Changelog](#-changelog)
- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Setup Modes](#-setup-modes)
- [Pin Configuration](#-pin-configuration)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [Usage](#-usage)
- [Troubleshooting](#-troubleshooting)
- [Credits](#-credits)

---

## 📝 Changelog

### Version 3.2.2 (November 2025)

**Critical Performance Fixes for PID Control**

This release fixes critical timing and performance issues that could cause unstable balancing.

#### 🔧 Critical Fixes

**1. micros() instead of millis() for PID Timing**
- **Problem**: Fast loops caused `elapsed_time = 0`, leading to division by zero in D-component
- **Solution**: Use `micros()` (microsecond precision) instead of `millis()` (millisecond precision)
- **Impact**: PID controller now works correctly at all loop speeds
- **Result**: Much more stable balancing behavior

**2. I2C Speed Increased to 400kHz**
- **Problem**: Default 100kHz I2C was slow for MPU6050 readings
- **Solution**: `Wire.setClock(400000)` for Fast Mode I2C
- **Impact**: 4x faster IMU data reading
- **Result**: Faster loop times, better balance response

**3. Replaced pow() with Direct Multiplication**
- **Problem**: `pow(x, 2)` is extremely slow (called 4x per loop)
- **Solution**: Changed to `x * x` (simple multiplication)
- **Impact**: Significant CPU time savings
- **Result**: Faster loops, more responsive control

#### ✅ Performance Improvements

- **Loop Speed**: Expect 50-100% faster loop times
- **PID Accuracy**: D-component now calculates correctly at all speeds
- **IMU Latency**: Reduced by ~75%
- **CPU Load**: Lower CPU usage leaves headroom for other tasks

#### ⚠️ Important Notes

- These fixes are **critical** for stable balancing
- Update recommended for all users
- No configuration changes needed
- Fully backward compatible with existing EEPROM settings

**Credit**: Analysis and recommendations by external code review

---

### Version 3.2.1 (November 2025)

**Live Operation Improvements & Safety Features**

Major improvements for real-world operation without Serial Monitor.

#### 🎯 New Features

**Performance Improvements:**
- ⚡ **Faster Startup**: No 3-second delay when Serial Monitor not connected
- ⚡ **2.5x Faster PWM Reading**: Reduced pulseIn() timeout from 25ms to 10ms
- ⚡ **Loop Frequency Monitoring**: Real-time performance tracking for PID tuning

**Safety Features:**
- 🛡️ **IMU Error Detection**: Automatic detection of MPU6050 connection issues
- 🛡️ **Watchdog Timer**: Auto-recovery from system hangs (2-second timeout)
- 🛡️ **Battery Recovery** (optional): Hysteresis prevents repeated emergency stops
- 🛡️ **Graceful Signal Loss**: Keeps balancing at neutral instead of hard stop

**Audio Feedback:**
- 🔊 **System Ready Sound**: Track 24 plays when D-O is ready
- 🔊 **Signal Lost Warning**: Track 25 alerts when RC signal is lost

**Code Quality:**
- ✨ Magic numbers converted to named constants
- ✨ Improved error handling throughout

#### ✅ Benefits

- **Better Live Operation**: Optimized for use without Serial Monitor
- **Improved Reliability**: Multiple safety layers prevent crashes
- **Better Performance**: Faster loop times, especially in Mode 0 (PWM)
- **User Feedback**: Audio cues replace missing visual indicators

---

### Version 3.2 (November 2025)

**Simplified to 2 Modes - Hybrid Mode Removed**

This update removes the unnecessary hybrid mode, simplifying the configuration.

#### 🎯 Major Changes

**Mode Simplification:**
- **Removed Mode 1 (Hybrid)** - it made no sense to have iBus for drive but PWM for sound
- Renumbered modes: Mode 0 (PWM), Mode 1 (iBus)
- If you have iBus, use **all** channels via iBus - much cleaner!
- Clearer, simpler configuration with only two logical choices

**Why Remove Hybrid Mode?**
- If you have an iBus receiver, you already have 10 channels available digitally
- Using PWM for some channels defeats the purpose of iBus
- More wiring without any benefit
- The hybrid mode added unnecessary complexity

#### New Mode Structure

**Mode 0 - PWM Only:**
- For standard PWM receivers
- All inputs via PWM (drive + sound)

**Mode 1 - iBus:**
- For iBus receivers (FlySky)
- All 10 channels via iBus

#### ✅ Benefits

- **Clearer Choice**: PWM receiver? Use Mode 0. iBus receiver? Use Mode 1.
- **Less Confusion**: No more deciding between hybrid and pure iBus
- **Cleaner Code**: Simplified logic, easier to maintain
- **Better Wiring**: No mixed PWM/iBus setups

#### ⚠️ Breaking Changes

- Mode 1 (Hybrid) no longer exists
- Old Mode 2 (Pure iBus) is now Mode 1
- Existing setups using Mode 2 need to change to Mode 1 in config menu

---

### Version 3.1 (November 2025)

**DFPlayer Now on Mega in ALL Modes - External Nano Removed**

- DFPlayer Mini now controlled by Mega in all modes
- External Arduino Nano completely removed from design
- Unified sound control across all input configurations

---

### Version 3.0 (November 2025)

**Initial Optimized Release with Multi-Mode Support**

- Three flexible setup modes (PWM/Hybrid/iBus)
- Interactive EEPROM configuration menu
- IMU calibration system
- Battery voltage monitoring
- Complete PID control system

---

## 🎯 Features

### Core Capabilities
- ✅ **MPU6050 IMU Self-Balancing** with PID control
- ✅ **Two Simple Setup Modes**: PWM Only or iBus
- ✅ **Integrated DFPlayer Sound System** (all modes - no external Nano!)
- ✅ **Multi-Servo Control** for head and mainbar movement (iBus mode only)
- ✅ **Battery Monitoring** with warnings and critical shutdown
- ✅ **Interactive Configuration Menu** via Serial Monitor
- ✅ **EEPROM Persistence** - all settings saved across reboots
- ✅ **IMU Calibration** for precise balance control

### Advanced Features
- Motor ramping for smooth acceleration
- Deadband and exponential RC input processing
- Mainbar auto-correction based on tilt angle
- Emergency stop on signal loss
- Configurable PID parameters
- Adjustable sound volume and intervals
- Real-time status monitoring

---

## 🔧 Hardware Requirements

### Core Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Microcontroller** | Arduino Mega 2560 | Required for Serial1 (iBus mode) |
| **IMU** | MPU6050 | I2C address 0x68 |
| **Motor Driver** | Cytron MD10C or similar | 2x for differential drive |
| **Sound Module** | DFPlayer Mini | With Micro SD card |
| **RC Receiver** | iBus-compatible OR PWM | FlySky FS-iA6B recommended |
| **Servos** | 4x Standard Servos | For head and mainbar |
| **Battery** | 2S LiPo (7.4V) | 2x in series = 14.8V total |
| **Voltage Divider** | 10kΩ + 3.3kΩ | For battery monitoring |
| **Speaker** | 8 Ohm, <3W | For DFPlayer |

### Optional Components
- 1kΩ resistor for DFPlayer TX line (recommended)
- Micro SD card (FAT16/FAT32, min. 512MB)

---

## 🔀 Setup Modes

The sketch supports two operation modes, selectable via the configuration menu:

### Mode 0: PWM Only
**Best for:** Users with traditional PWM remote control systems

**Inputs:**
- PWM on pins 2-3 for drive (tank-mixed)
- PWM on pins 14-17 (A0-A3) for sound switches
- DFPlayer on Mega (pins 7-8)
- **Servos: Connect DIRECTLY to RC receiver** (not to Arduino!)

**Advantages:**
- Compatible with any standard PWM receiver
- Simple setup
- No special RC equipment required
- Servos work independently (lower Arduino load)

**Limitations:**
- More cable connections (6 PWM wires to Arduino)
- No servo auto-correction features
- Servos bypass Arduino (direct RC receiver control)

---

### Mode 1: iBus (Recommended)
**Best for:** New installations and iBus-compatible receivers

**Inputs:**
- All 10 channels via iBus protocol (Pin 19)
- DFPlayer on Mega (pins 7-8)

**Advantages:**
- ✅ Cleanest wiring - **only 1 signal wire!**
- ✅ All 10 channels available
- ✅ Digital transmission - no interference
- ✅ Servo control included (CH3-6)
- ✅ All features available

**Requirements:**
- iBus-compatible receiver (e.g., FlySky FS-iA6B)

---

## 📍 Pin Configuration

### Motors (All Modes)
```
Pin 13 → Motor 1 Direction (DIR1)
Pin 12 → Motor 1 Speed (PWM1)
Pin 11 → Motor 2 Direction (DIR2)
Pin 10 → Motor 2 Speed (PWM2)
```

### Mode 0: PWM Only
```
DRIVE:
Pin 2  → Drive Channel 1 (PWM)
Pin 3  → Drive Channel 2 (PWM)

SOUND:
Pin 14 (A0) → Sound Mute (PWM)
Pin 15 (A1) → Sound Mode (PWM)
Pin 16 (A2) → Sound Mood (PWM, 3-position)
Pin 17 (A3) → Sound Squeak (PWM)
```

### Mode 1: iBus
```
ALL CHANNELS:
Pin 19 (Serial1 RX) → iBus Signal (all 10 channels)
```

### DFPlayer Mini (All Modes)
```
Pin 7 → DFPlayer RX
Pin 8 → DFPlayer TX
```

### Servos
```
MODE 0 (PWM Only):
  Connect servos DIRECTLY to RC receiver channels (CH3-6)
  Arduino does NOT control servos in this mode!

MODE 1 (iBus):
  Pin 4 → Mainbar Servo (controlled by Arduino)
  Pin 5 → Head Servo 1 - Pitch (controlled by Arduino)
  Pin 6 → Head Servo 2 - Yaw (controlled by Arduino)
  Pin 9 → Head Servo 3 - Roll (controlled by Arduino)
```

### I2C (MPU6050)
```
Pin 20 → SDA
Pin 21 → SCL
```

### Miscellaneous
```
Pin A15 → Battery Voltage Measurement
```

---

## 🔌 iBus Channel Mapping

*Relevant for Mode 1 only:*

| Channel | Function | Description |
|---------|----------|-------------|
| CH1 | Drive 1 | Left motor (tank-mixed) |
| CH2 | Drive 2 | Right motor (tank-mixed) |
| CH3 | Mainbar | Mainbar servo position |
| CH4 | Head 1 | Head pitch movement |
| CH5 | Head 2 | Head yaw movement |
| CH6 | Head 3 | Head roll movement |
| CH7 | Sound Mute | Sound on/off |
| CH8 | Sound Mode | Greeting / Default |
| CH9 | Sound Mood | Negative / Neutral / Positive (3-pos) |
| CH10 | Sound Squeak | Squeak sounds |

---

## 💾 SD Card File Structure

Audio files must be placed on the Micro SD card in the `/mp3/` folder:

```
/mp3/
  0001.mp3 - Startup sound ("battery charged")
  0002.mp3 - Default sound ("I am D-O")
  0003.mp3 - Greeting 1
  0004.mp3 - Greeting 2
  0005.mp3 - Greeting 3
  0006.mp3 - Negative 1
  0007.mp3 - Negative 2
  0008.mp3 - Negative 3
  0009.mp3 - Negative 4
  0010.mp3 - Positive 1
  0011.mp3 - Positive 2
  0012.mp3 - Positive 3
  0013.mp3 - Positive 4
  0014.mp3 - Positive 5
  0015.mp3 - Squeak 1
  0016.mp3 - Squeak 2
  0017.mp3 - Squeak 3
  0018.mp3 - Squeak 4
  0019.mp3 - Squeak 5
  0020.mp3 - Squeak 6
  0021.mp3 - (Reserved for future use)
  0022.mp3 - (Reserved for future use)
  0023.mp3 - Battery warning
  0024.mp3 - System ready (beep/chirp)
  0025.mp3 - Signal lost warning
```

**Important:**
- SD card must be formatted with FAT16 or FAT32
- Filenames must be exactly 4 digits (0001, 0002, etc.)
- Recommended format: MP3, 128-320 kbps

---

## 🚀 Installation

### 1. Required Arduino Libraries

Install the following libraries via Arduino Library Manager:

```
Required Libraries:
├── Wire (Arduino built-in)
├── IBusBM (by Bolder Flight Systems)
├── Servo (Arduino built-in)
├── EEPROM (Arduino built-in)
├── SoftwareSerial (Arduino built-in)
└── DFRobotDFPlayerMini (by DFRobot)
```

**Installation via Library Manager:**
1. Open Arduino IDE
2. Sketch → Include Library → Manage Libraries
3. Search for each library and install

### 2. Upload Sketch

1. Open `D_O_printed_droid_rc_optimized_v3.ino` in Arduino IDE
2. Select board: **Tools → Board → Arduino Mega 2560**
3. Select port: **Tools → Port → [Your COM Port]**
4. Upload: **Sketch → Upload** or Ctrl+U

### 3. Hardware Wiring

Connect all components according to the [Pin Configuration](#-pin-configuration).

**Important Notes:**
- MPU6050: Use short cables for I2C (max. 20cm)
- DFPlayer: 1kΩ resistor between Mega TX and DFPlayer RX recommended
- Battery: Voltage divider 10kΩ to GND, 3.3kΩ to A15
- Servos: Separate power supply recommended (BEC or dedicated PSU)

---

## ⚙️ Configuration

### Initial Setup

1. **Open Serial Monitor** (9600 baud)
2. After upload, you'll see:
   ```
   === D-O Universal Controller v3.2 ===
   Send 'm' for menu or 'c' for IMU calibration (3 sec)...
   ```

### IMU Calibration (Important!)

**Must be performed before first use:**

1. Send `c` at startup
2. Place D-O in balanced position
3. Keep absolutely still
4. Wait for calibration to complete
5. Values are automatically saved to EEPROM

### Configuration Menu

**Open menu:** Send `m` in Serial Monitor

```
=== CONFIGURATION MENU ===

1. Setup Mode (PWM/iBus)
2. PID Configuration
3. Battery Settings
4. Sound Settings
5. Feature Toggles
6. IMU Calibration
7. Show Current Status
8. Save and Exit
0. Exit without Saving
```

#### Change Setup Mode

1. Select option `1`
2. Choose mode:
   - `0` = PWM Only
   - `1` = iBus
3. **Restart required!**

#### Adjust PID Values

**Default values:**
- KP: 25.0
- KI: 0.0
- KD: 0.8
- Target Angle: -0.3

**Tuning:**
1. Select option `2` in menu
2. Enter values individually
3. Skip with empty line (Enter)

**Tips:**
- Oscillation: Decrease KP, increase KD
- Sluggish response: Increase KP
- Target Angle: Fine-tune balance (-1.0 to 1.0)

#### Battery Settings

**Default values (2S LiPo):**
- Warning Voltage: 6.8V (3.4V per cell)
- Critical Voltage: 6.4V (3.2V per cell)
- Voltage Divider: 4.03

**Calibrate voltage divider:**
1. Fully charge battery (8.4V)
2. Measure voltage with multimeter
3. Menu `3` → `Voltage Divider Factor`
4. Adjust until `Current battery reading` matches

#### Sound Settings

- **Volume:** 0-30 (default: 25)
- **Min Sound Interval:** Minimum time between sounds in ms

#### Feature Toggles

Enable/disable:
- Mainbar Auto-Correct (mainbar tilts with body)
- Motor Ramping (smooth acceleration)
- Battery Monitor (battery monitoring)
- Servo Control (enable/disable servos)

---

## 🎮 Usage

### First Startup

1. **Connect battery**
2. **Turn on RC transmitter**
3. **Arduino starts** - startup sound plays
4. **Wait for RC signal** (~10 seconds)
5. **"System ready!"** appears in Serial Monitor
6. **D-O self-balances automatically**

### Normal Operation

**Driving:**
- All modes: Tank-mixed control (two sticks/channels)
- Left channel: Left motor
- Right channel: Right motor

**Servos (Mode 1 Only):**
- CH3: Mainbar up/down
- CH4-6: Head movements

**Sound Control:**
- Mute: Sound on/off
- Mode: Switch between default and greeting
- Mood: 3 positions (Negative/Neutral/Positive)
- Squeak: Squeak sounds

### Emergency Stop

**Automatically triggered by:**
- RC signal loss (>500ms)
- Critical battery voltage

**Manually:**
- Send `m` and stay in menu
- System resumes when exiting menu

### Status Monitoring

In menu select option `7` for live status:
```
--- Current Status ---
Battery: 7.8V
Tilt Angle: -0.3
PID Output: 12.5
RC Drive: 1500 / 1500
Motors: 128 / 128
```

---

## 🔍 Troubleshooting

### Problem: D-O tips over / doesn't balance

**Solution:**
1. Perform IMU calibration (`c` at startup)
2. Adjust Target Angle in menu (usually between -1.0 and 1.0)
3. Check and adjust PID values
4. Verify MPU6050 wiring (I2C)

### Problem: No sound

**Solution:**
1. SD card formatted correctly? (FAT16/FAT32)
2. Sound files in `/mp3/` folder?
3. Filenames correct? (0001.mp3, etc.)
4. Check DFPlayer wiring
5. Increase volume in menu

### Problem: "EMERGENCY STOP - Signal lost!"

**Solution:**
1. RC transmitter turned on?
2. Receiver bound?
3. Correct wiring:
   - Mode 0: Pins 2-3 for drive, Pins 14-17 for sound
   - Mode 1: Pin 19 for iBus
4. For iBus: Serial1 baud rate = 115200

### Problem: Servos don't move

**Solution:**

**Mode 0 (PWM):**
- Servos should be connected DIRECTLY to RC receiver (CH3-6), NOT to Arduino!
- Arduino does not control servos in Mode 0
- Test servos by moving transmitter sticks/switches

**Mode 1 (iBus):**
1. Feature toggle "Servo Control" enabled?
2. Mode 1 selected?
3. Servos connected to Arduino pins 4, 5, 6, 9?
4. Separate power supply for servos connected?
5. Check iBus channel mapping

### Problem: Battery warning despite full battery

**Solution:**
1. Calibrate voltage divider (Menu → Option 3)
2. Check voltage divider wiring
3. Adjust Voltage Divider Factor until reading matches

### Problem: Motor only spins in one direction

**Solution:**
1. Check Direction pin wiring
2. Verify motor driver wiring
3. Test RC input (Serial Monitor)

### Problem: Old Mode 2 not working after update

**Solution:**
1. Mode 2 no longer exists
2. Enter configuration menu (send `m`)
3. Select option `1` (Setup Mode)
4. Choose `1` (iBus mode)
5. Save and restart

---

## 📊 Technical Details

### PID Control

Self-balancing uses a PID controller:

```
PID = (KP × Error) + (KI × ∫Error) + (KD × dError/dt)
```

- **P (Proportional):** Response to current deviation
- **I (Integral):** Correction of persistent deviations
- **D (Derivative):** Dampening of rapid changes

### Complementary Filter

IMU data combined with complementary filter:

```
Angle = 0.98 × (Angle + Gyro × dt) + 0.02 × Accel
```

- 98% Gyroscope (fast, drifts)
- 2% Accelerometer (slow, stable)

### Loop Time

- **Target:** <10ms per iteration (typically 5-8ms)
- **Critical:** No `delay()` in main loop!
- **PulseIn Timeout:** 10ms (Mode 0 only, optimized for speed)
- **Monitoring:** Real-time loop frequency available via Serial Monitor

---

## 📝 Comparison Table

| Feature | Mode 0 (PWM) | Mode 1 (iBus) |
|---------|--------------|---------------|
| **Receiver Type** | Standard PWM | iBus (FlySky) |
| **Wiring Complexity** | 6 PWM wires to Arduino | 1 iBus wire to Arduino |
| **Servo Control** | Direct to receiver (bypass Arduino) | Arduino-controlled (auto-correction) |
| **Servo Wiring** | Servos → Receiver CH3-6 | Servos → Arduino Pins 4,5,6,9 |
| **Interference** | Moderate | Very low |
| **Setup Difficulty** | Easy | Easy |
| **Recommended For** | Existing PWM setups | New installations |

---

## 👥 Credits

**Original Design:**
- D-O Droid Design: Star Wars / Lucasfilm
- 3D Model: [Printed-Droid.com](https://www.printed-droid.com)

**Software:**
- Base code: Reinhard Stockinger (2020)
- Optimization v3.x: PrintedDroid Community

**Libraries:**
- IBusBM: Bolder Flight Systems
- DFRobotDFPlayerMini: DFRobot
- MPU6050: Arduino Community

---

## 📄 License

This project is released under the **MIT License**.

```
MIT License

Copyright (c) 2025 PrintedDroid Community

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 🤝 Contributing

Contributions are welcome!

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 💬 Support

**Questions or issues?**

- GitHub Issues: [Create an issue](https://github.com/PrintedDroid/D-O-Printed-Droid/issues)
- Forum: [Printed-Droid.com Forum](https://www.printed-droid.com/forum)
- Discord: [PrintedDroid Community](https://discord.gg/printeddroid)

---

## ⚠️ Disclaimer

This code is provided "as is" without warranty. The authors assume no liability for damage to hardware, injuries, or other issues arising from the use of this code.

**Safety Notes:**
- Lithium batteries can be dangerous - follow all safety rules
- Test in a safe environment
- Children only under supervision
- Disconnect battery immediately if problems occur

---

<div align="center">

**May the Force be with your D-O!** ⚡🤖

[![Star on GitHub](https://img.shields.io/github/stars/PrintedDroid/D-O-Printed-Droid?style=social)](https://github.com/PrintedDroid/D-O-Printed-Droid)

*For the latest updates and community support, visit: [www.printed-droid.com](https://www.printed-droid.com)*

</div>
