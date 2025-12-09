# D-O Self-Balancing Droid - iBus Controller v2.1

![Version](https://img.shields.io/badge/version-2.1.2-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![License](https://img.shields.io/badge/license-MIT-orange.svg)

**Complete control system for self-balancing D-O droid with iBus RC support and advanced features**

This Arduino sketch provides full control over a self-balancing D-O droid (from Star Wars) with three setup modes, integrated sound system, and extensive configuration options.

## 📋 Table of Contents

- [Changelog](#-changelog)
- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Setup Modes](#-setup-modes)
- [Pin Configuration](#-pin-configuration)
- [Installation](#-installation)
- [Configuration Menu](#-configuration-menu)
- [Usage](#-usage)
- [Troubleshooting](#-troubleshooting)
- [Credits](#-credits)

---

## 📝 Changelog

### Version 2.1.2 (December 2025)

**RC Mixing Mode Selection**

#### ✨ New Features

- **Arcade Mixing Mode** (now default): One stick controls both throttle and steering
- **Tank Mixing Mode**: Original behavior - each stick controls one motor
- **CLI Configuration**: Select mixing mode via Serial Monitor menu (Option 5 → Driving Dynamics)

This fixes the issue where forward/backward driving was much slower than turning with standard FlySky configuration.

---

### Version 2.1.1 (December 2025)

**Robust IMU Clone Support**

#### 🔧 IMU Improvements

- **Dynamic I2C Address Detection**: Automatically finds IMU at 0x68 or 0x69
- **WHO_AM_I Register Check**: Identifies chip type for better compatibility
- **Clone Support**: Works with MPU6050, MPU6500, MPU9250, MPU6886
- **Improved Error Handling**: Validates I2C byte reads, skips bad data
- **Better Diagnostics**: Reports detected IMU type at startup

#### ✅ Supported IMU Chips

| WHO_AM_I | Chip Type | Status |
|----------|-----------|--------|
| 0x68 | MPU6050 | ✅ Original |
| 0x70 | MPU6500 | ✅ Clone |
| 0x71 | MPU9250 | ✅ Clone |
| 0x19 | MPU6886 | ✅ Clone |

---

### Version 2.1 (December 2025)

**Configurable iBus Baudrate & Critical Bug Fixes**

#### 🔧 Critical Fixes

**1. Missing `#define IBUS_ENABLED`**
- **Problem**: iBus was never initialized because the define was missing
- **Solution**: Added `#define IBUS_ENABLED` to feature flags
- **Impact**: iBus now works correctly - this was the main reason v2 "did nothing"

**2. Wrong Initialization Condition**
- **Problem**: `#ifdef IBUS_ENABLED` was used but never defined
- **Solution**: Changed to `#ifdef IBUS_AVAILABLE` which is set by `SETUP_TYPE_RUNTIME`

#### ✨ New Features

**Configurable iBus Baudrate:**
- Default: 9600 baud (matches v1.1 which works)
- Optional: 115200 baud (standard iBus)
- Configurable via Serial Monitor menu
- Saved to EEPROM

**How to change baudrate:**
1. Open Serial Monitor (9600 baud)
2. Send `m` → Configuration Menu
3. Select `8` → Setup Type
4. Press `b` → Change iBus baudrate
5. Select `1` (9600) or `2` (115200)
6. Select `9` → Save and Exit
7. Restart Arduino

---

### Version 2.0 (June 2025)

**Full Feature Set with Configuration Menu**

- Serial configuration menu with EEPROM storage
- Battery voltage monitoring (2x 2S LiPo in series)
- Motor ramping for smooth acceleration
- Idle action system with random animations
- State-based sound reactions (tilt warnings, etc.)
- Adaptive PID based on speed
- Dynamic target angle for natural leaning
- Deadband and exponential RC control
- Acceleration limiting
- Turn rate compensation

---

## 🎯 Features

### Core Capabilities
- ✅ **MPU6050 IMU Self-Balancing** with advanced PID control
- ✅ **Robust IMU Clone Support** (MPU6050/6500/9250/6886)
- ✅ **Three Setup Modes**: PWM Only, Hybrid, or Pure iBus
- ✅ **Configurable iBus Baudrate** (9600 or 115200)
- ✅ **Integrated DFPlayer Sound System** with personality features
- ✅ **Multi-Servo Control** for head and mainbar movement
- ✅ **Battery Monitoring** with low voltage protection
- ✅ **Interactive Configuration Menu** via Serial Monitor
- ✅ **EEPROM Persistence** - all settings saved across reboots
- ✅ **IMU Calibration** for precise balance control

### Advanced Features
- Motor ramping for smooth acceleration
- Adaptive PID (adjusts based on speed)
- Dynamic target angle (leans into movement)
- Idle action system (random animations when idle)
- State-based reactions (tilt warnings, recovery sounds)
- Deadband and exponential RC input processing
- Mainbar auto-correction based on tilt angle
- Turn rate compensation
- Emergency stop on signal loss

---

## 🔧 Hardware Requirements

### Core Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Microcontroller** | Arduino Mega 2560 | Required for Serial1 |
| **IMU** | MPU6050 | I2C address 0x68 |
| **Motor Driver** | Cytron MD10C or similar | 2x for differential drive |
| **Sound Module** | DFPlayer Mini | With Micro SD card |
| **RC Receiver** | iBus-compatible OR PWM | FlySky recommended |
| **Servos** | 4x Standard Servos | For head and mainbar |
| **Battery** | 2x 2S LiPo in series | 8.4V full, 6.0V empty |
| **Voltage Divider** | 10kΩ + 3.3kΩ | For battery monitoring on A15 |
| **Speaker** | 8 Ohm, <3W | For DFPlayer |

---

## 🔀 Setup Modes

### Mode 0: PWM Only (Original)
**Best for:** Balance only, external Nano handles sound

- PWM input on pins 3, 4 for drive (tank-mixed)
- No sound control from Mega
- Compatible with original D-O v2 Mega sketch

### Mode 1: Hybrid
**Best for:** iBus drive with PWM sound switches

- iBus for drive channels (CH1-6)
- PWM on pins 14-17 (A0-A3) for sound switches
- Good for mixed setups

### Mode 2: Pure iBus (Default, Recommended)
**Best for:** New installations with iBus receivers

- All 10 channels via iBus protocol
- Cleanest wiring - only 1 signal wire!
- All features available
- Requires iBus-compatible receiver (e.g., FlySky FS-iA6B)

---

## 📍 Pin Configuration

### Motors (All Modes)
```
Pin 13 → Motor 1 Direction (DIR1)
Pin 12 → Motor 1 Speed (PWM1)
Pin 11 → Motor 2 Direction (DIR2)
Pin 10 → Motor 2 Speed (PWM2)
```

### Servos (All Modes)
```
Pin 0 → Mainbar Servo
Pin 1 → Head Servo 1
Pin 5 → Head Servo 2
Pin 6 → Head Servo 3
```

### DFPlayer Mini (All Modes)
```
Pin 7 → DFPlayer RX
Pin 8 → DFPlayer TX
```

### Mode 0: PWM Only
```
Pin 3 → Drive Channel 1 (PWM)
Pin 4 → Drive Channel 2 (PWM)
```

### Mode 1: Hybrid
```
Pin 19 (Serial1 RX) → iBus Signal (CH1-6)
Pin 14 (A0) → Sound Mute (PWM)
Pin 15 (A1) → Sound Mode (PWM)
Pin 16 (A2) → Sound Mood (PWM, 3-position)
Pin 17 (A3) → Sound Squeak (PWM)
```

### Mode 2: Pure iBus
```
Pin 19 (Serial1 RX) → iBus Signal (all 10 channels)
```

### I2C (MPU6050)
```
Pin 20 → SDA
Pin 21 → SCL
```

### Battery Monitor
```
Pin A15 → Voltage divider input
```

---

## 🔌 iBus Channel Mapping

*For Modes 1 and 2:*

| Channel | Function | Description |
|---------|----------|-------------|
| CH1 | Drive 1 | Left motor (tank-mixed) |
| CH2 | Drive 2 | Right motor (tank-mixed) |
| CH3 | Mainbar | Mainbar servo position |
| CH4 | Head 1 | Head pitch movement |
| CH5 | Head 2 | Head yaw movement |
| CH6 | Head 3 | Head roll movement |
| CH7 | Sound Mute | Sound on/off (Mode 2 only) |
| CH8 | Sound Mode | Greeting / Default (Mode 2 only) |
| CH9 | Sound Mood | Negative / Neutral / Positive (Mode 2 only) |
| CH10 | Sound Squeak | Squeak sounds (Mode 2 only) |

---

## 🎮 RC Mixing Modes

The sketch supports two RC mixing modes, configurable via the Serial menu:

### Arcade Mode (Default, Recommended)

**One stick controls everything:**
- CH2 (Throttle) = Forward/Backward
- CH1 (Steering) = Turn Left/Right

```
        Forward
           ↑
   Left ←  ●  → Right
           ↓
        Backward
```

**Advantages:**
- Works with standard FlySky configuration (no transmitter mixing needed)
- More intuitive - same as RC cars
- One-handed control possible

### Tank Mode (Original)

**Two sticks, each controls one motor:**
- CH1 = Left Motor
- CH2 = Right Motor

```
Left Stick    Right Stick
    ↑             ↑
    ●             ●
    ↓             ↓
  Motor1        Motor2
```

**Advantages:**
- More precise control at low speeds
- Better for tight maneuvering

### Changing Mixing Mode

1. Open Serial Monitor (9600 baud)
2. Send `m` → Configuration Menu
3. Select `5` → Driving Dynamics
4. At the end, select mixing mode:
   - `0` = Tank Mode
   - `1` = Arcade Mode (recommended)
5. Select `9` → Save and Exit

---

## 💾 SD Card File Structure

Audio files must be placed on the Micro SD card in the `/mp3/` folder:

```
/mp3/
  0001.mp3 - Startup sound ("battery charged")
  0002.mp3 - Default sound ("I am D-O")
  0003.mp3 to 0005.mp3 - Greeting sounds
  0006.mp3 to 0009.mp3 - Negative sounds
  0010.mp3 to 0014.mp3 - Positive sounds
  0015.mp3 to 0020.mp3 - Squeaky wheel sounds
  0021.mp3 - Tilt warning sound
  0022.mp3 - Recovery/relief sound
  0023.mp3 - Low battery warning
  0024.mp3 to 0030.mp3 - Idle sounds
```

---

## 🚀 Installation

### 1. Required Arduino Libraries

Install via Arduino Library Manager:

- **Wire** (Arduino built-in)
- **IBusBM** (by Bolder Flight Systems)
- **Servo** (Arduino built-in)
- **EEPROM** (Arduino built-in)
- **SoftwareSerial** (Arduino built-in)
- **DFRobotDFPlayerMini** (by DFRobot)

### 2. Upload Sketch

1. Open `D_O_printed_droid_rc_ibus_v2.1.ino` in Arduino IDE
2. Select board: **Tools → Board → Arduino Mega 2560**
3. Select port: **Tools → Port → [Your COM Port]**
4. Upload: **Sketch → Upload**

---

## ⚙️ Configuration Menu

### Access Menu

1. Open Serial Monitor (9600 baud)
2. Send `m` within 3 seconds of startup, OR
3. Send `m` anytime during operation

### Menu Options

```
=== CONFIGURATION MENU ===

1. PID Configuration
2. Feature Toggles
3. Battery Settings
4. Sound Settings
5. Driving Dynamics
6. Adaptive PID Settings
7. IMU Calibration
8. Setup Type (PWM/Hybrid/iBus)
9. Save and Exit
0. Exit without Saving
```

### Quick Reference

| Option | What it configures |
|--------|-------------------|
| 1 | KP, KI, KD, Target Angle, Max Integral |
| 2 | Ramping, Adaptive PID, Dynamic Angle, Idle Actions, State Reactions, Battery Monitor |
| 3 | Warning/Critical Voltage, Voltage Divider Factor |
| 4 | Volume (0-30), Sound Intervals |
| 5 | Ramp Rate, Max Acceleration, Max Lean, Deadband, Expo |
| 6 | KP/KD values for Slow/Medium/Fast speeds |
| 7 | Run IMU calibration routine |
| 8 | Select Mode (0/1/2), **Change iBus Baudrate** |

### Changing iBus Baudrate

1. Select option `8` (Setup Type)
2. Press `b` to change baudrate
3. Select `1` for 9600 or `2` for 115200
4. Save and restart

---

## 🎮 Usage

### First Startup

1. Connect battery
2. Turn on RC transmitter
3. Arduino starts - you'll see:
   ```
   === D-O Self-Balancing Controller v2.1 ===
   Configuration loaded from EEPROM
   iBus initialized @ 9600 baud
   Waiting for RC signal...
   ```
4. Wait for RC signal detection
5. D-O self-balances automatically

### IMU Calibration (Important!)

**Must be performed before first use:**

1. Send `c` at startup (or option 7 in menu)
2. Place D-O in balanced position
3. Keep absolutely still for 3 seconds
4. Wait for calibration to complete (~3 seconds)
5. Values are automatically saved to EEPROM

### Default PID Values

```
KP: 25.0
KI: 0.0
KD: 0.8
Target Angle: -0.3
```

---

## 🔍 Troubleshooting

### Problem: D-O does nothing / no response

**Solution:**
1. Check Serial Monitor output
2. Verify you see `iBus initialized @ XXXX baud`
3. If not, the iBus init failed - check wiring
4. Try changing baudrate (9600 ↔ 115200)

### Problem: "EMERGENCY STOP - Signal lost!"

**Solution:**
1. RC transmitter turned on?
2. Receiver bound to transmitter?
3. Correct wiring for your mode?
4. Try different iBus baudrate

### Problem: iBus works with v1.1 but not v2.1

**Solution:**
1. v2.1 defaults to 9600 baud (like v1.1)
2. If still not working, check Serial output for init messages
3. Make sure you have the latest v2.1 with the bug fixes

### Problem: D-O tips over / doesn't balance

**Solution:**
1. Perform IMU calibration (send `c`)
2. Adjust Target Angle (usually -1.0 to 1.0)
3. Check PID values
4. Verify MPU6050 wiring

### Problem: No sound

**Solution:**
1. SD card formatted FAT16/FAT32?
2. Files in `/mp3/` folder with correct names?
3. Check DFPlayer wiring (pins 7, 8)
4. Increase volume in menu (option 4)

---

## 📊 Technical Details

### PID Control

```
PID = (KP × Error) + (KI × ∫Error) + (KD × dError/dt)
```

### Complementary Filter

```
Angle = 0.98 × (Angle + Gyro × dt) + 0.02 × Accel
```

### Adaptive PID

Speed-dependent PID adjustment:
- **Slow** (<50): Higher KP/KD for stability
- **Medium** (50-150): Balanced values
- **Fast** (>150): Lower KP/KD for smoothness

---

## 👥 Credits

**Original Design:**
- D-O Droid Design: Star Wars / Lucasfilm

**Software:**
- Original code: Reinhard Stockinger (2020)
- v2.0 enhancements: Printed-Droid.com
- v2.1 bug fixes: Printed-Droid.com

**Libraries:**
- IBusBM: Bolder Flight Systems
- DFRobotDFPlayerMini: DFRobot

---

## 💬 Support

- GitHub Issues: [Create an issue](https://github.com/PrintedDroid/D-O-Printed-Droid/issues)

---

<div align="center">

**May the Force be with your D-O!** ⚡🤖

*For the latest updates, visit: [www.printed-droid.com](https://www.printed-droid.com)*

</div>

