# D-O Self-Balancing Droid - iBus Controller v2.1

![Version](https://img.shields.io/badge/version-2.1.6-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![License](https://img.shields.io/badge/license-Non--Commercial-red.svg)

**Advanced control system for self-balancing D-O droid replica from Star Wars**

---

## 🤖 Project Overview

This enhanced Arduino sketch provides complete control over a self-balancing D-O droid with three flexible setup modes, interactive configuration menu, EEPROM persistence, and extensive customization options.

Built on the proven v1.1 foundation with significant feature additions for builders who want full configurability without code modifications.

### Key Features

- **🎯 MPU6050 IMU Self-Balancing** - Advanced PID control with adaptive tuning
- **📡 Three Setup Modes** - PWM Only, Hybrid, or Pure iBus
- **⚙️ Interactive CLI Menu** - Configure all settings via Serial Monitor
- **💾 EEPROM Persistence** - All settings saved across reboots
- **🔋 Battery Monitoring** - Low voltage protection and warnings
- **🎮 RC Mixing Modes** - Arcade (recommended) or Tank style control
- **🎭 Idle Animations** - Random sounds and head movements when idle
- **📈 Adaptive PID** - Speed-dependent PID tuning for optimal control
- **🔊 DFPlayer Sound System** - 30+ sound effects with personality features
- **✅ IMU Clone Support** - Works with MPU6050/6500/9250/6886

---

## 📋 Table of Contents

- [Changelog](#-changelog)
- [Hardware Requirements](#-hardware-requirements)
- [Setup Modes](#-setup-modes)
- [Pin Configuration](#-pin-configuration)
- [iBus Channel Mapping](#-ibus-channel-mapping)
- [RC Mixing Modes](#-rc-mixing-modes)
- [Installation](#-installation)
- [Serial Commands (CLI)](#-serial-commands-cli)
- [Configuration Menu](#-configuration-menu)
- [SD Card File Structure](#-sd-card-file-structure)
- [Usage](#-usage)
- [Troubleshooting](#-troubleshooting)
- [Safety & Disclaimer](#-safety--disclaimer)
- [Community & Support](#-community--support)
- [License](#-license)
- [Credits](#-credits)

---

## 📝 Changelog

### Version 2.1.6 (December 2025)

**IMU Axis Invert + Test & Safety Features**

#### ✨ New Features

- **IMU Axis Invert** (`F` in IMU test menu): Software fix for reversed IMU orientation
  - No need to physically remount the IMU
  - Toggle front/back direction via CLI
  - Saved to EEPROM
- **IMU Axis Test** (`i` in CLI): Live angle display to verify IMU orientation
  - Shows Angle[0] (balance) and Angle[1] (side) in real-time
  - Displays raw accelerometer values (X, Y, Z)
  - Instructions to verify correct front/back tilt response
  - Helps diagnose X/Y axis swap or wrong IMU mounting
- **Motor Test Menu** (`m` in CLI): Test individual motors and configure wiring
  - Test Motor 1 (Left) / Motor 2 (Right) individually
  - Test both motors forward/backward
  - Toggle motor swap (Left↔Right)
  - Toggle motor direction inversion (per motor)
  - Helps diagnose wiring issues that cause PID oscillation
- **45° Tilt Safety Cutoff**: Motors automatically stop when droid falls over
  - Prevents motor burnout from stalled motors
  - Auto-resumes when droid is upright again

---

### Version 2.1.3 (December 2025)

**Safety Tilt Cutoff**

- Added 45° tilt angle safety cutoff to prevent motor damage

---

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

### Version 2.1.0 (December 2025)

**Configurable iBus Baudrate & Critical Bug Fixes**

#### 🐛 Critical Fixes

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

---

### Version 2.0.0 (June 2025)

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

## 🔧 Hardware Requirements

### Core Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Microcontroller** | Arduino Mega 2560 | Required for Serial1 (iBus) |
| **IMU** | MPU6050 or compatible | I2C address 0x68 or 0x69 |
| **Motor Driver** | Cytron MD10C (2x) | Or similar dual H-bridge |
| **Sound Module** | DFPlayer Mini | With Micro SD card (FAT32) |
| **RC Receiver** | iBus-compatible or PWM | FlySky FS-iA6B recommended |
| **Servos** | 4x Standard Servos | SG90 or similar |
| **Battery** | 2x 2S LiPo in series | 8.4V full, 6.0V empty |
| **Voltage Divider** | 10kΩ + 3.3kΩ resistors | For battery monitoring on A15 |
| **Speaker** | 8 Ohm, <3W | For DFPlayer audio output |

### Power Requirements

| Component | Voltage | Current |
|-----------|---------|---------|
| Arduino Mega | 7-12V | 200mA |
| Motors (2x) | Battery voltage | 1-2A each (stall) |
| Servos (4x) | 5-6V | 500mA total |
| DFPlayer | 3.3-5V | 200mA |

### Optional Components

- 1kΩ resistor for DFPlayer TX line (recommended)
- 100µF capacitor on battery input (noise reduction)

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

### Complete Pin Assignment

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                      ARDUINO MEGA 2560 PIN ASSIGNMENT                        │
├──────────────────────────────────────────────────────────────────────────────┤
│  Pin  │ Function           │ Direction │ Notes                              │
├───────┼────────────────────┼───────────┼────────────────────────────────────┤
│    0  │ Mainbar Servo      │ OUTPUT    │ PWM servo control                  │
│    1  │ Head Servo 1       │ OUTPUT    │ PWM servo control (Pitch)          │
│    3  │ PWM Drive 1        │ INPUT     │ Mode 0 only                        │
│    4  │ PWM Drive 2        │ INPUT     │ Mode 0 only                        │
│    5  │ Head Servo 2       │ OUTPUT    │ PWM servo control (Yaw)            │
│    6  │ Head Servo 3       │ OUTPUT    │ PWM servo control (Roll)           │
│    7  │ DFPlayer RX        │ OUTPUT    │ SoftwareSerial TX → DFPlayer       │
│    8  │ DFPlayer TX        │ INPUT     │ SoftwareSerial RX ← DFPlayer       │
│   10  │ Motor 2 PWM        │ OUTPUT    │ Motor speed                        │
│   11  │ Motor 2 DIR        │ OUTPUT    │ Motor direction                    │
│   12  │ Motor 1 PWM        │ OUTPUT    │ Motor speed                        │
│   13  │ Motor 1 DIR        │ OUTPUT    │ Motor direction                    │
│   14  │ Sound Switch 1     │ INPUT     │ Mode 0/1 - Mute (A0)               │
│   15  │ Sound Switch 2     │ INPUT     │ Mode 0/1 - Mode (A1)               │
│   16  │ Sound Switch 3     │ INPUT     │ Mode 0/1 - Mood (A2)               │
│   17  │ Sound Switch 4     │ INPUT     │ Mode 0/1 - Squeak (A3)             │
│   19  │ iBus RX            │ INPUT     │ Serial1 RX - Mode 1/2              │
│   20  │ I2C SDA            │ I/O       │ MPU6050 data                       │
│   21  │ I2C SCL            │ OUTPUT    │ MPU6050 clock                      │
│  A15  │ Battery Voltage    │ INPUT     │ Via voltage divider                │
└───────┴────────────────────┴───────────┴────────────────────────────────────┘
```

### Battery Voltage Divider

```
Battery (+) ──┬── 10kΩ ──┬── 3.3kΩ ──┬── GND
              │          │           │
              │          └── A15     │
              │                      │
Battery (-) ──┴──────────────────────┘

Voltage at A15 = Battery Voltage × (3.3 / 13.3)
```

---

## 🔌 iBus Channel Mapping

*For Modes 1 and 2:*

| Channel | Function | Description |
|---------|----------|-------------|
| **CH1** | Drive 1 | Left motor / Steering (depends on mixing mode) |
| **CH2** | Drive 2 | Right motor / Throttle (depends on mixing mode) |
| **CH3** | Mainbar | Mainbar servo position |
| **CH4** | Head 1 | Head pitch movement |
| **CH5** | Head 2 | Head yaw movement |
| **CH6** | Head 3 | Head roll movement |
| **CH7** | Sound Mute | Sound on/off (Mode 2 only) |
| **CH8** | Sound Mode | Greeting / Default (Mode 2 only) |
| **CH9** | Sound Mood | Negative / Neutral / Positive (Mode 2 only) |
| **CH10** | Sound Squeak | Squeak sounds (Mode 2 only) |

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
5. Select `s` → Save and Exit

---

## 🚀 Installation

### Step 1: Required Arduino Libraries

Install via Arduino Library Manager:

```
Required Libraries:
├── Wire                    (Arduino built-in)
├── IBusBM                  (by Bolder Flight Systems)
├── Servo                   (Arduino built-in)
├── EEPROM                  (Arduino built-in)
├── SoftwareSerial          (Arduino built-in)
└── DFRobotDFPlayerMini     (by DFRobot)
```

### Step 2: Board Configuration

1. Open Arduino IDE
2. Go to **Tools → Board → Arduino Mega or Mega 2560**
3. Select correct **Port** under Tools menu

### Step 3: Upload Sketch

1. Connect Arduino Mega via USB
2. Open `D_O_printed_droid_rc_ibus_v2.1.ino`
3. Click **Upload** (Ctrl+U)
4. Wait for "Done uploading" message

### Step 4: Initial Configuration

1. Open Serial Monitor (9600 baud)
2. Send `m` within 3 seconds to enter configuration menu
3. Configure your setup mode and preferences
4. Save and restart

---

## ⌨️ Serial Commands (CLI)

Connect via Serial Monitor at **9600 baud**.

### Quick Commands

| Command | Description |
|---------|-------------|
| `m` | Open configuration menu |
| `c` | Run IMU calibration |
| `s` | Show current status |

### Configuration Menu Access

Send `m` at any time to open the full configuration menu.

---

## ⚙️ Configuration Menu

### Menu Structure

```
=== CONFIGURATION MENU ===

1. PID Configuration
2. Feature Toggles
3. Battery Settings
4. Sound Settings
5. Driving Dynamics (includes Mixing Mode)
6. Adaptive PID Settings
7. IMU Calibration
8. Setup Type (PWM/Hybrid/iBus + Baudrate)
m. Motor Test & Config
i. IMU Axis Test (live angles)
9. Save and Exit
0. Exit without Saving
```

### Menu Options Quick Reference

| Option | What it configures |
|--------|-------------------|
| **1** | KP, KI, KD, Target Angle, Max Integral |
| **2** | Ramping, Adaptive PID, Dynamic Angle, Idle Actions, State Reactions, Battery Monitor |
| **3** | Warning/Critical Voltage, Voltage Divider Factor |
| **4** | Volume (0-30), Sound Intervals |
| **5** | Ramp Rate, Max Acceleration, Max Lean, Deadband, Expo, **Mixing Mode** |
| **6** | KP/KD values for Slow/Medium/Fast speeds |
| **7** | Run IMU calibration routine |
| **8** | Select Mode (0/1/2), Change iBus Baudrate |

### Changing iBus Baudrate

1. Select option `8` (Setup Type)
2. Press `b` to change baudrate
3. Select `1` for 9600 or `2` for 115200
4. Save and restart

---

## 💾 SD Card File Structure

Audio files must be placed on the Micro SD card in the `/mp3/` folder:

```
📁 /mp3/
├── 0001.mp3    Startup sound ("battery charged")
├── 0002.mp3    Default sound ("I am D-O")
├── 0003.mp3    Greeting 1
├── 0004.mp3    Greeting 2
├── 0005.mp3    Greeting 3
├── 0006.mp3    Negative 1
├── 0007.mp3    Negative 2
├── 0008.mp3    Negative 3
├── 0009.mp3    Negative 4
├── 0010.mp3    Positive 1
├── 0011.mp3    Positive 2
├── 0012.mp3    Positive 3
├── 0013.mp3    Positive 4
├── 0014.mp3    Positive 5
├── 0015.mp3    Squeak 1
├── 0016.mp3    Squeak 2
├── 0017.mp3    Squeak 3
├── 0018.mp3    Squeak 4
├── 0019.mp3    Squeak 5
├── 0020.mp3    Squeak 6
├── 0021.mp3    Tilt warning sound
├── 0022.mp3    Recovery/relief sound
├── 0023.mp3    Low battery warning
├── 0024.mp3    Idle sound 1
├── 0025.mp3    Idle sound 2
├── 0026.mp3    Idle sound 3
├── 0027.mp3    Idle sound 4
├── 0028.mp3    Idle sound 5
├── 0029.mp3    Idle sound 6
└── 0030.mp3    Idle sound 7
```

**Requirements:**
- SD card: FAT16 or FAT32 formatted
- Filenames: Exactly 4 digits (0001, 0002, etc.)
- Format: MP3, 128-320 kbps recommended

---

## 🎮 Usage

### First Startup

1. Connect battery
2. Turn on RC transmitter
3. Arduino starts - you'll see:
   ```
   === D-O Self-Balancing Controller v2.1.6 ===
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

```cpp
KP: 25.0
KI: 0.0
KD: 0.8
Target Angle: -0.3
```

---

## 🔍 Troubleshooting

### Problem: D-O does nothing / no response

**Solutions:**
1. Check Serial Monitor output
2. Verify you see `iBus initialized @ XXXX baud`
3. If not, the iBus init failed - check wiring
4. Try changing baudrate (9600 ↔ 115200)

### Problem: "EMERGENCY STOP - Signal lost!"

**Solutions:**
1. RC transmitter turned on?
2. Receiver bound to transmitter?
3. Correct wiring for your mode?
4. Try different iBus baudrate

### Problem: Forward/backward much slower than turning

**Solutions:**
1. Change to Arcade mixing mode (Menu → 5 → select 1)
2. Or configure tank mixing on your transmitter

### Problem: Battery shows wrong voltage (1.8V, 2.8V)

**Solutions:**
1. Voltage divider not connected - enable via Menu → 2 → Battery Monitor OFF
2. Or connect proper voltage divider to A15

### Problem: iBus works with v1.1 but not v2.1

**Solutions:**
1. v2.1 defaults to 9600 baud (like v1.1)
2. If still not working, check Serial output for init messages
3. Make sure you have the latest v2.1 with the bug fixes

### Problem: D-O tips over / doesn't balance

**Solutions:**
1. Perform IMU calibration (send `c`)
2. Adjust Target Angle (usually -1.0 to 1.0)
3. Check PID values
4. Verify MPU6050 wiring

### Problem: No sound

**Solutions:**
1. SD card formatted FAT16/FAT32?
2. Files in `/mp3/` folder with correct names?
3. Check DFPlayer wiring (pins 7, 8)
4. Increase volume in menu (option 4)

---

## ⚠️ Safety & Disclaimer

### Safety Warnings

1. **Lithium Batteries are dangerous!**
   - Never leave charging batteries unattended
   - Use proper LiPo charger with balance charging
   - Store in fireproof LiPo bag
   - Dispose of damaged batteries properly

2. **Moving Parts**
   - Keep fingers away from wheels during operation
   - Secure head before testing balance
   - Test in open area away from obstacles

3. **Electronics**
   - Do not connect/disconnect while powered
   - Check polarity before connecting batteries
   - Use appropriate fuses for motor circuits
   - Ensure adequate ventilation for motor drivers

### Disclaimer

```
THIS SOFTWARE AND HARDWARE DESIGN IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
KIND, EXPRESS OR IMPLIED. THE AUTHORS AND CONTRIBUTORS ARE NOT LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
ARISING FROM THE USE OF THIS PROJECT.

USE AT YOUR OWN RISK. You are responsible for ensuring safe operation of your
build. Always supervise operation and be prepared to cut power immediately if
something goes wrong.

This is a fan project for personal, non-commercial use only. D-O, Star Wars,
and all related properties are trademarks of Lucasfilm Ltd. and The Walt Disney
Company.
```

---

## 🌐 Community & Support

### Official Resources

- **Website**: [www.printed-droid.com](https://www.printed-droid.com)
- **Facebook Group**: [Printed Droid Community](https://www.facebook.com/groups/printeddroid/)

### Getting Help

1. Check the [Troubleshooting](#-troubleshooting) section first
2. Search the Facebook group for similar issues
3. Post in the Facebook group with:
   - Your hardware setup (photos help!)
   - Serial Monitor output
   - Arduino IDE version
   - What you've already tried

### Contributing

Found a bug or have an improvement?
- Report issues on GitHub
- Share your builds in the Facebook group!

---

## 📄 License

```
NON-COMMERCIAL LICENSE

Copyright (c) 2020-2025 Reinhard Stockinger & Printed-Droid.com Community

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to use,
copy, modify, and distribute the Software for PERSONAL, NON-COMMERCIAL purposes
only, subject to the following conditions:

1. The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

2. The Software shall NOT be used for commercial purposes, including but not
   limited to: selling, licensing, or including in commercial products or
   services.

3. Attribution must be given to the original authors and the Printed-Droid.com
   community in any derivative works.

4. This is a fan project. Star Wars, D-O, and all related properties are
   trademarks of Lucasfilm Ltd. and The Walt Disney Company.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

---

## 👥 Credits

**Original Design:**
- D-O Droid Design: Star Wars / Lucasfilm / The Walt Disney Company

**Software:**
- Original code: Reinhard Stockinger (2020)
- v2.0+ enhancements: Printed-Droid.com Community
- v2.1 bug fixes & features: Printed-Droid.com

**Libraries:**
- IBusBM: Bolder Flight Systems
- DFRobotDFPlayerMini: DFRobot

**Community:**
- Testing and feedback: Printed-Droid.com Community

---

<div align="center">

**May the Force be with your D-O!** ⚡🤖

*For the latest updates and community support:*

🌐 [www.printed-droid.com](https://www.printed-droid.com) | 📘 [Facebook Group](https://www.facebook.com/groups/printeddroid/)

</div>
