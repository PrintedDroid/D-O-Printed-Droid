# D-O Self-Balancing Droid - Universal Controller v3

![Version](https://img.shields.io/badge/version-3.3.3-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![License](https://img.shields.io/badge/license-Non--Commercial-red.svg)

**Universal control system for self-balancing D-O droid replica from Star Wars**

---

## 🤖 Project Overview

This is a complete rewrite of the D-O control system with optimized architecture, simplified setup modes, and advanced features. It combines the best of v1.1 and v2.1 with significant improvements in reliability, performance, and maintainability.

Designed for builders who want the most advanced and reliable D-O control system.

### Key Features

- **🎯 Advanced IMU Self-Balancing** - Precise PID control with micros() timing
- **📡 Two Simple Setup Modes** - PWM Only or Pure iBus (no hybrid confusion)
- **⚙️ Interactive CLI Menu** - Configure all settings via Serial Monitor
- **💾 EEPROM Persistence** - All settings saved across reboots
- **🛡️ Watchdog Timer** - Automatic crash recovery (2s timeout)
- **🔋 Battery Monitoring** - Low voltage protection with hysteresis
- **🎮 RC Mixing Modes** - Arcade (recommended) or Tank style control
- **🎭 Idle Animations** - Random sounds and head movements when idle
- **📈 Adaptive PID** - Speed-dependent PID tuning for optimal control
- **🔄 Dynamic Lean Angle** - Natural forward lean when driving
- **🔊 DFPlayer Sound System** - 30+ sound effects with state reactions
- **✅ IMU Clone Support** - Works with MPU6050/6500/9250/6886
- **⚡ I2C Fast Mode** - 400kHz for faster IMU readings
- **🔌 Pin-Compatible** - Works with v1.1/v2.1 wiring

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

### Version 3.3.3 (December 2025)

**Motor Test & Configuration Menu + Safety Features**

#### ✨ New Features

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

### Version 3.3.2 (December 2025)

**Watchdog Fix & Safety Tilt Cutoff**

- **Fixed watchdog reset loop**: Watchdog now enables AFTER menu wait (was causing continuous resets)
- Added 45° tilt angle safety cutoff to prevent motor damage

---

### Version 3.3.1 (December 2025)

**RC Mixing Mode Selection**

#### ✨ New Features

- **Arcade Mixing Mode** (now default): One stick controls both throttle and steering
- **Tank Mixing Mode**: Original behavior - each stick controls one motor
- **CLI Configuration**: Select mixing mode via Serial Monitor menu (Option 4 → Driving Dynamics)

This fixes the issue where forward/backward driving was much slower than turning with standard FlySky configuration.

---

### Version 3.3.0 (December 2025)

**Added Idle Animations, Adaptive PID, Dynamic Lean**

This release brings feature parity with v2.1 while maintaining v3's advanced architecture.

#### ✨ New Features

**1. Idle Action System**
- Random sounds and head movements when idle (no RC activity for 3+ seconds)
- 70% chance of idle sounds (tracks 24-30)
- 30% chance of servo animations (nod, look around, head shake)
- Configurable intervals via CLI menu

**2. Adaptive PID**
- Speed-dependent PID tuning for optimal control
- **Slow** (<50): Higher KP/KD for stability
- **Medium** (50-150): Balanced values
- **Fast** (>150): Lower KP/KD for smoothness
- All values configurable via CLI

**3. Dynamic Lean Angle**
- Automatic forward lean based on drive input
- Creates natural movement feel
- Configurable max lean angle (default: 3°)

**4. State-Based Reactions**
- Tilt warning sound when angle exceeds 15°
- Recovery sound when stabilized
- Prevents audio spam with 5-second cooldown

**5. Configurable iBus Baudrate**
- 9600 baud (non-standard, for compatibility)
- 115200 baud (standard iBus)
- Selectable via CLI menu

#### 🔊 New Sound Tracks

| Track | Function |
|-------|----------|
| 0021.mp3 | Tilt warning |
| 0022.mp3 | Recovery/relief |
| 0024-0030.mp3 | Idle sounds |
| 0031.mp3 | System ready |
| 0032.mp3 | Signal lost |

---

### Version 3.2.0 (November 2025)

**Complete Architecture Rewrite**

- Simplified to 2 modes: PWM Only or iBus
- Watchdog timer for crash recovery
- I2C Fast Mode (400kHz)
- micros() timing for precise PID
- Improved IMU error handling with auto-recovery
- IMU clone support (MPU6500/9250/6886)
- Pin-compatible with v1.1 and v2.1

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

### Mode 0: PWM Only (Classic)

**Best for:** Basic setups with standard PWM receivers

- PWM input on pins 3, 4 for drive (tank-mixed)
- PWM input on pins 14-17 (A0-A3) for sound switches
- DFPlayer ENABLED on Mega (no external Nano needed!)
- Compatible with original D-O v2 Mega sketch

### Mode 1: iBus (Recommended)

**Best for:** Modern setups with FlySky receivers

- All 10 channels via iBus protocol
- Cleanest wiring - only 1 signal wire!
- All features available
- DFPlayer ENABLED on Mega
- Configurable baudrate (9600 or 115200)

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
│   14  │ Sound Switch 1     │ INPUT     │ Mode 0 - Mute (A0)                 │
│   15  │ Sound Switch 2     │ INPUT     │ Mode 0 - Mode (A1)                 │
│   16  │ Sound Switch 3     │ INPUT     │ Mode 0 - Mood (A2)                 │
│   17  │ Sound Switch 4     │ INPUT     │ Mode 0 - Squeak (A3)               │
│   19  │ iBus RX            │ INPUT     │ Serial1 RX - Mode 1                │
│   20  │ I2C SDA            │ I/O       │ MPU6050 data (Fast Mode 400kHz)    │
│   21  │ I2C SCL            │ OUTPUT    │ MPU6050 clock (Fast Mode 400kHz)   │
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

*Relevant for Mode 1 only:*

| Channel | Function | Description |
|---------|----------|-------------|
| **CH1** | Drive 1 | Left motor / Steering (depends on mixing mode) |
| **CH2** | Drive 2 | Right motor / Throttle (depends on mixing mode) |
| **CH3** | Mainbar | Mainbar servo position |
| **CH4** | Head 1 | Head pitch movement |
| **CH5** | Head 2 | Head yaw movement |
| **CH6** | Head 3 | Head roll movement |
| **CH7** | Sound Mute | Sound on/off |
| **CH8** | Sound Mode | Greeting / Default |
| **CH9** | Sound Mood | Negative / Neutral / Positive (3-pos) |
| **CH10** | Sound Squeak | Squeak sounds |

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
3. Select `4` → Driving Dynamics
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
2. Open `D_O_printed_droid_rc_ibus_v3.ino`
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

### Configuration Menu Access

Send `m` at any time to open the full configuration menu.

---

## ⚙️ Configuration Menu

### Menu Structure

```
=== CONFIGURATION MENU ===

1. Setup Mode (PWM/iBus)
2. PID Configuration
3. Adaptive PID Settings
4. Driving Dynamics (includes Mixing Mode)
5. Battery Settings
6. Sound Settings
7. Feature Toggles
8. IMU Calibration
9. Show Current Status
m. Motor Test & Config
s. Save and Exit
0. Exit without Saving
```

### Menu Options Quick Reference

| Option | What it configures |
|--------|-------------------|
| **1** | Setup Mode (0=PWM, 1=iBus), iBus Baudrate |
| **2** | KP, KI, KD, Target Angle, Max Integral |
| **3** | KP/KD values for Slow/Medium/Fast speeds |
| **4** | Ramp Rate, Max Acceleration, Max Lean, Deadband, Expo, **Mixing Mode** |
| **5** | Warning/Critical Voltage, Voltage Divider, Recovery Mode |
| **6** | Volume (0-30), Sound Intervals |
| **7** | Mainbar Correction, Ramping, Adaptive PID, Dynamic Angle, Idle Actions, State Reactions, Battery Monitor, Servos, Watchdog |
| **8** | Run IMU calibration routine |
| **9** | Display current system status |

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
├── 0023.mp3    Battery warning
├── 0024.mp3    Idle sound 1
├── 0025.mp3    Idle sound 2
├── 0026.mp3    Idle sound 3
├── 0027.mp3    Idle sound 4
├── 0028.mp3    Idle sound 5
├── 0029.mp3    Idle sound 6
├── 0030.mp3    Idle sound 7
├── 0031.mp3    System ready
└── 0032.mp3    Signal lost
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
   === D-O Universal Controller v3.3.3 ===
   Configuration loaded
   Setup Mode: iBus (Recommended)
   IMU found at 0x68
   WHO_AM_I: 0x68 (MPU6050)
   iBus @ 115200 baud
   Waiting for RC signal...
   System ready!
   ```
4. Wait for "System ready!" sound
5. D-O self-balances automatically

### IMU Calibration (Important!)

**Must be performed before first use:**

1. Send `c` at startup (or option 8 in menu)
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

### Watchdog Timer

v3 includes a watchdog timer that automatically restarts the Arduino if the code hangs for more than 2 seconds. This can be disabled via Menu → 7 → Watchdog Timer.

---

## 🔍 Troubleshooting

### Problem: D-O does nothing / no response

**Solutions:**
1. Check Serial Monitor output
2. Verify you see `iBus @ XXXX baud`
3. If not, the iBus init failed - check wiring
4. Try changing baudrate (Menu → 1 → b)

### Problem: Signal lost but keeps balancing

**This is intentional!** v3 continues balancing when signal is lost, just without drive input. The motors only stop for battery critical or IMU failure.

### Problem: Forward/backward much slower than turning

**Solutions:**
1. Change to Arcade mixing mode (Menu → 4 → select 1)
2. Or configure tank mixing on your transmitter

### Problem: Battery shows wrong voltage

**Solutions:**
1. Battery monitoring is OFF by default in v3.3.1
2. Enable via Menu → 7 → Battery Monitor → ON
3. Connect voltage divider to A15 first!

### Problem: System keeps restarting

**Solutions:**
1. Watchdog is triggering - code is hanging somewhere
2. Check IMU wiring (I2C errors cause restarts)
3. Temporarily disable watchdog via Menu → 7

### Problem: D-O tips over / doesn't balance

**Solutions:**
1. Perform IMU calibration (send `c`)
2. Adjust Target Angle (usually -1.0 to 1.0)
3. Check PID values
4. Verify MPU6050 wiring (SDA=20, SCL=21)

### Problem: No sound

**Solutions:**
1. SD card formatted FAT16/FAT32?
2. Files in `/mp3/` folder with correct names?
3. Check DFPlayer wiring (pins 7, 8)
4. Increase volume in menu (option 6)

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
- v3 architecture: Printed-Droid.com Community (2025)

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
