# D-O Self-Balancing Droid - iBus Controller v1.1

![Version](https://img.shields.io/badge/version-1.1-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![License](https://img.shields.io/badge/license-Non--Commercial-red.svg)

**Original iBus control system for self-balancing D-O droid replica from Star Wars**

---

## 🤖 Project Overview

This is the original stable Arduino sketch for the D-O Printed Droid. It provides reliable self-balancing control with iBus RC support, DFPlayer sound system, and multi-servo control for head and mainbar movement.

Designed for builders who want a proven, stable foundation for their D-O build.

### Key Features

- **🎯 MPU6050 IMU Self-Balancing** - PID-controlled balance with complementary filter
- **📡 iBus RC Support** - 10 channels via FlySky iBus protocol
- **🔊 DFPlayer Sound System** - 20+ sound effects with mood categories
- **🎭 4-Servo Control** - Head pitch/yaw/roll + mainbar movement
- **⚖️ Mainbar Tilt Correction** - Optional automatic mainbar angle compensation
- **✅ Proven Stable** - Tested and working in production builds

---

## 📋 Table of Contents

- [Hardware Requirements](#-hardware-requirements)
- [Pin Configuration](#-pin-configuration)
- [iBus Channel Mapping](#-ibus-channel-mapping)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [SD Card File Structure](#-sd-card-file-structure)
- [Usage](#-usage)
- [Troubleshooting](#-troubleshooting)
- [Upgrading](#-upgrading)
- [Safety & Disclaimer](#-safety--disclaimer)
- [Community & Support](#-community--support)
- [License](#-license)
- [Credits](#-credits)

---

## 🔧 Hardware Requirements

### Core Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Microcontroller** | Arduino Mega 2560 | Required for Serial1 (iBus) |
| **IMU** | MPU6050 | I2C address 0x68 |
| **Motor Driver** | Cytron MD10C (2x) | Or similar dual H-bridge |
| **Sound Module** | DFPlayer Mini | With Micro SD card (FAT32) |
| **RC Receiver** | iBus-compatible | FlySky FS-iA6B recommended |
| **Servos** | 4x Standard Servos | SG90 or similar |
| **Speaker** | 8 Ohm, <3W | For DFPlayer audio output |

### Power Requirements

| Component | Voltage | Current |
|-----------|---------|---------|
| Arduino Mega | 7-12V | 200mA |
| Motors (2x) | Battery voltage | 1-2A each (stall) |
| Servos (4x) | 5-6V | 500mA total |
| DFPlayer | 3.3-5V | 200mA |

**Recommended:** 2S-3S LiPo battery (7.4V - 11.1V) with BEC for servo power.

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
│    3  │ PWM Drive 1        │ INPUT     │ Fallback if iBus disabled          │
│    4  │ PWM Drive 2        │ INPUT     │ Fallback if iBus disabled          │
│    5  │ Head Servo 2       │ OUTPUT    │ PWM servo control (Yaw)            │
│    6  │ Head Servo 3       │ OUTPUT    │ PWM servo control (Roll)           │
│    7  │ DFPlayer RX        │ OUTPUT    │ Software Serial TX → DFPlayer RX   │
│    8  │ DFPlayer TX        │ INPUT     │ Software Serial RX ← DFPlayer TX   │
│   10  │ Motor 2 PWM        │ OUTPUT    │ Motor speed (LEDC)                 │
│   11  │ Motor 2 DIR        │ OUTPUT    │ Motor direction                    │
│   12  │ Motor 1 PWM        │ OUTPUT    │ Motor speed (LEDC)                 │
│   13  │ Motor 1 DIR        │ OUTPUT    │ Motor direction                    │
│   14  │ Sound Switch 1     │ INPUT     │ PWM fallback (A0)                  │
│   15  │ Sound Switch 2     │ INPUT     │ PWM fallback (A1)                  │
│   16  │ Sound Switch 3     │ INPUT     │ PWM fallback (A2)                  │
│   17  │ Sound Switch 4     │ INPUT     │ PWM fallback (A3)                  │
│   19  │ iBus RX            │ INPUT     │ Serial1 RX - iBus signal           │
│   20  │ I2C SDA            │ I/O       │ MPU6050 data                       │
│   21  │ I2C SCL            │ OUTPUT    │ MPU6050 clock                      │
└───────┴────────────────────┴───────────┴────────────────────────────────────┘
```

### Motor Controller Wiring (Cytron MD10C)

```
┌─────────────────────────────────────────────────────────────┐
│  Arduino Mega          │  Cytron MD10C #1  │  Cytron MD10C #2 │
├────────────────────────┼───────────────────┼──────────────────┤
│  Pin 13 (DIR1)         │  DIR              │                  │
│  Pin 12 (PWM1)         │  PWM              │                  │
│  Pin 11 (DIR2)         │                   │  DIR             │
│  Pin 10 (PWM2)         │                   │  PWM             │
│  GND                   │  GND              │  GND             │
└────────────────────────┴───────────────────┴──────────────────┘
```

### DFPlayer Mini Wiring

```
┌────────────────────────────────────────────────────────────┐
│  Arduino Mega          │  DFPlayer Mini                    │
├────────────────────────┼───────────────────────────────────┤
│  Pin 7 (TX)            │  RX                               │
│  Pin 8 (RX)            │  TX                               │
│  5V                    │  VCC                              │
│  GND                   │  GND                              │
│                        │  SPK1 → Speaker +                 │
│                        │  SPK2 → Speaker -                 │
└────────────────────────┴───────────────────────────────────┘
```

---

## 🔌 iBus Channel Mapping

| Channel | Function | Description |
|---------|----------|-------------|
| **CH1** | Drive 1 | Left motor speed (tank-mixed) |
| **CH2** | Drive 2 | Right motor speed (tank-mixed) |
| **CH3** | Mainbar | Mainbar servo position |
| **CH4** | Head 1 | Head pitch movement |
| **CH5** | Head 2 | Head yaw movement |
| **CH6** | Head 3 | Head roll movement |
| **CH7** | Sound Mute | Sound on/off toggle |
| **CH8** | Sound Mode | Greeting / Default |
| **CH9** | Sound Mood | Negative / Neutral / Positive (3-pos) |
| **CH10** | Sound Squeak | Squeak sounds trigger |

---

## 🚀 Installation

### Step 1: Required Arduino Libraries

Install via Arduino Library Manager (Sketch → Include Library → Manage Libraries):

```
Required Libraries:
├── Wire                    (Arduino built-in)
├── IBusBM                  (by Bolder Flight Systems)
├── Servo                   (Arduino built-in)
├── SoftwareSerial          (Arduino built-in)
└── DFRobotDFPlayerMini     (by DFRobot)
```

### Step 2: Board Configuration

1. Open Arduino IDE
2. Go to **Tools → Board → Arduino Mega or Mega 2560**
3. Select correct **Port** under Tools menu
4. Set **Programmer** to "AVRISP mkII" (default)

### Step 3: Feature Configuration

Edit the feature flags at the top of the sketch:

```cpp
#define IBUS_ENABLED        // Enable iBus RC control
//#define MAINBAR_CORRECTION  // Uncomment to enable mainbar tilt correction
#define DFPLAYER_ENABLED    // Enable sound system
#define SERVOS_ENABLED      // Enable servo control
```

### Step 4: Upload

1. Connect Arduino Mega via USB
2. Open `D-O_printed_droid_rc_ibus_v1.1.ino`
3. Click **Upload** (Ctrl+U)
4. Wait for "Done uploading" message

---

## ⚙️ Configuration

### PID Tuning

These values control the self-balancing behavior. Edit in the code:

```cpp
float kp = 25;              // Proportional gain - main balance response
float ki = 0;               // Integral gain - usually keep at 0
float kd = 0.8;             // Derivative gain - damping/smoothing
float desired_angle = -0.3; // Target balance angle in degrees
```

**Tuning Tips:**
- If D-O oscillates: Decrease KP or increase KD
- If D-O is sluggish: Increase KP
- If D-O drifts slowly: Slightly increase KI (be careful!)
- Adjust `desired_angle` until D-O balances naturally

### Mainbar Correction (Optional)

```cpp
#define MAINBAR_CORRECTION  // Uncomment to enable

int centreangle = -10;      // Offset for center position
int anglestrength = 40;     // Correction sensitivity (lower = stronger)
```

### Sound Volume

Volume is set in the setup() function:

```cpp
myDFPlayer.volume(25);  // Volume 0-30
```

---

## 💾 SD Card File Structure

Create this folder structure on a FAT32 formatted Micro SD card:

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
└── 0020.mp3    Squeak 6
```

**Requirements:**
- SD card: FAT16 or FAT32 formatted
- Filenames: Exactly 4 digits (0001, 0002, etc.)
- Format: MP3, 128-320 kbps recommended

---

## 🎮 Usage

### First Startup

1. **Power on** with D-O in balanced upright position
2. **Wait for startup sound** (0001.mp3)
3. **Turn on RC transmitter** and verify connection
4. **D-O should self-balance** automatically

### RC Control

- **Left Stick (CH1/CH2)**: Drive forward/backward and turn
- **Right Stick or Dials**: Control head servos
- **Switches**: Trigger sounds based on channel mapping

### Sound Triggers

| Switch Position | Sound Category |
|----------------|----------------|
| CH7 Low→High | Unmute (plays default) |
| CH8 High | Play greeting |
| CH9 Low | Negative mood |
| CH9 Center | Neutral (no change) |
| CH9 High | Positive mood |
| CH10 High | Squeak sound |

---

## 🔍 Troubleshooting

### Problem: D-O doesn't balance / tips over

**Solutions:**
1. Check MPU6050 wiring (SDA=Pin 20, SCL=Pin 21)
2. Verify MPU6050 is level when D-O is balanced
3. Adjust `desired_angle` value (try -1.0 to 1.0)
4. Tune PID values (start with KP, then KD)
5. Check motor wiring (both motors same direction)

### Problem: No RC response / iBus not working

**Solutions:**
1. Check iBus wire connected to Pin 19 (Serial1 RX)
2. Verify receiver is bound to transmitter (solid LED)
3. Confirm `#define IBUS_ENABLED` is not commented out
4. iBus baudrate is 9600 in this version
5. Try different receiver or check iBus output setting

### Problem: No sound

**Solutions:**
1. SD card formatted as FAT16/FAT32?
2. Files in `/mp3/` folder with 4-digit names?
3. Check DFPlayer wiring (Pin 7→RX, Pin 8→TX)
4. Verify `#define DFPLAYER_ENABLED` is not commented out
5. Check speaker connection (SPK1/SPK2)

### Problem: Servos don't move

**Solutions:**
1. Check servo connections to Pins 0, 1, 5, 6
2. Verify `#define SERVOS_ENABLED` is not commented out
3. Check servo power supply (needs 5-6V BEC)
4. Try moving individual channels on RC

---

## ⬆️ Upgrading

### To v2.1.2

v2.1 adds many new features while remaining pin-compatible:
- ✨ Configuration menu via Serial Monitor
- ✨ EEPROM storage for all settings
- ✨ Battery monitoring with low-voltage protection
- ✨ Configurable iBus baudrate (9600 or 115200)
- ✨ Idle animations (sounds and movements)
- ✨ Adaptive PID (speed-dependent tuning)
- ✨ Dynamic lean angle
- ✨ RC Mixing Mode (Arcade/Tank)
- ✨ IMU clone support (MPU6500, MPU9250, MPU6886)

**No rewiring needed!** Simply upload the new sketch.

### To v3.3.1

v3 is a complete rewrite with advanced features:
- ✨ Simplified 2-mode system (PWM or iBus only)
- ✨ Watchdog timer for crash recovery
- ✨ I2C Fast Mode (400kHz) for faster IMU readings
- ✨ micros() timing for precise PID control
- ✨ All features from v2.1 included

**No rewiring needed!** (v3.2.3+ is pin-compatible)

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
   - Arduino IDE version
   - Error messages (if any)
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
- PCB Design: Nitewing / Printed-Droid.com

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
