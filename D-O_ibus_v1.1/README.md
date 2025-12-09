# D-O Self-Balancing Droid - iBus Controller v1.1

![Version](https://img.shields.io/badge/version-1.1-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![License](https://img.shields.io/badge/license-MIT-orange.svg)

**Original iBus control system for self-balancing D-O droid**

This is the original stable Arduino sketch for the D-O Printed Droid. It provides reliable self-balancing control with iBus RC support.

---

## 📋 Table of Contents

- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Pin Configuration](#-pin-configuration)
- [iBus Channel Mapping](#-ibus-channel-mapping)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [SD Card File Structure](#-sd-card-file-structure)
- [Troubleshooting](#-troubleshooting)
- [Upgrading](#-upgrading)
- [Credits](#-credits)

---

## 🎯 Features

### Core Capabilities
- ✅ **MPU6050 IMU Self-Balancing** with PID control
- ✅ **iBus RC Support** (10 channels via Serial1)
- ✅ **DFPlayer Mini Sound System** with multiple sound categories
- ✅ **4 Servo Control** for head and mainbar movement
- ✅ **Mainbar Tilt Correction** (optional)
- ✅ **Proven Stable** - tested and working!

### Sound Categories
- Startup sounds
- Greeting sounds (3 tracks)
- Negative mood sounds (4 tracks)
- Positive mood sounds (5 tracks)
- Squeaky wheel sounds (6 tracks)

---

## 🔧 Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Microcontroller** | Arduino Mega 2560 | Required for Serial1 |
| **IMU** | MPU6050 | I2C address 0x68 |
| **Motor Driver** | Cytron MD10C or similar | 2x for differential drive |
| **Sound Module** | DFPlayer Mini | With Micro SD card |
| **RC Receiver** | iBus-compatible | FlySky FS-iA6B recommended |
| **Servos** | 4x Standard Servos | For head and mainbar |
| **Speaker** | 8 Ohm, <3W | For DFPlayer |

---

## 📍 Pin Configuration

### Motors
```
Pin 13 → Motor 1 Direction (DIR1)
Pin 12 → Motor 1 Speed (PWM1)
Pin 11 → Motor 2 Direction (DIR2)
Pin 10 → Motor 2 Speed (PWM2)
```

### Servos
```
Pin 0 → Mainbar Servo
Pin 1 → Head Servo 1
Pin 5 → Head Servo 2
Pin 6 → Head Servo 3
```

### iBus Input
```
Pin 19 (Serial1 RX) → iBus Signal from receiver
```

### DFPlayer Mini
```
Pin 7 → DFPlayer RX (directly from Mega, no resistor needed in v1.1)
Pin 8 → DFPlayer TX
```

### PWM Fallback (if iBus disabled)
```
Pin 3 → Drive Channel 1 (PWM input)
Pin 4 → Drive Channel 2 (PWM input)
Pin 14 (A0) → Sound Switch 1
Pin 15 (A1) → Sound Switch 2
Pin 16 (A2) → Sound Switch 3
Pin 17 (A3) → Sound Switch 4
```

### I2C (MPU6050)
```
Pin 20 → SDA
Pin 21 → SCL
```

---

## 🔌 iBus Channel Mapping

| Channel | Function | Description |
|---------|----------|-------------|
| CH1 | Drive 1 | Left motor (tank-mixed) |
| CH2 | Drive 2 | Right motor (tank-mixed) |
| CH3 | Mainbar | Mainbar servo position |
| CH4 | Head 1 | Head pitch movement |
| CH5 | Head 2 | Head yaw movement |
| CH6 | Head 3 | Head roll movement |
| CH7 | Sound Mute | Sound on/off toggle |
| CH8 | Sound Mode | Greeting / Default |
| CH9 | Sound Mood | Negative / Neutral / Positive (3-pos) |
| CH10 | Sound Squeak | Squeak sounds |

---

## 🚀 Installation

### 1. Required Arduino Libraries

Install via Arduino Library Manager:

- **Wire** (Arduino built-in)
- **IBusBM** (by Bolder Flight Systems)
- **Servo** (Arduino built-in)
- **SoftwareSerial** (Arduino built-in)
- **DFRobotDFPlayerMini** (by DFRobot)

### 2. Feature Flags

Edit these at the top of the sketch to enable/disable features:

```cpp
#define IBUS_ENABLED        // Enable iBus (comment out for PWM mode)
//#define MAINBAR_CORRECTION  // Enable mainbar tilt correction
#define DFPLAYER_ENABLED    // Enable sound system
#define SERVOS_ENABLED      // Enable servo control
```

### 3. Upload Sketch

1. Open `D-O_printed_droid_rc_ibus_v1.1.ino` in Arduino IDE
2. Select board: **Tools → Board → Arduino Mega 2560**
3. Select port: **Tools → Port → [Your COM Port]**
4. Upload: **Sketch → Upload**

---

## ⚙️ Configuration

### PID Tuning

Edit these values in the code:

```cpp
float kp = 25;      // Proportional gain
float ki = 0;       // Integral gain (usually 0)
float kd = 0.8;     // Derivative gain
float desired_angle = -0.3;  // Target balance angle
```

### Mainbar Correction

```cpp
int centreangle = -10;      // Offset for center position
int anglestrength = 40;     // Correction sensitivity (lower = stronger)
```

### iBus Baudrate

```cpp
Serial1.begin(9600);  // iBus baudrate (9600 works with this setup)
```

---

## 💾 SD Card File Structure

Audio files must be placed on the Micro SD card in the `/mp3/` folder:

```
/mp3/
  0001.mp3 - Startup sound
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
```

**Important:**
- SD card must be formatted FAT16 or FAT32
- Filenames must be exactly 4 digits (0001, 0002, etc.)

---

## 🔍 Troubleshooting

### Problem: D-O doesn't balance

**Solution:**
1. Check MPU6050 wiring (I2C: SDA=20, SCL=21)
2. Adjust `desired_angle` value
3. Tune PID values (start with KP, then KD)

### Problem: No RC response

**Solution:**
1. Check iBus connection to Pin 19
2. Verify receiver is bound to transmitter
3. Check that `#define IBUS_ENABLED` is not commented out
4. iBus baudrate is 9600 in this version

### Problem: No sound

**Solution:**
1. Check SD card is formatted FAT16/FAT32
2. Verify files are in `/mp3/` folder with correct names
3. Check DFPlayer wiring (RX=7, TX=8)
4. Check that `#define DFPLAYER_ENABLED` is not commented out

### Problem: Servos don't move

**Solution:**
1. Check servo connections (Pins 0, 1, 5, 6)
2. Verify `#define SERVOS_ENABLED` is not commented out
3. Check servo power supply

---

## ⬆️ Upgrading

### To v2.1
v2.1 adds:
- Configuration menu via Serial Monitor
- EEPROM storage for settings
- Battery monitoring
- Configurable iBus baudrate
- Idle animations
- Adaptive PID

**Pin-compatible!** No rewiring needed.

### To v3.2.3
v3.2.3 adds:
- Simplified 2-mode system (PWM or iBus)
- Watchdog timer
- IMU error recovery
- micros() timing for better PID
- I2C Fast Mode (400kHz)

**Pin-compatible!** No rewiring needed (v3.2.3+).

---

## 👥 Credits

**Original Design:**
- D-O Droid Design: Star Wars / Lucasfilm
- 3D Model: [Printed-Droid.com](https://www.printed-droid.com)

**Software:**
- Original code: Reinhard Stockinger (2020)
- PCB Design: Nitewing

---

## 📄 License

MIT License - See LICENSE file for details.

---

## 💬 Support

- GitHub Issues: [Create an issue](https://github.com/PrintedDroid/D-O-Printed-Droid/issues)
- Forum: [Printed-Droid.com Forum](https://www.printed-droid.com/forum)

---

<div align="center">

**May the Force be with your D-O!** ⚡🤖

*For the latest updates, visit: [www.printed-droid.com](https://www.printed-droid.com)*

</div>
