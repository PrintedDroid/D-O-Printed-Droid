# D-O Self-Balancing Droid - ESP32-S3 Controller

![Version](https://img.shields.io/badge/version-1.1.1-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-green.svg)
![Board](https://img.shields.io/badge/board-TENSTAR%20TS--ESP32--S3-purple.svg)
![License](https://img.shields.io/badge/license-Non--Commercial-red.svg)

**Advanced ESP32-S3 based controller for D-O self-balancing droid with integrated TFT display**

This ESP32-S3 sketch provides complete control over a self-balancing D-O droid (from Star Wars) with onboard IMU, color TFT display, and advanced features not possible on Arduino Mega.

---

## 📋 Table of Contents

- [Safety & Disclaimer](#-safety--disclaimer)
- [Features](#-features)
- [Hardware Requirements](#-hardware-requirements)
- [Pin Configuration](#-pin-configuration)
- [I2C Devices](#-i2c-devices)
- [Button Functions](#-button-functions)
- [Display Modes](#-display-modes)
- [Installation](#-installation)
- [Configuration](#-configuration)
- [Usage](#-usage)
- [PID Auto-Tune](#-pid-auto-tune)
- [Troubleshooting](#-troubleshooting)
- [Community & Support](#-community--support)
- [Credits](#-credits)
- [License](#-license)

---

## ⚠️ Safety & Disclaimer

### Important Safety Warnings

**BEFORE YOU BEGIN - READ CAREFULLY:**

This project involves electrical components, lithium batteries, motors, and moving mechanical parts. Working with these components carries inherent risks including but not limited to:

- **Electrical hazards** from improper wiring or short circuits
- **Fire hazards** from lithium batteries if mishandled, overcharged, or damaged
- **Mechanical injuries** from moving servos, wheels, and the droid body
- **Property damage** from an uncontrolled or malfunctioning droid

### Safety Guidelines

1. **Battery Safety**
   - ALWAYS use a proper LiPo charger with balance charging capability
   - NEVER leave batteries charging unattended
   - NEVER use damaged, swollen, or punctured batteries
   - Store batteries in fireproof LiPo bags when not in use
   - Disconnect batteries immediately if any unusual behavior occurs
   - Monitor battery voltage - do not over-discharge below 3.3V per cell

2. **Electrical Safety**
   - Double-check all wiring before powering on
   - Use appropriate wire gauges for high-current connections
   - Ensure proper polarity on all connections
   - Use fuses or circuit breakers where appropriate
   - Keep liquids away from electronics

3. **Mechanical Safety**
   - Secure all mechanical parts properly before operation
   - Keep fingers, hair, and loose clothing away from moving parts
   - Test in an open area with soft flooring initially
   - Remove head during PID tuning to prevent damage
   - Be prepared to catch the droid if it loses balance
   - Keep bystanders at a safe distance during testing

4. **Operating Safety**
   - Always supervise the droid during operation
   - Have a clear "kill switch" ready (disconnect battery)
   - Test new code changes incrementally
   - Start with low motor speeds and increase gradually
   - Never operate near stairs, ledges, or obstacles

### Disclaimer

**THIS PROJECT IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.**

The creators, contributors, and community members of this project assume NO responsibility or liability for:

- Any damage to property, equipment, or electronic components
- Personal injury to yourself or others
- Fire, electrical damage, or any other hazards
- Malfunction or unexpected behavior of the droid
- Any direct, indirect, incidental, or consequential damages

**BY USING THIS CODE AND BUILDING THIS PROJECT, YOU ACKNOWLEDGE:**

1. You understand and accept all risks involved
2. You take full responsibility for your own safety and the safety of others
3. You will follow all applicable safety guidelines and local regulations
4. You have adequate knowledge and skills for this type of project
5. Any modifications or changes you make are your own responsibility

**This is a hobby project for educational and entertainment purposes. If you are not comfortable with electronics, programming, or working with lithium batteries, please seek assistance from someone with appropriate experience.**

---

## 🎯 Features

### Core Capabilities
- ✅ **QMI8658C IMU Self-Balancing** with PID control (onboard sensor!)
- ✅ **Integrated 240x135 TFT Display** for real-time telemetry
- ✅ **iBus RC Control** via FlySky receiver
- ✅ **4 Servo Control** for head and mainbar movement
- ✅ **DFPlayer Sound System** with 30+ sound effects
- ✅ **WS2812 NeoPixel** status LED
- ✅ **4 Button Interface** for complete control without Serial Monitor
- ✅ **EEPROM Persistence** - all settings saved across reboots

### Advanced Features
- **PID Auto-Tune** - Automatic PID parameter optimization
- **Real-time Telemetry** - Live pitch, roll, yaw, motor speeds on TFT
- **Diagnostics Mode** - System health monitoring
- **IMU Fusion Ready** - Architecture supports multiple IMUs
- **Watchdog Timer** - Automatic crash recovery (10s timeout)
- **100Hz IMU Update Rate** - Smooth balance control
- **Complementary Filter** - Stable angle estimation
- **Motor Ramping** - Smooth acceleration/deceleration
- **Emergency Stop** - Automatic shutdown at 45° tilt

### ESP32-S3 Advantages over Arduino Mega
| Feature | Arduino Mega | ESP32-S3 |
|---------|--------------|----------|
| **CPU Speed** | 16 MHz | 240 MHz |
| **RAM** | 8 KB | 512 KB |
| **Display** | None | 240x135 TFT |
| **IMU** | External MPU6050 | Onboard QMI8658C |
| **Status LED** | Simple LED | RGB NeoPixel |
| **WiFi/BT** | None | Built-in |
| **USB** | Serial only | Native USB |

---

## 🔧 Hardware Requirements

### Main Board

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Board** | TENSTAR TS-ESP32-S3 | Adafruit Feather ESP32-S3 TFT Clone |
| **Display** | ST7789 240x135 | Integrated on board |
| **IMU** | QMI8658C 6-axis | Integrated on board (I2C 0x6B) |
| **LED** | WS2812 NeoPixel | Integrated on board (GPIO 33) |

### External Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Motor Driver** | Cytron MD10C (2x) | Or similar dual H-bridge |
| **Sound Module** | DFPlayer Mini | With Micro SD card |
| **RC Receiver** | FlySky FS-iA6B | iBus protocol required |
| **Servos** | 4x Standard Servos | Head (3x) + Mainbar (1x) |
| **Battery** | 2S-3S LiPo | 7.4V - 11.1V |
| **Speaker** | 8 Ohm, <3W | For DFPlayer |
| **Buttons** | 4x Tactile Switches | Active LOW with internal pullup |

### Optional Components
- 1kΩ resistor for DFPlayer TX line (recommended)
- External IMU (LSM6DS3, MPU6050) for redundancy

---

## 📍 Pin Configuration

### Complete Pinout Table

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                    TENSTAR TS-ESP32-S3 PIN ASSIGNMENT                        │
├──────────────────────────────────────────────────────────────────────────────┤
│  GPIO │ Function           │ Direction │ Notes                              │
├───────┼────────────────────┼───────────┼────────────────────────────────────┤
│    0  │ BUTTON4            │ INPUT     │ Boot pin - safe after boot         │
│    1  │ BUTTON3            │ INPUT     │ TX0 - USB Serial still works       │
│    2  │ iBus RX            │ INPUT     │ RC Receiver iBus signal            │
│    5  │ BUTTON1            │ INPUT     │ Start/Stop button                  │
│    6  │ SERVO_MAINBAR      │ OUTPUT    │ PWM servo control                  │
│    7  │ TFT_CS             │ OUTPUT    │ Display chip select (internal)     │
│    8  │ SERVO_HEAD3        │ OUTPUT    │ PWM servo control                  │
│    9  │ MOTOR2_PWM         │ OUTPUT    │ Motor 2 speed (LEDC)               │
│   10  │ MOTOR2_DIR         │ OUTPUT    │ Motor 2 direction                  │
│   11  │ MOTOR1_PWM         │ OUTPUT    │ Motor 1 speed (LEDC)               │
│   12  │ BUTTON2            │ INPUT     │ Calibrate/Setup button             │
│   13  │ MOTOR1_DIR         │ OUTPUT    │ Motor 1 direction                  │
│   14  │ SERVO_HEAD2        │ OUTPUT    │ PWM servo control                  │
│   15  │ SERVO_HEAD1        │ OUTPUT    │ PWM servo control                  │
│   16  │ (Reserved)         │ -         │ MPU6050 INT (not used)             │
│   17  │ DFPLAYER_RX        │ INPUT     │ ESP RX ← DFPlayer TX               │
│   18  │ DFPLAYER_TX        │ OUTPUT    │ ESP TX → DFPlayer RX               │
│   21  │ I2C_POWER          │ OUTPUT    │ CRITICAL! Must be HIGH for I2C     │
│   33  │ NEOPIXEL           │ OUTPUT    │ WS2812 RGB LED data                │
│   35  │ TFT_MOSI           │ OUTPUT    │ Display SPI data (internal)        │
│   36  │ TFT_SCLK           │ OUTPUT    │ Display SPI clock (internal)       │
│   37  │ TFT_MISO           │ INPUT     │ Display SPI (internal)             │
│   39  │ TFT_DC             │ OUTPUT    │ Display data/command (internal)    │
│   40  │ TFT_RST            │ OUTPUT    │ Display reset (internal)           │
│   41  │ I2C_SCL            │ I/O       │ I2C clock (QMI8658C)               │
│   42  │ I2C_SDA            │ I/O       │ I2C data (QMI8658C)                │
│   45  │ TFT_BL             │ OUTPUT    │ Display backlight (internal)       │
└───────┴────────────────────┴───────────┴────────────────────────────────────┘
```

### Motor Controller Wiring (Cytron MD10C)

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3              │  Cytron MD10C #1  │  Cytron MD10C #2 │
├────────────────────────┼───────────────────┼──────────────────┤
│  GPIO 13 (MOTOR1_DIR)  │  DIR              │                  │
│  GPIO 11 (MOTOR1_PWM)  │  PWM              │                  │
│  GPIO 10 (MOTOR2_DIR)  │                   │  DIR             │
│  GPIO  9 (MOTOR2_PWM)  │                   │  PWM             │
│  GND                   │  GND              │  GND             │
└────────────────────────┴───────────────────┴──────────────────┘

Note: Motor driver power (VM) from battery, logic power (VCC) from 5V
```

### Servo Wiring

```
┌────────────────────────────────────────────────────────┐
│  ESP32-S3              │  Servo            │  Function │
├────────────────────────┼───────────────────┼───────────┤
│  GPIO  6               │  Mainbar Servo    │  Balance  │
│  GPIO 15               │  Head Servo 1     │  Pitch    │
│  GPIO 14               │  Head Servo 2     │  Yaw      │
│  GPIO  8               │  Head Servo 3     │  Roll     │
└────────────────────────┴───────────────────┴───────────┘

Note: Servo power (5-6V) from BEC or separate supply, NOT from ESP32!
      Only signal wire to ESP32 GPIO.
```

### DFPlayer Mini Wiring

```
┌────────────────────────────────────────────────────────┐
│  ESP32-S3              │  DFPlayer Mini                │
├────────────────────────┼───────────────────────────────┤
│  GPIO 18 (TX)          │  RX (via 1kΩ resistor)        │
│  GPIO 17 (RX)          │  TX                           │
│  3.3V                  │  VCC                          │
│  GND                   │  GND                          │
│                        │  SPK1 → Speaker +             │
│                        │  SPK2 → Speaker -             │
└────────────────────────┴───────────────────────────────┘

Note: 1kΩ resistor between ESP32 TX and DFPlayer RX recommended!
```

### RC Receiver Wiring (iBus)

```
┌────────────────────────────────────────────────────────┐
│  ESP32-S3              │  FlySky FS-iA6B               │
├────────────────────────┼───────────────────────────────┤
│  GPIO 2                │  iBus / Servo (single wire)   │
│  5V (or VIN)           │  VCC                          │
│  GND                   │  GND                          │
└────────────────────────┴───────────────────────────────┘

Note: iBus uses inverted serial at 115200 baud
```

### Button Wiring

```
┌────────────────────────────────────────────────────────┐
│  Button  │  GPIO  │  Wiring                           │
├──────────┼────────┼───────────────────────────────────┤
│  BTN1    │    5   │  Button between GPIO and GND      │
│  BTN2    │   12   │  Button between GPIO and GND      │
│  BTN3    │    1   │  Button between GPIO and GND      │
│  BTN4    │    0   │  Button between GPIO and GND      │
└──────────┴────────┴───────────────────────────────────┘

Note: Internal pullups enabled - no external resistors needed!
      GPIO 0 is boot pin - hold during reset for bootloader
      GPIO 1 is TX0 - USB Serial still works normally
```

---

## 📡 I2C Devices

### I2C Bus Configuration

```
I2C Speed: 100 kHz (Standard Mode)
SDA: GPIO 42
SCL: GPIO 41
Power Enable: GPIO 21 (MUST be HIGH!)
```

### Connected Devices

| Device | Address | Function | Status |
|--------|---------|----------|--------|
| **QMI8658C** | 0x6B | 6-axis IMU (Accel + Gyro) | Onboard - Primary |
| MPU6050 | 0x68 | 6-axis IMU | Optional - Disabled |
| BNO055 | 0x28 | 9-axis IMU | Not recommended (clock stretching) |
| BMP280 | 0x77 | Pressure sensor | Optional |

### QMI8658C IMU Specifications

```
┌─────────────────────────────────────────────────────────┐
│  QMI8658C - 6-Axis Inertial Measurement Unit            │
├─────────────────────────────────────────────────────────┤
│  Accelerometer Range    │  ±2g / ±4g / ±8g / ±16g      │
│  Gyroscope Range        │  ±16 / ±32 / ±64 / ... dps   │
│  Output Data Rate       │  Up to 8000 Hz               │
│  Interface              │  I2C / SPI                   │
│  Operating Voltage      │  1.71V - 3.6V                │
│  Temperature Sensor     │  Built-in                    │
├─────────────────────────────────────────────────────────┤
│  Advantages over MPU6050:                               │
│  • No clock stretching issues with ESP32                │
│  • Higher ODR capability                                │
│  • Better noise performance                             │
│  • Lower power consumption                              │
└─────────────────────────────────────────────────────────┘
```

---

## 🔘 Button Functions

### Single Button Press (Short)

| Button | STATE_READY | STATE_RUNNING | STATE_ERROR |
|--------|-------------|---------------|-------------|
| **B1** | Start balancing | Stop balancing | Restart system |
| **B2** | Start IMU calibration | - | - |
| **B3** | Cycle display mode | - | - |
| **B4** | Toggle debug output | - | - |

### Long Press (Hold 1+ second)

| Button | Function |
|--------|----------|
| **B1** | Emergency Stop |
| **B2** | Enter PID Menu |
| **B3** | Toggle "Display Always On" |
| **B4** | Show System Info |

### Button Combinations

| Combo | Hold Time | Function |
|-------|-----------|----------|
| **B1 + B2** | 2 seconds | System Reset (ESP.restart) |
| **B2 + B3** | Instant | Toggle Sound On/Off |
| **B3 + B4** | 3 seconds | Factory Reset (clear all EEPROM) |

### PID Menu Navigation

When in PID Menu (B2 long press):

| Button | Function |
|--------|----------|
| **B1** | Exit menu and save |
| **B3** | Select next parameter (Kp → Ki → Kd → Auto-Tune) |
| **B4** | Adjust selected value (+0.5 for Kp, +0.1 for Ki/Kd) |

---

## 🖥️ Display Modes

### Mode 0: Telemetry (Default)

```
┌─────────────────────────────────────────┐
│  D-O TELEMETRY                    RC:✓  │
├─────────────────────────────────────────┤
│  Pitch:  -2.3°    Roll:   0.5°          │
│  Yaw:    45.2°    Hz:     98            │
│                                         │
│  M1: ████████░░  156                    │
│  M2: ████████░░  148                    │
│                                         │
│  [Status messages appear here]          │
└─────────────────────────────────────────┘
```

### Mode 1: Diagnostics

```
┌─────────────────────────────────────────┐
│  DIAGNOSTICS                            │
├─────────────────────────────────────────┤
│  IMU:    QMI OK                         │
│  Sound:  ENABLED                        │
│  Temp:   32.5°C                         │
│  Heap:   245.3 KB free                  │
│  Uptime: 01:23:45                       │
└─────────────────────────────────────────┘
```

### Mode 2: Display Off

Display backlight turns off to maximize performance during balancing.
(Can be overridden with "Display Always On" setting)

---

## 🚀 Installation

### 1. Required Arduino Libraries

Install via Arduino Library Manager:

```
Required Libraries:
├── Wire (Arduino built-in)
├── SPI (Arduino built-in)
├── ESP32Servo (by Kevin Harrington)
├── Preferences (ESP32 built-in)
├── Adafruit_GFX (by Adafruit)
├── Adafruit_ST7789 (by Adafruit)
├── Adafruit_NeoPixel (by Adafruit)
├── DFRobotDFPlayerMini (by DFRobot)
└── SensorLib (by Lewis He) - for QMI8658C
```

### 2. Board Setup

1. **Install ESP32 Board Package:**
   - Arduino IDE → Preferences → Additional Board URLs:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
   - Tools → Board Manager → Search "ESP32" → Install

2. **Select Board:**
   - Tools → Board → ESP32S3 Dev Module
   - Or: Adafruit Feather ESP32-S3 TFT

3. **Board Settings:**
   ```
   USB CDC On Boot: Enabled
   USB Mode: Hardware CDC and JTAG
   Flash Size: 4MB (or 8MB/16MB depending on board)
   Partition Scheme: Default 4MB with spiffs
   PSRAM: Disabled (or OPI if available)
   ```

### 3. Upload Sketch

1. Open `D_O_ESP32_S3.ino` in Arduino IDE
2. Connect ESP32-S3 via USB-C
3. Select correct port in Tools → Port
4. Upload: Sketch → Upload (Ctrl+U)

### 4. Hardware Assembly

1. **Connect I2C Power first!** GPIO 21 must be HIGH for I2C to work
2. Wire motor drivers (see pinout above)
3. Connect servos (signal only - separate power!)
4. Wire DFPlayer Mini with 1kΩ resistor
5. Connect iBus receiver
6. Install buttons between GPIO and GND

---

## ⚙️ Configuration

### First Boot

1. **Power on ESP32-S3**
2. **Wait for splash screen** on TFT
3. **"System Ready"** appears with green LED
4. **Perform IMU Calibration:**
   - Place D-O in balanced upright position
   - Press B2 (short press)
   - Keep absolutely still for 5 seconds
   - "Calibration Complete" appears

### PID Tuning

**Default PID Values:**
```
Kp: 12.0  (Proportional)
Ki:  0.8  (Integral)
Kd:  0.5  (Derivative)
```

**Manual Tuning:**
1. Long press B2 to enter PID menu
2. Use B3 to select parameter
3. Use B4 to increase value
4. Press B1 to save and exit

**Tuning Tips:**
- D-O oscillates: Decrease Kp, increase Kd
- D-O sluggish: Increase Kp
- D-O drifts slowly: Increase Ki (carefully!)
- Values wrap around at maximum

### Sound Configuration

**Volume:** Set in `config.h` → `DEFAULT_VOLUME` (0-30)

**Sound Toggle:** Press B2+B3 simultaneously

---

## 🔊 SD Card Sound Files

Place in `/mp3/` folder on Micro SD card:

```
/mp3/
  0001.mp3 - Startup sound
  0002.mp3 - Default/ready sound
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

**Requirements:**
- SD card: FAT16 or FAT32 formatted
- Filenames: Exactly 4 digits (0001, 0002, etc.)
- Format: MP3, 128-320 kbps recommended

---

## 🎮 Usage

### Starting D-O

1. **Power on** - Splash screen appears
2. **Wait for "System Ready"** - Green LED
3. **Press B1** to start balancing
4. **D-O self-balances** automatically

### RC Control (iBus Channels)

| Channel | Function |
|---------|----------|
| CH1 | Drive - Left motor |
| CH2 | Drive - Right motor |
| CH3 | Mainbar servo |
| CH4 | Head Servo 1 (Pitch) |
| CH5 | Head Servo 2 (Yaw) |
| CH6 | Head Servo 3 (Roll) |
| CH7 | Sound Mute |
| CH8 | Sound Mode |
| CH9 | Sound Mood (3-position) |
| CH10 | Squeak sounds |

### Stopping D-O

- **Normal Stop:** Press B1
- **Emergency Stop:** Long press B1 (or tilt >45°)
- **After Emergency:** Press B1 to restart system

---

## 🔧 PID Auto-Tune

The ESP32-S3 version includes automatic PID tuning!

### Starting Auto-Tune

1. Ensure D-O is in READY state
2. Long press B2 to enter PID menu
3. Press B3 until "Auto-Tune" is selected
4. Press B4 to start
5. **Remove D-O head** before starting!
6. Press B1 to confirm start

### During Auto-Tune

- D-O will oscillate intentionally
- Progress shown on display (0-100%)
- Takes approximately 30 seconds
- Press B1 to abort at any time

### After Auto-Tune

- New PID values automatically applied
- Values saved to EEPROM
- System returns to READY state
- Test balance before mounting head!

---

## 🔍 Troubleshooting

### Problem: Display shows nothing / white screen

**Solution:**
1. Check GPIO 21 (I2C_POWER) is set HIGH in code
2. Verify TFT pins are not used for other functions
3. Check if using correct board definition
4. Try reducing SPI speed in display init

### Problem: IMU not detected / "No IMU Found"

**Solution:**
1. **CRITICAL:** Ensure GPIO 21 is HIGH (I2C power enable!)
2. Check I2C wiring (SDA=GPIO 42, SCL=GPIO 41)
3. Run I2C scanner to verify address 0x6B
4. Check SensorLib is installed correctly

### Problem: D-O doesn't balance / tips over

**Solution:**
1. Perform IMU calibration (B2 short press)
2. Check PID values (try defaults: Kp=12, Ki=0.8, Kd=0.5)
3. Verify motors are wired correctly (same direction)
4. Check motor driver connections
5. Ensure adequate battery voltage

### Problem: No sound from DFPlayer

**Solution:**
1. SD card formatted as FAT16/FAT32?
2. Files in `/mp3/` folder with 4-digit names?
3. 1kΩ resistor between ESP TX and DFPlayer RX?
4. Speaker connected to SPK1/SPK2?
5. Check volume setting (0-30)

### Problem: RC not responding / "RC: ✗"

**Solution:**
1. Receiver bound to transmitter?
2. iBus wire connected to GPIO 2?
3. Receiver powered (5V)?
4. Check if receiver LED is solid (not blinking)
5. Verify iBus output enabled on receiver

### Problem: Servos jittering or not moving

**Solution:**
1. Servos need separate power (5-6V BEC)
2. Only signal wire to ESP32
3. Check servo pin assignments
4. Verify not exceeding servo current limits

### Problem: "EMERGENCY STOP" immediately

**Solution:**
1. Check if D-O is tilted >45°
2. Verify IMU calibration was done
3. Check IMU orientation (correct mounting)
4. Look at pitch angle on display

### Problem: System resets randomly

**Solution:**
1. Check power supply capacity
2. Motor stall causing brownout?
3. Add capacitor to power rail (1000µF)
4. Check Serial Monitor for reset reason
5. Watchdog timeout = infinite loop in code

---

## 📊 Technical Specifications

### Performance

| Parameter | Value |
|-----------|-------|
| **IMU Update Rate** | 100 Hz |
| **PID Loop Rate** | 100 Hz |
| **Display Update** | 10 Hz |
| **RC Update** | 20 Hz |
| **Servo Update** | 50 Hz |
| **Watchdog Timeout** | 10 seconds |

### PID Control

```
Output = (Kp × Error) + (Ki × ∫Error) + (Kd × dError/dt)

Where:
  Error = Target Angle - Current Angle
  Target Angle = 0° (upright)
  Output limits: ±255 (PWM)
  Integral anti-windup: ±100
```

### Complementary Filter

```
Angle = α × (Angle + Gyro × dt) + (1-α) × Accel_Angle

Where:
  α = 0.98 (trust gyro 98%)
  dt = loop time (~10ms)
```

### Memory Usage (Typical)

```
Free Heap at boot:  ~300 KB
Free Heap running:  ~240 KB
Stack size:         8 KB
```

---

## 📁 File Structure

```
D_O_ESP32_S3/
├── D_O_ESP32_S3.ino      # Main sketch
├── config.h              # Pin definitions & settings
├── imu_handler.h         # IMU class header
├── imu_handler.cpp       # IMU implementation
├── display_handler.h     # Display class header
├── display_handler.cpp   # Display implementation
├── sound_controller.h    # Sound class header
├── sound_controller.cpp  # Sound implementation
├── rc_receiver.h         # RC receiver header
├── rc_receiver.cpp       # RC receiver implementation
├── button_handler.h      # Button handler header
├── button_handler.cpp    # Button handler implementation
├── pid_autotune.h        # PID auto-tune header
├── pid_autotune.cpp      # PID auto-tune implementation
├── utilities.h           # Helper functions header
├── utilities.cpp         # Helper functions implementation
└── README.md             # This file
```

---

## 🌐 Community & Support

### Getting Help

If you need assistance with your D-O droid build, have questions, or want to share your progress:

### Official Resources

| Resource | Link | Description |
|----------|------|-------------|
| **Website** | [www.printed-droid.com](https://www.printed-droid.com) | Main project website with guides and resources |
| **Facebook Group** | [Printed Droid Community](https://www.facebook.com/groups/printeddroid/) | Active community for support, sharing, and discussion |

### How to Get Support

1. **Check the Troubleshooting Section** - Most common issues are documented above
2. **Search the Facebook Group** - Your question may already be answered
3. **Post in the Facebook Group** - Include:
   - Which version you're using (ESP32-S3)
   - What you've tried so far
   - Photos/videos of your setup if relevant
   - Serial Monitor output if applicable
   - Board and component specifications

### Contributing

This is a community project, and contributions are welcome:

- **Bug Reports:** Post issues in the Facebook group with detailed description
- **Feature Requests:** Share your ideas with the community
- **Code Improvements:** Fork the repository and submit pull requests
- **Documentation:** Help improve guides and documentation
- **Testing:** Test new features and provide feedback

### Community Guidelines

- Be respectful and helpful to fellow builders
- Share your builds and progress - we love seeing D-O droids!
- Help newcomers get started
- Give credit when using others' work or modifications
- Keep discussions constructive and on-topic

---

## 👥 Credits

**Original Design:**
- D-O Droid Design: Star Wars / Lucasfilm

**Software:**
- ESP32-S3 Version: Printed-Droid.com (2024-2025)
- Base concepts from Arduino Mega version

**Libraries:**
- ESP32Servo: Kevin Harrington
- SensorLib (QMI8658C): Lewis He
- Adafruit GFX/ST7789: Adafruit Industries
- DFRobotDFPlayerMini: DFRobot

---

## 📄 License

### Non-Commercial License

This project is released under a **Non-Commercial License**.

```
D-O Printed Droid - Non-Commercial License

Copyright (c) 2024-2025 Printed-Droid.com Community

TERMS AND CONDITIONS:

1. GRANT OF LICENSE
   You are granted a non-exclusive, non-transferable license to use, copy,
   and modify this software and associated documentation for PERSONAL,
   NON-COMMERCIAL purposes only.

2. RESTRICTIONS
   You may NOT:
   - Sell this software or any derivative works
   - Use this software for commercial purposes
   - Include this software in commercial products or services
   - Charge money for droids built using this software
   - Use this software for commercial exhibitions or shows (paid entry)

3. ATTRIBUTION
   You must give appropriate credit to the Printed-Droid.com community,
   provide a link to the original source, and indicate if changes were made.

4. SHARE-ALIKE
   If you modify or build upon this software, you must distribute your
   contributions under the same non-commercial license.

5. DISCLAIMER
   THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND.
   THE AUTHORS ARE NOT LIABLE FOR ANY DAMAGES ARISING FROM USE OF
   THIS SOFTWARE.

6. EXCEPTIONS
   For commercial licensing inquiries, please contact the community
   administrators through www.printed-droid.com

By using this software, you agree to these terms.
```

### What This Means

| Allowed | Not Allowed |
|---------|-------------|
| Personal builds for yourself | Selling droids for profit |
| Sharing at free community events | Commercial exhibitions (paid entry) |
| Educational use in schools | Including in commercial products |
| Modifying for personal use | Selling modifications or code |
| Sharing your improvements (with same license) | Removing attribution |

**Note:** Star Wars and D-O are trademarks of Lucasfilm/Disney. This is a fan project and not affiliated with or endorsed by Lucasfilm or Disney.

---

<div align="center">

**May the Force be with your D-O!**

*For the latest updates and community support, visit: [www.printed-droid.com](https://www.printed-droid.com)*

*Join our community: [Facebook Group](https://www.facebook.com/groups/printeddroid/)*

</div>
