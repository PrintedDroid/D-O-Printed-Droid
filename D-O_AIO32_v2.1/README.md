# D-O AIO32 v2.1 Controller — ESP32-S3

![Version](https://img.shields.io/badge/firmware-2.1.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-green.svg)
![Board](https://img.shields.io/badge/board-D--O%20AIO32%20v2.1-purple.svg)
![Status](https://img.shields.io/badge/status-experimental-orange.svg)

> **⚠ Experimental — active development branch.** The D-O AIO32 v2.1 controller is a second-generation D-O controller for the same droid chassis as the V1.6 / V1.7 Mega boards, built around an ESP32-S3 with integrated TFT and a dedicated on-board IMU. Transmitter setup, wiring to motors/servos/DFPlayer, sound files, and iBus channel map are intentionally identical to the Mega line so the same physical droid can be moved between controllers. See [`../README.md`](../README.md) for the sketch inventory.

---

## Hardware

- **Controller board:** D-O AIO32 v2.1 (Printed-Droid.com)
- **MCU module:** TENSTAR TS-ESP32-S3 (plugged in) — ESP32-S3FH4R2, 1.14" ST7789 TFT, QMI8658C IMU, BMP280 pressure sensor, WS2812 NeoPixel
- **On-board IMU (AIO32 PCB):** GY-LSM6DS3 at I²C 0x6A — fixed to droid frame, preferred primary for balance once integrated
- **Motor driver:** Cytron MDD10A (dual-channel, 10 A per channel)
- **Sound:** DFPlayer Mini + MicroSD (FAT16/32)
- **RC:** FlySky iBus-capable 10-channel receiver (e.g. FS-RX2A Pro, or any standard PWM receiver that exposes an iBus output)
- **Servos:** 1 × MG996R (mainbar) + 3 × MG90 (head pitch/yaw/roll)
- **Battery:** 2 × 2S LiPo in series = 4S (16.8 V full, 14.8 V nominal, 12.0 V empty)
- **User interface:** 6 tact buttons via two ADC resistor ladders, onboard NeoPixel, 240 × 135 TFT

---

## Pin map (AIO32 v2.1 connectors)

| Mega-Linie Äquivalent | AIO32-Label | ESP32-S3 GPIO | Function |
|-----------------------|:-----------:|:-------------:|----------|
| Motor 1 DIR | — | **13** | Cytron MDD10A channel 1 direction |
| Motor 1 PWM | — | **11** | Cytron MDD10A channel 1 speed |
| Motor 2 DIR | — | **10** | Cytron MDD10A channel 2 direction |
| Motor 2 PWM | — | **9**  | Cytron MDD10A channel 2 speed |
| iBus RX | — | **2**  | Receiver iBus signal (Serial1) |
| Servo CH3 | 3 | **6**  | Mainbar servo (FlySky CH3) |
| Servo CH4 | 4 | **15** | Head pitch (FlySky CH4) |
| Servo CH5 | 5 | **14** | Head yaw (FlySky CH5) |
| Servo CH6 | 6 | **8**  | Head roll (FlySky CH6) |
| DFPlayer TX | — | **18** | ESP → DFPlayer (via 1 kΩ) |
| DFPlayer RX | — | **17** | ESP ← DFPlayer |
| Button ladder 1–3 | — | **5** (ADC1_CH4) | resistor ladder, see below |
| Button ladder 4–6 | — | **12** (ADC2_CH1) | resistor ladder, see below |
| Battery voltage sense | IO36 silkscreen | **16** (ADC2_CH5) | ⚠ requires solder jumper from IO36 to IO16 |

**TENSTAR-internal pins** (fixed by the carrier module, not changeable): TFT on 7 / 35 / 36 / 37 / 39 / 40 / 45, I²C on 41 / 42, I²C power enable on 21, NeoPixel on 33.

### Button ladder

Each of the two ADC pins decodes **3 buttons** via a resistor ladder (pull-up 10 kΩ to 3.3 V, each button to GND through a series R):

| Button | Series R | Pin voltage | ADC raw (12-bit, 11 dB attenuation) |
|--------|----------|-------------|-------------------------------------|
| Button 1 / 4 | 0 Ω | 0.00 V | ~ 0 |
| Button 2 / 5 | 4.7 kΩ | 1.05 V | ~ 1300 |
| Button 3 / 6 | 15 kΩ | 1.98 V | ~ 2450 |
| released | — | 3.30 V (clamped) | ~ 4095 |

Simultaneous presses on the same ladder cannot be distinguished. Across the two ladders they work fine.

### Battery voltage sensing (hardware workaround)

The AIO32 v2.1 silk-screen labels the divider sense node as `IO36`. GPIO 36 on ESP32-S3 is **not ADC-capable** (ADC1 = GPIO 1–10, ADC2 = GPIO 11–20). On this board revision a **solder jumper from IO36 to IO16** moves the sense line onto `ADC2_CH5`. The divider itself is 100 kΩ / 22 kΩ (factor 5.545), giving ~3.03 V at 4S full.

`BATTERY_MONITOR_ENABLED` is `false` by default. Flip it in `config.h` once the jumper is populated and the reading is verified.

---

## iBus channel map (identical to Mega sketch v3.4)

| CH | Function | FlySky default control |
|----|----------|------------------------|
| 1  | Drive 1 (steering / motor 1) | Right Stick L/R |
| 2  | Drive 2 (throttle / motor 2) | Right Stick U/D |
| 3  | Mainbar servo | **must be assigned manually** (VrA or switch) |
| 4  | Head pitch | Left Stick U/D |
| 5  | Head yaw | Left Stick L/R |
| 6  | Head roll | VrB |
| 7  | Sound Mute (2-pos) | SwA |
| 8  | Sound Mode / Greet (2-pos) | SwB |
| 9  | Sound Mood (3-pos neg/mid/pos) | SwC |
| 10 | Sound Squeak (2-pos) | free — assign to any unused control |

The SD-card sound-file layout (`/mp3/0001.mp3 … 0032.mp3`) is also identical to the Mega line.

---

## What is not yet parity with the Mega v3.4 sketch

This AIO32 firmware baseline intentionally matches the Mega wiring and user-facing behaviour, but several higher-level features are still unique / missing:

- **IMU:** currently reads the QMI8658C only (on the TENSTAR carrier). LSM6DS3 primary support is the next milestone.
- **RC shaping:** deadband + expo curve like in v3.4 not yet ported. Inputs are passed almost directly into the PID mix.
- **Motor ramping / adaptive PID / dynamic lean angle:** not yet ported.
- **Idle animations + state reactions:** not yet implemented.
- **Configuration menu:** the Mega-style serial `m` menu is not implemented on the ESP32-S3 side. Settings live in `Preferences` (NVS) without a menu UI yet.
- **Battery monitor:** pin wired through solder jumper, feature is `OFF` by default until verified.

See the root [`docs/ROADMAP.md`](../docs/ROADMAP.md) for tracking.

---

## Safety

LiPo handling, motor currents, first power-on with wheels off the ground — the Mega documentation applies 1:1 because it is the same droid. Read the safety chapter of the Mega user handbook before powering this board for the first time.

---

## Author

Printed-Droid.com · Community: [Printed Droid Facebook group](https://www.facebook.com/groups/printeddroid/)
