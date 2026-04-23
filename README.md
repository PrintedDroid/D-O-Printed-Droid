# D-O Controller Family (Printed-Droid.com)

Control & power electronics for the self-balancing **D-O droid** from *Star Wars: The Rise of Skywalker*. Two parallel product lines, both driving the same physical droid (same chassis, motors, servos, sound files, transmitter mapping) — pick whichever suits your build:

| Line | Controller | Board revisions | Current firmware | Folder |
|------|-----------|-----------------|------------------|--------|
| **AIO Mega** | Arduino Mega 2560 + Cytron MDD10A | V1.6 Standard PCB, V1.7 Mini iBus | `D-O_ibus_v3.4/` — v3.4.1 (point release) | this folder (`D-O/`) |
| **AIO32 v2** | ESP32-S3 (TENSTAR module) + Cytron MDD10A | AIO32 v2 | `D-O_AIO32_v2.1/` — v2.1.1 | [`../D-O_AIO32/`](../D-O_AIO32/) |

Both controllers use the **same FlySky iBus / Futaba SBUS channel map, the same DFPlayer SD-card layout, and the same 4S LiPo supply** — you can swap boards in a finished droid and reuse your transmitter, sound files and mechanics without reconfiguration.

See [`../D-O_AIO32/README.md`](../D-O_AIO32/README.md) for the ESP32-S3 line's specifics (dual IMU, colour TFT, 6-button UI, CLI, runtime SBUS switching).

---

## Mega AIO line (this folder)

Arduino-based control and power system for the self-balancing D-O droid. Supports two actively maintained board generations: **V1.6** (Standard Control PCB, PWM + optional Nano for sound) and **V1.7** (Mini iBus Board, iBus-only, sound integrated on the Mega).

**Current sketch:** `D-O_ibus_v3.4/` — v3.4.1 (April 2026 point release: SBUS support, Madgwick + PIDController + filename-based DFPlayer backports from the AIO32 line, `max_acceleration` field removed)

---

## Features

- MPU6050-based self-balancing with PID control (clone-compatible: MPU6050 / MPU6500 / MPU9250 / MPU6886)
- FlySky iBus reception (10 channels) or classic PWM
- 4-servo control (mainbar + 3 × head)
- DFPlayer Mini sound system with personality (greetings, moods, idle animations, tilt warning, low-battery)
- Cytron MDD10A dual-channel motor driver (10 A per channel)
- Interactive serial CLI with EEPROM-backed configuration (PID tuning, feature toggles, calibration)
- Battery monitoring (2 × 2S LiPo in series = 4S, 14.8 V nominal) with low-voltage protection
- Motor ramping, adaptive PID, dynamic target angle

---

## Board variants

| Board | Receiver | Best sketch | Why |
|-------|----------|-------------|-----|
| **V1.7** (Mini iBus) | iBus or SBUS | `D-O_ibus_v3.4` Mode 1 | V1.7 has a serial RC input and no Nano slot. v3.4.1 adds SBUS option (needs external inverter). |
| **V1.6** (Standard PCB) | iBus or SBUS | `D-O_ibus_v3.4` Mode 1 | Cleanest wiring, DFPlayer runs on the Mega, all features available. SBUS optional. |
| **V1.6** (Standard PCB) | classic PWM | `D-O_ibus_v3.4` Mode 0 | PWM receiver stays, DFPlayer still runs on the Mega — no external Nano required. |
| **V1.6** (existing Nano setup) | PWM + external Nano | `D-O_ibus_v2.1` + `D_O_Nano_Sketch_v2` | Only for builds where the Nano is already wired to the DFPlayer. |

Older boards (v1.4 Control PCB, v1.5 Mini iBus) are **pin-compatible** — same sketch choice as the respective successor board.

---

## Sketch inventory

Full overview of the sketches in this repository. The individual sketch folders link back to this table.

| Sketch | Platform | Role | Status | Target user |
|--------|----------|------|--------|-------------|
| **`D-O_ibus_v3.4`** (v3.4.1) | Arduino Mega 2560 | Universal controller | **Current, recommended** | Any new build on V1.6 / V1.7 |
| `D-O_ibus_v2.1` | Arduino Mega 2560 | Legacy controller with external Nano sound | Maintenance | Existing V1.6 installs with a wired-up Nano |
| `D-O_ibus_v1.1` | Arduino Mega 2560 | Original iBus version (2020-11) | Archive | Historical reference |
| `D-Ov2_Mega2560_sketch` | Arduino Mega 2560 | Original PWM version (2020) | Archive | Historical reference |
| `D_O_Nano_Sketch_v2` | Arduino Nano | External sound co-controller | Legacy companion | Paired with v2.1 on V1.6 |
| [`../D-O_AIO32/D-O_AIO32_v2.1`](../D-O_AIO32/D-O_AIO32_v2.1) | ESP32-S3 (TENSTAR) | AIO32 v2 board firmware | **Current, recommended for AIO32 v2** | Any build on the AIO32 v2 controller |

### `D-O_ibus_v3.4` — universal controller (current)

- **Base:** Rewrite of v2.1 (December 2025), consolidated and bug-fixed in April 2026.
- **Setup modes:** `0 = PWM Only`, `1 = iBus`. No more Hybrid mode.
- **Sound:** DFPlayer always driven by the Mega (D7 / D8). No external Nano required.
- **Features (v3.4.1):** RC shaping with deadband + expo curve (input clamped against overshoot), motor ramping, arcade / tank mixing, idle animations with signal gating, battery monitoring, watchdog, IMU clone support, **SBUS support (with external inverter)**, filename-based DFPlayer addressing, optional Madgwick AHRS / class-style PID, runtime Madgwick beta tuning, EEPROM magic `0xD044`.
- **Pinout:** identical to v2.1 / v1.1 (PCB compatibility preserved).
- **Details:** [`D-O_ibus_v3.4/README.md`](D-O_ibus_v3.4/README.md)

### `D-O_ibus_v2.1` — legacy with optional Nano

- **Base:** December 2025, v2.1.7.
- **Setup modes:** `0 = PWM Only`, `1 = Hybrid`, `2 = Pure iBus`.
- **Sound:** `setup_type = 0` expects an external Nano on the DFPlayer. Other modes drive the DFPlayer on the Mega directly.
- **Why it still exists:** installs where the Nano is already wired and the user does not want to rework it.
- **Details:** [`D-O_ibus_v2.1/README.md`](D-O_ibus_v2.1/README.md)

### `D-O_ibus_v1.1` — original iBus version

- **Base:** November 2020, Reinhard Stockinger.
- **Scope:** iBus RC, 4 servos, DFPlayer on the Mega, balance PID, optional mainbar tilt correction — everything gated by `#define` feature flags.
- **Why it still exists:** historical reference; pin-compatible and still runs on existing builds.
- **No CLI, no EEPROM, no battery monitor, no idle system.**
- **Details:** [`D-O_ibus_v1.1/README.md`](D-O_ibus_v1.1/README.md)

### `D-Ov2_Mega2560_sketch` — original PWM version

- **Base:** 2020, roughly 200 lines.
- **Scope:** Plain balance with PWM-RC drive on D3 / D4. No iBus, no servos, no sound.
- **Why it still exists:** the historical code kernel from which v1.1 grew.
- **Recommendation:** not for productive use.

### `../D-O_AIO32/D-O_AIO32_v2.1` — AIO32 v2 firmware (ESP32-S3)

- **Base:** 2026-04-23, v2.1.1 (point release: SBUS via CLI, Madgwick beta tuning, filename-based DFPlayer).
- **Platform:** ESP32-S3 on TENSTAR TS-ESP32-S3 module (4 MB flash, native USB-C).
- **Hardware differences vs Mega line:** onboard 1.14" ST7789 colour TFT, dual IMU with Madgwick fusion (QMI8658C + LSM6DS3TR-C), WS2812 status LED with state-based patterns, 6 on-board buttons (B1..B6 Start/Stop / Calibrate / Display / Debug / Mode / Back-Emergency).
- **RC protocols:** FlySky iBus (default) AND Futaba/FrSky SBUS — both **runtime-switchable via CLI** with NVS persistence. No external inverter needed for SBUS (ESP32 UART hardware invert).
- **Sound:** Cytron MDD10A motor driver, DFPlayer Mini + Makuna DFMiniMp3 library with filename-based `playMp3FolderTrack()` — SD-card copy order doesn't matter.
- **CLI:** full interactive CLI on USB-serial at 115200 baud (`help`, `pid kp ...`, `imu mode ...`, `rc protocol sbus`, `debug stream ...`, etc.), every parameter runtime-tuneable and NVS-persisted.
- **Details:** [`../D-O_AIO32/README.md`](../D-O_AIO32/README.md) · user handbook PDF (EN + DE) in the AIO32 root.

### `D_O_Nano_Sketch_v2` — external sound controller

- **Base:** June 2025.
- **Platform:** Arduino Nano (not Mega).
- **Role:** reads 4 PWM channels from the receiver in parallel to the Mega and drives the DFPlayer directly. No direct data link to the Mega.
- **Channels (FlySky CH7–CH10 / board labels 7–10):** Mute / Mode / Mood / Squeak.
- **Features:** non-blocking state machine, hysteresis, edge detection, sound queue, mute function.
- **Only needed with v2.1 `setup_type = 0`.** Not needed with v3.4.

---

## Board labels and pinout

On V1.6 / V1.7 the connectors are labelled by **FlySky channel number**, **not by Arduino pin**. This is a common source of confusion.

| Board label | Mega pin | FlySky CH | Function |
|:-----------:|:--------:|:---------:|----------|
| **3** | D0 | CH3 | Mainbar servo (stabiliser bar) — *CH3 must be manually assigned on the transmitter* |
| **4** | D1 | CH4 | Head pitch |
| **5** | D5 | CH5 | Head yaw |
| **6** | D6 | CH6 | Head roll |

Sound switches on the Nano (legacy V1.6 only):

| Board label | Nano pin | FlySky CH | Function |
|:-----------:|:--------:|:---------:|----------|
| 7 | D11 | CH7 | Sound Mute |
| 8 | D10 | CH8 | Sound Mode (Greet / Default) |
| 9 | D12 | CH9 | Sound Mood (neg / mid / pos) |
| 10 | D9 | CH10 | Sound Squeak |

---

## Manuals

Full user manuals (EN / DE) as PDF:

**Mega AIO line** — in this folder:
- [D-O_Control_&_Power_Board_System_Documentation_v2.2.pdf](D-O_Control_&_Power_Board_System_Documentation_v2.2.pdf) — user handbook (EN)
- [D-O_Control_&_Power_Board_System_Documentation_v2.2_DE.pdf](D-O_Control_&_Power_Board_System_Documentation_v2.2_DE.pdf) — Nutzerhandbuch (DE)
- [D-O_Control_&_Power_Board_Developer_Reference_v2.2.pdf](D-O_Control_&_Power_Board_Developer_Reference_v2.2.pdf) — developer reference (internal)
- [D-O_Control_&_Power_Board_Developer_Reference_v2.2_DE.pdf](D-O_Control_&_Power_Board_Developer_Reference_v2.2_DE.pdf) — Developer Reference (intern)

**AIO32 v2 line** — in [`../D-O_AIO32/`](../D-O_AIO32/):
- [D-O_AIO32_v2_User_Handbook_v2.1.1.pdf](../D-O_AIO32/D-O_AIO32_v2_User_Handbook_v2.1.1.pdf) — user handbook (EN)
- [D-O_AIO32_v2_User_Handbook_v2.1.1_DE.pdf](../D-O_AIO32/D-O_AIO32_v2_User_Handbook_v2.1.1_DE.pdf) — Nutzerhandbuch (DE)

---

## Author

Printed-Droid.com

Community: [Printed Droid Facebook group](https://www.facebook.com/groups/printeddroid/)
