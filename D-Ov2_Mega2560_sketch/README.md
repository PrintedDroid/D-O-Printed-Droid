# D-O v2 Mega2560 Sketch (original version)

![Version](https://img.shields.io/badge/version-2020-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![Status](https://img.shields.io/badge/status-archive-lightgrey.svg)

> **⚠ Archive / historical reference.** For new builds use [`D-O_ibus_v3.4`](../D-O_ibus_v3.4/). This is the **plain balance original** from 2020 (~200 lines) — PWM-RC drive on D3 / D4, no iBus, no servos, no sound. It is the code kernel from which v1.1 grew.
>
> Sketch inventory and recommendation matrix: see [`../README.md`](../README.md).

---

## Purpose

Self-balancing D-O droid with an MPU6050 IMU, Cytron motor driver and classic PWM-RC drive input.

## Hardware (minimum)

- Arduino Mega 2560
- MPU6050 (I²C, 0x68)
- Cytron MDD10A (dual-channel H-bridge)
- PWM RC receiver
- 2 × 2S LiPo in series (4S total)

## Pinout

| Mega pin | Function |
|:--------:|----------|
| D13 | Motor 1 direction (DIR1) |
| D12 | Motor 1 speed (PWM1) |
| D11 | Motor 2 direction (DIR2) |
| D10 | Motor 2 speed (PWM2) |
| D3 | Drive CH1 (PWM RC) |
| D4 | Drive CH2 (PWM RC) |
| D20 (SDA) | IMU data |
| D21 (SCL) | IMU clock |

Pin-compatible with v1.1 / v2.1 / v3.4.

## History

Original version. Not maintained, no updates. Kept as a historical reference only.
