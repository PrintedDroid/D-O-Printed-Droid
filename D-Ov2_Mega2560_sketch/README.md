# D-O v2 Mega2560 Sketch (Ur-Version)

![Version](https://img.shields.io/badge/version-2020-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega%202560-green.svg)
![Status](https://img.shields.io/badge/status-archive-lightgrey.svg)

> **⚠ Archiv / historische Referenz.** Für Neu-Aufbauten verwende [`D-O_ibus_v3.4`](../D-O_ibus_v3.4/). Dieser Sketch ist die **reine Balance-Ur-Version** von 2020 (~200 Zeilen) — nur PWM-RC-Drive auf D3/D4, kein iBus, keine Servos, kein Sound. Er ist der Code-Kern, aus dem v1.1 entstand.
>
> Sketch-Übersicht und Empfehlungs-Matrix: siehe [`../README.md`](../README.md).

---

## Zweck

Selbstbalancierender D-O-Droid mit MPU6050 IMU, Cytron-Motortreiber und klassischer PWM-RC-Drive-Eingabe.

## Hardware (Minimum)

- Arduino Mega 2560
- MPU6050 (I²C, 0x68)
- Cytron MD10C × 2 (oder vergleichbar)
- PWM-RC-Empfänger
- 2× 2S LiPo in Serie

## Pinbelegung

| Mega-Pin | Funktion |
|:--------:|----------|
| D13 | Motor 1 Richtung (DIR1) |
| D12 | Motor 1 Speed (PWM1) |
| D11 | Motor 2 Richtung (DIR2) |
| D10 | Motor 2 Speed (PWM2) |
| D3 | Drive CH1 (PWM-RC) |
| D4 | Drive CH2 (PWM-RC) |
| D20 (SDA) | IMU-Daten |
| D21 (SCL) | IMU-Takt |

Pin-kompatibel mit v1.1 / v2.1 / v3.4.

## Historie

Ur-Version. Keine Wartung, keine Updates. Wird nur als historische Referenz aufbewahrt.
