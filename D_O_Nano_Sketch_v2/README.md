# D-O Nano Sound Controller v2

![Version](https://img.shields.io/badge/version-2.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Nano-green.svg)
![Status](https://img.shields.io/badge/status-legacy--companion-yellow.svg)

> **⚠ Legacy companion sketch.** This sketch runs on an **external Arduino Nano** and drives the DFPlayer only. It is only needed when a V1.6 board already has its Nano wired to the DFPlayer and is running alongside [`D-O_ibus_v2.1`](../D-O_ibus_v2.1/) in `setup_type = 0`.
>
> **Not required for new builds** — [`D-O_ibus_v3.4`](../D-O_ibus_v3.4/) drives the DFPlayer directly from the Mega, even in PWM mode.
>
> Sketch inventory and recommendation matrix: see [`../README.md`](../README.md).

---

## Purpose

The Nano reads 4 PWM channels from the RC receiver in parallel with the Mega and plays the corresponding sounds through the DFPlayer Mini. There is no direct data link to the Mega — both listen to the same receiver.

## Features (rewrite 2025-06)

- **Non-blocking state machine** — one RC channel per loop iteration (instead of blocking `pulseIn` × 4)
- **Hysteresis** on PWM values → no jitter
- **Edge detection** — sound fires only on a position change, no retrigger spam
- **Mute function** on CH7 (FlySky)
- **Sound queue** with `MIN_SOUND_INTERVAL = 400 ms`
- **Startup delay** 2 s — no inputs accepted during the boot sound
- **`#define DEBUG_MODE`** — debug prints are conditionally compiled out

## Hardware

- Arduino Nano (or compatible, ATmega328P)
- DFPlayer Mini + MicroSD (FAT16 / 32)
- RC receiver (PWM outputs)
- Speaker 8 Ω, < 3 W
- 1 kΩ resistor on the TX line

## Pinout

| Board label (V1.6) | FlySky CH | Nano pin | Function |
|:------------------:|:---------:|:--------:|----------|
| 7 | CH7 | D11 | Sound Mute |
| 8 | CH8 | D10 | Sound Mode (Greet / Default) |
| 9 | CH9 | D12 | Sound Mood (neg / mid / pos, 3-pos) |
| 10 | CH10 | D9 | Sound Squeak |
| — | — | D0 (RX0) | DFPlayer TX |
| — | — | D1 (TX1) + 1 kΩ | DFPlayer RX |
| — | — | 5V / GND | DFPlayer power |

## Sound file mapping

Files named `NNNN.mp3` in the SD card `/mp3/` folder:

| Track | Function |
|-------|----------|
| 0001 | Startup ("battery charged") |
| 0002 | Default ("I am D-O") |
| 0003–0005 | Greetings |
| 0006–0009 | Negative |
| 0010–0014 | Positive |
| 0015–0020 | Squeaky wheel |

Tracks 0021+ are only used by the Mega sketch (v2.1 / v3.4), not by the Nano.

## DFPlayer routing on V1.6 (solder jumper)

The V1.6 board has a solder jumper that switches the DFPlayer signal lines:
- **Default (legacy setup):** Nano D0 / D1 → DFPlayer
- **Jumpered (v3.4 setup):** Mega D7 / D8 → DFPlayer — **the Nano is no longer required**

The V1.7 board has **no Nano slot** — the DFPlayer is always connected to the Mega.

## History

Replaces the original `D-O_Nano_Sketch` (2016, DFRobot example-based, 203 lines, now deleted), which did blocking `pulseIn` × 4 per loop.
