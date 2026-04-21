# D-O Nano Sound Controller v2

![Version](https://img.shields.io/badge/version-2.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Nano-green.svg)
![Status](https://img.shields.io/badge/status-legacy--companion-yellow.svg)

> **⚠ Legacy-Begleitsketch.** Dieser Sketch läuft auf einem **externen Arduino Nano** und steuert ausschließlich den DFPlayer. Er ist nur nötig, wenn auf einem V1.6-Board der Nano bereits am DFPlayer verdrahtet ist und zusammen mit [`D-O_ibus_v2.1`](../D-O_ibus_v2.1/) (`setup_type = 0`) betrieben wird.
>
> **Für Neu-Aufbauten nicht nötig** — [`D-O_ibus_v3.4`](../D-O_ibus_v3.4/) betreibt den DFPlayer direkt am Mega, auch im PWM-Mode.
>
> Sketch-Übersicht und Empfehlungs-Matrix: siehe [`../README.md`](../README.md).

---

## Zweck

Der Nano liest 4 PWM-Kanäle vom RC-Empfänger parallel zum Mega und spielt die passenden Sounds über den DFPlayer Mini ab. Es gibt keinen direkten Datenlink zum Mega — beide hören auf denselben Empfänger.

## Features (Rewrite 2025-06)

- **Non-blocking State-Machine** — pro Loop-Iteration nur ein RC-Kanal (kein blockierendes `pulseIn` × 4)
- **Hysterese** auf PWM-Werten → kein Jitter
- **Edge-Detection** — Sound nur bei Stellungswechsel, kein Retrigger-Spam
- **Mute-Funktion** auf CH7 (FlySky)
- **Sound-Queue** mit `MIN_SOUND_INTERVAL = 400 ms`
- **Startup-Delay** 2 s — keine Inputs während Boot-Sound
- **`#define DEBUG_MODE`** — Debug-Prints per Conditional Compilation wegkompilierbar

## Hardware

- Arduino Nano (oder kompatibel, ATmega328P)
- DFPlayer Mini + MicroSD (FAT16/32)
- RC-Empfänger (PWM-Ausgänge)
- Lautsprecher 8 Ω, < 3 W
- 1 kΩ Widerstand auf TX-Leitung

## Pinbelegung

| Board-Label (V1.6) | FlySky-CH | Nano-Pin | Funktion |
|:------------------:|:---------:|:--------:|----------|
| 7 | CH7 | D11 | Sound Mute |
| 8 | CH8 | D10 | Sound Mode (Greet/Default) |
| 9 | CH9 | D12 | Sound Mood (neg/mid/pos, 3-Pos) |
| 10 | CH10 | D9 | Sound Squeak |
| — | — | D0 (RX0) | DFPlayer TX |
| — | — | D1 (TX1) + 1 kΩ | DFPlayer RX |
| — | — | 5V / GND | DFPlayer Versorgung |

## Sound-File-Mapping

Dateien `NNNN.mp3` im `/mp3/`-Ordner der SD:

| Track | Funktion |
|-------|----------|
| 0001 | Startup ("battery charged") |
| 0002 | Default ("I am D-O") |
| 0003–0005 | Greetings |
| 0006–0009 | Negative |
| 0010–0014 | Positive |
| 0015–0020 | Squeaky Wheel |

Tracks 0021+ werden **nur** vom Mega-Sketch (v2.1 / v3.4) abgespielt, nicht vom Nano.

## DFPlayer-Routing auf V1.6 (Solder-Jumper)

Das V1.6-Board hat einen Solder-Jumper, der die DFPlayer-Signalleitungen umschaltet:
- **Default (Legacy-Setup):** Nano D0/D1 → DFPlayer
- **Umgejumpert (v3.4-Setup):** Mega D7/D8 → DFPlayer — **Nano wird dann nicht mehr gebraucht**

Das V1.7-Board hat **keinen Nano-Slot** — DFPlayer hängt immer am Mega.

## Historie

Ersetzt den ursprünglichen `D-O_Nano_Sketch` (2016, DFRobot-Beispiel-Basis, 203 Zeilen, inzwischen gelöscht), der blockierendes `pulseIn` × 4 pro Loop machte.
