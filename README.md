# D-O Control & Power Board

Arduino-basiertes Control- & Power-System für den selbstbalancierenden D-O-Droiden aus *Star Wars: Rise of Skywalker*. Unterstützt zwei aktive Board-Generationen: **V1.6** (Standard Control PCB, PWM + optionaler Nano für Sound) und **V1.7** (Mini iBus Board, iBus-only, Sound auf Mega integriert).

**Aktueller Sketch:** `D-O_ibus_v3.4` (April 2026, Bug-Fix-Release)

---

## Features

- MPU6050-basierte Selbstbalance mit PID-Regelung (klon-kompatibel: MPU6050/6500/9250/6886)
- FlySky iBus-Empfang (10 Kanäle) oder klassisches PWM
- 4-fach Servo-Ansteuerung (Mainbar + 3× Kopf)
- DFPlayer Mini Sound-System mit Persönlichkeit (Greetings, Stimmungen, Idle-Animationen, Tilt-Warnung, Low-Battery)
- Cytron MDD10A Dual-Motortreiber (10 A je Kanal)
- Interaktives Serial-CLI mit EEPROM-Konfiguration (PID-Tuning, Feature-Toggles, Kalibrierung)
- Battery-Monitoring (2× 2S LiPo in Reihe = 4S, 14.8 V nominal) mit Low-Voltage-Schutz
- Motor-Ramping, Adaptive PID, Dynamic Target Angle

---

## Board-Varianten

| Board | Empfänger | Bester Sketch | Warum |
|-------|-----------|---------------|-------|
| **V1.7** (Mini iBus) | iBus | `D-O_ibus_v3.4` Mode 1 | V1.7 hat nur iBus-Eingang und keinen Nano-Slot. v3.4 ist der aktuelle Stand. |
| **V1.6** (Standard PCB) | iBus | `D-O_ibus_v3.4` Mode 1 | Sauberste Verkabelung, DFPlayer am Mega, alle Features. |
| **V1.6** (Standard PCB) | klassisch PWM | `D-O_ibus_v3.4` Mode 0 | PWM-Empfänger bleibt, DFPlayer läuft trotzdem am Mega — kein Nano nötig. |
| **V1.6** (bestehendes Nano-Setup) | PWM + externer Nano | `D-O_ibus_v2.1` + `D_O_Nano_Sketch_v2` | Nur für Aufbauten, bei denen der Nano bereits am DFPlayer hängt. |

Ältere Boards (v1.4 Control PCB, v1.5 Mini iBus) sind **pin-kompatibel** — gleiche Sketch-Wahl wie das jeweilige Nachfolge-Board.

---

## Sketch-Inventar

Vollständiger Überblick aller Sketche im Repository. Einzelne Sketch-Ordner verweisen hierher zurück.

| Sketch | Plattform | Rolle | Status | Zielgruppe |
|--------|-----------|-------|--------|------------|
| **`D-O_ibus_v3.4`** | Arduino Mega 2560 | Universal-Controller | **Aktuell, empfohlen** | Alle Neu-Aufbauten auf V1.6 / V1.7 |
| `D-O_ibus_v2.1` | Arduino Mega 2560 | Legacy mit externem Nano-Sound | Wartung | Bestehende V1.6-Installationen mit verdrahtetem Nano |
| `D-O_ibus_v1.1` | Arduino Mega 2560 | Ur-iBus-Version (2020-11) | Archiv | Historische Referenz |
| `D-Ov2_Mega2560_sketch` | Arduino Mega 2560 | Ur-PWM-Version (2020) | Archiv | Historische Referenz |
| `D_O_Nano_Sketch_v2` | Arduino Nano | Externer Sound-Co-Controller | Legacy-Begleiter | Zusammen mit v2.1 auf V1.6 |
| `D_O_ESP32_S3` | ESP32-S3 | Alternative MCU-Linie | Experimentell | Außerhalb Scope dieser Doku |

### `D-O_ibus_v3.4` — Universal-Controller (aktuell)

- **Basis:** Rewrite von v2.1 (Dezember 2025, ursprünglich als v3.0–v3.3.6), plus Codex-Review-Merge und Bug-Fix-Runde (April 2026)
- **Setup-Modi:** `0 = PWM Only`, `1 = iBus`. Kein Hybrid mehr.
- **Sound:** DFPlayer immer am Mega (D7/D8). Kein externer Nano benötigt.
- **Features:** RC-Shaping mit Deadband + Expo-Kurve (Input-Clamp gegen Overshoot), Motor-Ramping, Arcade/Tank-Mixing, Idle-Animationen mit Signal-Gate, Battery-Monitoring, Watchdog, IMU-Klon-Support, EEPROM-Magic `0xD043`
- **Pinbelegung:** identisch zu v2.1 / v1.1 (PCB-Kompatibilität)
- **Detail-Doku:** [`D-O_ibus_v3.4/README.md`](D-O_ibus_v3.4/README.md)

### `D-O_ibus_v2.1` — Legacy mit Nano-Option

- **Basis:** Dezember 2025, v2.1.7
- **Setup-Modi:** `0 = PWM Only`, `1 = Hybrid`, `2 = Pure iBus`
- **Sound:** `setup_type = 0` erwartet externen Nano am DFPlayer. Andere Modi betreiben DFPlayer am Mega.
- **Warum noch da:** Installationen, bei denen der Nano bereits verkabelt ist und nicht umgebaut werden soll
- **Bekannte Bugs** (in der Praxis mild, siehe `BUGFIXES.md` #1): RC-Shaping-Overshoot und Idle-nach-Signal-Loss (beide in v3.4 gefixt)
- **Detail-Doku:** [`D-O_ibus_v2.1/README.md`](D-O_ibus_v2.1/README.md)

### `D-O_ibus_v1.1` — Ur-iBus-Version

- **Basis:** 2020-11, Reinhard Stockinger
- **Umfang:** iBus-RC, 4 Servos, DFPlayer am Mega, Balance-PID, optionale Mainbar-Tilt-Korrektur — alles via Feature-Flags konfigurierbar
- **Warum noch da:** Historische Referenz; pinkompatibel, läuft noch auf existierenden Aufbauten
- **Kein CLI, kein EEPROM, kein Battery-Monitor, kein Idle-System**
- **Detail-Doku:** [`D-O_ibus_v1.1/README.md`](D-O_ibus_v1.1/README.md)

### `D-Ov2_Mega2560_sketch` — Ur-PWM-Version

- **Basis:** 2020, ca. 200 Zeilen
- **Umfang:** Reine Balance mit PWM-RC-Drive an D3/D4. Kein iBus, keine Servos, kein Sound.
- **Warum noch da:** Historischer Code-Kern, aus dem v1.1 entstand
- **Empfehlung:** nicht produktiv einsetzen

### `D_O_Nano_Sketch_v2` — Externer Sound-Controller

- **Basis:** 2025-06
- **Plattform:** Arduino Nano (nicht Mega)
- **Rolle:** Liest 4 PWM-Kanäle vom Empfänger parallel zum Mega und steuert den DFPlayer direkt. Kein direkter Datenlink zum Mega.
- **Kanäle (FlySky CH7–10 / Board-Label 7–10):** Mute / Mode / Mood / Squeak
- **Features:** Non-blocking State-Machine, Hysterese, Edge-Detection, Sound-Queue, Mute-Funktion
- **Wird nur gebraucht mit v2.1 `setup_type = 0`.** Mit v3.4 nicht mehr nötig.

### `D_O_ESP32_S3` — Alternative Plattform-Linie

- **Basis:** 2024, v1.1.1, modular aufgeteilt in 8 `.cpp` / `.h`-Module
- **Status:** Experimentell, parallele Entwicklungslinie
- **Nicht pin-kompatibel** mit Mega-Boards
- **Außerhalb Scope** dieses Handbuchs — eigene Doku folgt separat
- **Detail-Doku:** [`D_O_ESP32_S3/README.md`](D_O_ESP32_S3/README.md)

---

## Schnelleinstieg

```bash
# Repository klonen (Repo derzeit privat, Zugang beim Autor anfragen)
git clone https://github.com/PrintedDroid/D-O-Printed-Droid
cd D-O-Printed-Droid

# Sketch in Arduino IDE öffnen (empfohlen: v3.4)
# Datei: D-O_ibus_v3.4/D_O_printed_droid_rc_ibus_v3.4.ino
# Board: Arduino Mega 2560
# Port: passend wählen
# Upload ausführen

# Serial Monitor bei 9600 Baud öffnen
# Innerhalb 3 Sekunden 'c' senden für IMU-Kalibrierung
# 'm' für das Konfigurationsmenü
```

**Upgrade-Hinweis:** Beim ersten Boot auf v3.4 werden alle EEPROM-Einstellungen auf Defaults zurückgesetzt (neue Magic-Nummer `0xD043` verhindert Cross-Version-Korruption). IMU neu kalibrieren mit `c`, Einstellungen via `m`-Menü neu setzen.

---

## Board-Labels und Pinbelegung

Auf V1.6 / V1.7 sind die Anschlüsse **nach FlySky-Kanalnummer** beschriftet, **nicht nach Arduino-Pin**. Häufige Verwechslungsquelle.

| Board-Label | Mega-Pin | FlySky-CH | Funktion |
|:-----------:|:--------:|:---------:|----------|
| **3** | D0 | CH3 | Mainbar-Servo (Stabilisator) — *muss am Sender manuell CH3 zugewiesen werden* |
| **4** | D1 | CH4 | Kopf Pitch (nicken) |
| **5** | D5 | CH5 | Kopf Yaw (drehen) |
| **6** | D6 | CH6 | Kopf Roll (seitlich neigen) |

Sound-Switches am Nano (nur V1.6 Legacy):

| Board-Label | Nano-Pin | FlySky-CH | Funktion |
|:-----------:|:--------:|:---------:|----------|
| 7 | D11 | CH7 | Sound Mute |
| 8 | D10 | CH8 | Sound Mode (Greet/Default) |
| 9 | D12 | CH9 | Sound Mood (neg/mid/pos) |
| 10 | D9 | CH10 | Sound Squeak |

Vollständige Pin-Tabelle inkl. Motoren, IMU, iBus, DFPlayer und Battery-Monitor: siehe [`SYSTEM_OVERVIEW.md`](SYSTEM_OVERVIEW.md) Abschnitt 3.

---

## Dokumentation

- **Handbücher (PDF, DE/EN):** [`generate docs/`](generate%20docs/) — System Documentation + Developer Reference
- **Projekt-Kontext & Regeln:** [`CLAUDE.md`](CLAUDE.md)
- **Technischer Überblick:** [`SYSTEM_OVERVIEW.md`](SYSTEM_OVERVIEW.md)
- **Bestand:** [`docs/SYSTEM_STATUS.md`](docs/SYSTEM_STATUS.md) · **Planung:** [`docs/ROADMAP.md`](docs/ROADMAP.md)
- **Versions-Historie:** [`CHANGELOG.md`](CHANGELOG.md) · **Bug-Journal:** [`BUGFIXES.md`](BUGFIXES.md)

---

## Entwicklungshistorie

```
2020           Mega2560 (PWM-Balance)
  │
2020-11        v1.1 (+ iBus, + Servos, + DFPlayer)
  │
Dez 2025       v2.1.7 (+ EEPROM, + CLI, + Idle, + Battery, + Adaptive PID)
  │
Dez 2025       v3.3.6 (Rewrite, DFPlayer immer am Mega, nur 2 Modi,
  │            Watchdog, IMU-Recovery — aber 3 Fahrdynamik-Features verloren)
  │
April 2026     v3.4.0 (Codex-Review-Merge: Expo-Kurve zurück;
               + Bug-Fixes: RC-Clamp, EEPROM-Magic 0xD043, Idle-Gate)
```

Parallel-Linien:

```
2020     alter Nano-Sound-Sketch (gelöscht)
2025-06  D_O_Nano_Sketch_v2 (Rewrite für v2.1-Legacy-Pfad)
2024     D_O_ESP32_S3 v1.1.1 (ESP32-S3, modulare Architektur, experimentell)
```

---

## Struktur

```
D-O/
├── D-O_ibus_v3.4/                ← aktueller Mega-Sketch (empfohlen)
├── D-O_ibus_v2.1/                ← Legacy-Pfad mit optionalem Nano
├── D-O_ibus_v1.1/                ← Ur-iBus-Version (2020, Archiv)
├── D-Ov2_Mega2560_sketch/        ← Ur-PWM-Version (2020, Archiv)
├── D_O_Nano_Sketch_v2/           ← externer Sound-Controller (Legacy-Begleiter)
├── D_O_ESP32_S3/                 ← parallele Plattform-Linie (experimentell)
├── docs/                         ← Handbuch-Inhalte (Status, Roadmap)
├── generate docs/                ← PDF-Generator (DE/EN) + erzeugte PDFs
├── README.md                     ← diese Datei
├── CLAUDE.md                     ← AI-Kontext + Regeln
├── CHANGELOG.md                  ← Versions-Historie
├── BUGFIXES.md                   ← Bug-Journal
├── NEXT_SESSION_PROMPT.md        ← Session-Handoff
└── SYSTEM_OVERVIEW.md            ← technischer Überblick
```

---

## Autor

Printed-Droid.com

Community: [Facebook-Gruppe Printed Droid](https://www.facebook.com/groups/printeddroid/)
