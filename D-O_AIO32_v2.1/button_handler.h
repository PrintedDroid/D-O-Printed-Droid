/**
 * @file button_handler.h
 * @brief Button handler for D-O AIO32 v2.1 — 6 buttons on 2 ADC ladders
 * @version 3.0
 * @date 2026-04
 *
 * @details AIO32 v2.1 wires six tact buttons to two ESP32-S3 ADC-capable pins
 *          through resistor ladders:
 *
 *            IO5  (ADC1_CH4) — Buttons 1, 2, 3
 *            IO12 (ADC2_CH1) — Buttons 4, 5, 6
 *
 *            Each ladder: 10 kOhm pull-up to 3.3 V, per-button series R to GND:
 *              Button 1/4: 0 Ohm    -> ADC ~0
 *              Button 2/5: 4.7 kOhm -> ADC ~1300 (12-bit @ 11 dB att.)
 *              Button 3/6: 15  kOhm -> ADC ~2450
 *              released  : pulled   -> ADC ~4095
 *
 *          The class decodes which single button is down, debounces, and emits
 *          short-press / long-press / double-click / hold events per button.
 *          Simultaneous presses on the same ladder cannot be distinguished
 *          electrically — the handler reports only the highest-priority button
 *          (lowest R wins). Simultaneous presses across the two ladders do work.
 */

#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>
#include "config.h"

// Button timing constants (overridable via config.h)
#ifndef BUTTON_DOUBLE_CLICK_TIME
#define BUTTON_DOUBLE_CLICK_TIME 300
#endif
#ifndef BUTTON_HOLD_REPEAT_TIME
#define BUTTON_HOLD_REPEAT_TIME 500
#endif
#ifndef BUTTON_COMBO_TIMEOUT
#define BUTTON_COMBO_TIMEOUT 100
#endif

// ============================================================================
// EVENT ENUM
// ============================================================================

enum ButtonEvent {
    BUTTON_NONE,
    BUTTON_SHORT_PRESS,
    BUTTON_LONG_PRESS,
    BUTTON_DOUBLE_CLICK,
    BUTTON_HOLD
};

// ============================================================================
// INTERNAL: SINGLE BUTTON STATE (one entry per physical button)
// ============================================================================

struct ButtonState {
    bool pressed;                ///< Debounced "currently pressed" flag
    bool longPressHandled;       ///< Long-press fired flag (to prevent repeats)
    uint32_t pressTime;          ///< millis() at press start
    uint32_t releaseTime;        ///< millis() at last release
    uint8_t clickCount;          ///< Click counter for double-click detection
    ButtonEvent lastEvent;       ///< Event waiting to be collected by consumer
};

// ============================================================================
// ADC LADDER — decodes 3 buttons on 1 pin
// ============================================================================

class ButtonLadder {
public:
    // Decoded button index: 0 = nothing pressed, 1..3 = button 1/2/3 of ladder
    enum { LADDER_NONE = 0, LADDER_BTN_1 = 1, LADDER_BTN_2 = 2, LADDER_BTN_3 = 3 };

    ButtonLadder();
    void begin(uint8_t adcPin);

    /** @brief Sample ADC and update internal debounced state. Call often. */
    void update();

    /** @brief Which button of this ladder is currently decoded (0..3). */
    uint8_t currentButton() const { return _decodedButton; }

    /** @brief Raw last ADC sample (useful for calibration debug). */
    uint16_t lastRaw() const { return _lastRaw; }

private:
    uint8_t _adcPin;
    uint16_t _lastRaw;
    uint8_t _rawButton;           ///< Undebounced decode from the ADC value
    uint8_t _decodedButton;       ///< Debounced button index (0..3)
    uint32_t _lastChangeMs;       ///< millis() at last _rawButton change
    uint8_t  _decodeRaw(uint16_t raw) const;
};

// ============================================================================
// BUTTON HANDLER — exposes 6 logical buttons with event detection
// ============================================================================

class ButtonHandler {
public:
    static constexpr uint8_t NUM_BUTTONS = 6;

    ButtonHandler();

    bool begin();
    void update();              ///< Call from main loop

    // --- Per-button event API (1..6) ---
    ButtonEvent getButtonEvent(uint8_t n);  ///< n = 1..6; clears after read
    bool        isButtonPressed(uint8_t n); ///< n = 1..6

    // --- Legacy convenience wrappers (1..4) to keep older call-sites working
    ButtonEvent getButton1Event() { return getButtonEvent(1); }
    ButtonEvent getButton2Event() { return getButtonEvent(2); }
    ButtonEvent getButton3Event() { return getButtonEvent(3); }
    ButtonEvent getButton4Event() { return getButtonEvent(4); }
    bool        isButton1Pressed() { return isButtonPressed(1); }
    bool        isButton2Pressed() { return isButtonPressed(2); }
    bool        isButton3Pressed() { return isButtonPressed(3); }
    bool        isButton4Pressed() { return isButtonPressed(4); }

    // --- Combo detection (simple: both buttons down at the same time) ---
    /** @param a, b are 1-based button indices. Returns true while both are pressed. */
    bool isComboPressed(uint8_t a, uint8_t b);
    /** @brief Duration (ms) the given combo has been held; 0 if not currently held. */
    uint32_t getComboDuration(uint8_t a, uint8_t b);

    // --- Debugging ---
    void setDebugEnabled(bool enable) { _debug = enable; }
    void printButtonStates();
    uint16_t getLadderRaw(uint8_t ladderIdx) const; ///< 0 for IO5, 1 for IO12

private:
    ButtonLadder _ladder[2];           ///< [0] = IO5, [1] = IO12
    ButtonState  _btn[NUM_BUTTONS];    ///< _btn[0] = Button 1, ..., _btn[5] = Button 6
    uint32_t     _comboStart[NUM_BUTTONS * NUM_BUTTONS]; ///< flat matrix, indexed by a*NB+b
    bool         _debug;

    void _updateButton(uint8_t btnIndex, bool rawPressed);
};

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

extern ButtonHandler buttonHandler;

#endif // BUTTON_HANDLER_H
