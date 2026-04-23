/**
 * @file button_handler.cpp
 * @brief 6-button ADC-ladder handler for D-O AIO32 v2.1
 * @version 3.0
 * @date 2026-04
 */

#include "button_handler.h"

// Global singleton
ButtonHandler buttonHandler;

// ===========================================================================
// ButtonLadder
// ===========================================================================

ButtonLadder::ButtonLadder()
    : _adcPin(0xFF), _lastRaw(4095), _rawButton(LADDER_NONE),
      _decodedButton(LADDER_NONE), _lastChangeMs(0) {}

void ButtonLadder::begin(uint8_t adcPin) {
    _adcPin = adcPin;
    pinMode(_adcPin, INPUT);       // ADC pin, external pull-up handles rest
    // ESP32-S3 Arduino core performs ADC config on first analogRead()
    _lastRaw = analogRead(_adcPin);
    _rawButton = _decodeRaw(_lastRaw);
    _decodedButton = _rawButton;
    _lastChangeMs = millis();
}

uint8_t ButtonLadder::_decodeRaw(uint16_t raw) const {
    // Windows are intentionally generous; tighten after hardware measurement.
    if (raw >= BTN_ADC_RELEASED_MIN) return LADDER_NONE;
    if (raw <= BTN_ADC_B1_MAX)       return LADDER_BTN_1;
    if (raw >= BTN_ADC_B2_MIN && raw <= BTN_ADC_B2_MAX) return LADDER_BTN_2;
    if (raw >= BTN_ADC_B3_MIN && raw <= BTN_ADC_B3_MAX) return LADDER_BTN_3;
    // Noise / transition zone — keep previous decision
    return _decodedButton;
}

void ButtonLadder::update() {
    if (_adcPin == 0xFF) return;

    _lastRaw = analogRead(_adcPin);
    uint8_t decoded = _decodeRaw(_lastRaw);

    uint32_t now = millis();
    if (decoded != _rawButton) {
        _rawButton = decoded;
        _lastChangeMs = now;
    }

    // Promote raw -> decoded after stable debounce period
    if (decoded != _decodedButton && (now - _lastChangeMs) >= BUTTON_DEBOUNCE_DELAY) {
        _decodedButton = decoded;
    }
}

// ===========================================================================
// ButtonHandler
// ===========================================================================

ButtonHandler::ButtonHandler() : _debug(false) {
    for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
        _btn[i] = {false, false, 0, 0, 0, BUTTON_NONE};
    }
    for (uint8_t i = 0; i < NUM_BUTTONS * NUM_BUTTONS; ++i) {
        _comboStart[i] = 0;
    }
}

bool ButtonHandler::begin() {
    _ladder[0].begin(BUTTON_LADDER1_PIN);  // Buttons 1, 2, 3
    _ladder[1].begin(BUTTON_LADDER2_PIN);  // Buttons 4, 5, 6
    return true;
}

void ButtonHandler::_updateButton(uint8_t btnIndex, bool rawPressed) {
    if (btnIndex >= NUM_BUTTONS) return;
    ButtonState &b = _btn[btnIndex];
    uint32_t now = millis();

    // --- Press edge ---
    if (rawPressed && !b.pressed) {
        b.pressed = true;
        b.pressTime = now;
        b.longPressHandled = false;
    }

    // --- Hold: emit LONG_PRESS once, then periodic HOLD ---
    if (b.pressed && !b.longPressHandled) {
        if (now - b.pressTime >= BUTTON_LONG_PRESS_TIME) {
            b.lastEvent = BUTTON_LONG_PRESS;
            b.longPressHandled = true;
        }
    } else if (b.pressed && b.longPressHandled) {
        if (now - b.pressTime >= BUTTON_LONG_PRESS_TIME + BUTTON_HOLD_REPEAT_TIME) {
            b.lastEvent = BUTTON_HOLD;
            b.pressTime = now - BUTTON_LONG_PRESS_TIME; // reset hold cadence
        }
    }

    // --- Release edge ---
    if (!rawPressed && b.pressed) {
        b.pressed = false;
        uint32_t held = now - b.pressTime;
        if (!b.longPressHandled && held >= BUTTON_DEBOUNCE_DELAY) {
            // Potential short press — defer to check for double-click
            if (b.clickCount == 0) {
                b.clickCount = 1;
                b.releaseTime = now;
            } else if ((now - b.releaseTime) <= BUTTON_DOUBLE_CLICK_TIME) {
                b.clickCount = 0;
                b.lastEvent = BUTTON_DOUBLE_CLICK;
            } else {
                // Previous single click expired; start new single count
                b.clickCount = 1;
                b.releaseTime = now;
            }
        }
    }

    // --- Commit pending single-click if double-click window expired ---
    if (!b.pressed && b.clickCount == 1 &&
        (now - b.releaseTime) > BUTTON_DOUBLE_CLICK_TIME) {
        b.clickCount = 0;
        if (b.lastEvent == BUTTON_NONE) {
            b.lastEvent = BUTTON_SHORT_PRESS;
        }
    }
}

void ButtonHandler::update() {
    _ladder[0].update();
    _ladder[1].update();

    // Ladder 1 (IO5) -> Buttons 1, 2, 3 (array indices 0..2)
    uint8_t l1 = _ladder[0].currentButton();
    _updateButton(0, l1 == ButtonLadder::LADDER_BTN_1);
    _updateButton(1, l1 == ButtonLadder::LADDER_BTN_2);
    _updateButton(2, l1 == ButtonLadder::LADDER_BTN_3);

    // Ladder 2 (IO12) -> Buttons 4, 5, 6 (array indices 3..5)
    uint8_t l2 = _ladder[1].currentButton();
    _updateButton(3, l2 == ButtonLadder::LADDER_BTN_1);
    _updateButton(4, l2 == ButtonLadder::LADDER_BTN_2);
    _updateButton(5, l2 == ButtonLadder::LADDER_BTN_3);

    if (_debug) {
        static uint32_t lastPrint = 0;
        uint32_t now = millis();
        if (now - lastPrint > 500) {
            lastPrint = now;
            DEBUG_PRINTF("[btn] L1 raw=%u dec=%u  L2 raw=%u dec=%u  states: ",
                         _ladder[0].lastRaw(), (unsigned)l1,
                         _ladder[1].lastRaw(), (unsigned)l2);
            for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
                DEBUG_PRINTF("%u%c ", (unsigned)(i + 1), _btn[i].pressed ? '*' : '.');
            }
            DEBUG_PRINTLN("");
        }
    }
}

ButtonEvent ButtonHandler::getButtonEvent(uint8_t n) {
    if (n < 1 || n > NUM_BUTTONS) return BUTTON_NONE;
    ButtonEvent ev = _btn[n - 1].lastEvent;
    _btn[n - 1].lastEvent = BUTTON_NONE;
    return ev;
}

bool ButtonHandler::isButtonPressed(uint8_t n) {
    if (n < 1 || n > NUM_BUTTONS) return false;
    return _btn[n - 1].pressed;
}

bool ButtonHandler::isComboPressed(uint8_t a, uint8_t b) {
    return isButtonPressed(a) && isButtonPressed(b);
}

uint32_t ButtonHandler::getComboDuration(uint8_t a, uint8_t b) {
    if (a < 1 || a > NUM_BUTTONS || b < 1 || b > NUM_BUTTONS) return 0;
    uint8_t i = (a - 1) * NUM_BUTTONS + (b - 1);
    uint32_t now = millis();
    bool pressed = isComboPressed(a, b);
    if (pressed) {
        if (_comboStart[i] == 0) _comboStart[i] = now;
        return now - _comboStart[i];
    } else {
        _comboStart[i] = 0;
        return 0;
    }
}

uint16_t ButtonHandler::getLadderRaw(uint8_t ladderIdx) const {
    if (ladderIdx > 1) return 0;
    return _ladder[ladderIdx].lastRaw();
}

void ButtonHandler::printButtonStates() {
    DEBUG_PRINTF("[btn] L1 raw=%u (%s)  L2 raw=%u (%s)\n",
                 _ladder[0].lastRaw(),
                 _ladder[0].currentButton() ? "pressed" : "idle",
                 _ladder[1].lastRaw(),
                 _ladder[1].currentButton() ? "pressed" : "idle");
    for (uint8_t i = 0; i < NUM_BUTTONS; ++i) {
        DEBUG_PRINTF("  Button %u: %s\n", (unsigned)(i + 1),
                     _btn[i].pressed ? "PRESSED" : "released");
    }
}
