/**
 * @file button_handler.cpp
 * @brief Button Handler Implementation - CORRECTED VERSION
 * @version 2.1
 * @date 2024
 * 
 * @details Complete implementation with static memory allocation
 *          and all timing constants moved to config
 */

#include "button_handler.h"

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

/// Global button handler instance
ButtonHandler buttonHandler;

// ============================================================================
// STATIC VARIABLES
// ============================================================================

/// Debug output enable flag
static bool debugEnabled = DEBUG_MODE;

// ============================================================================
// BUTTONEX IMPLEMENTATION
// ============================================================================

/**
 * @brief Constructor for ButtonEx
 * @param buttonPin GPIO pin number
 * @param activelow True if button pulls to GND when pressed
 */
ButtonEx::ButtonEx(uint8_t buttonPin, bool activelow) : 
    pin(buttonPin), 
    inverted(activelow),
    currentState(false),
    lastState(false),
    pressed(false),
    longPressHandled(false),
    pressTime(0),
    releaseTime(0),
    lastDebounce(0),
    clickCount(0),
    lastEvent(BUTTON_NONE) {
}

/**
 * @brief Initialize button hardware
 */
void ButtonEx::begin() {
    if (pin > 0) {  // Only configure if pin is set
        // Configure pin with appropriate pull resistor
        pinMode(pin, inverted ? INPUT_PULLUP : INPUT_PULLDOWN);
        
        // Read initial state
        lastState = readButton();
        currentState = lastState;
        
        if (debugEnabled) {
            DEBUG_PRINTF("ButtonEx: Pin %d initialized (inverted: %s, initial: %s)\n", 
                         pin, 
                         inverted ? "true" : "false",
                         currentState ? "pressed" : "released");
        }
    }
}

/**
 * @brief Read raw button state with inversion
 * @return True if button is pressed (after inversion)
 */
bool ButtonEx::readButton() {
    if (pin == 0) return false;
    bool state = digitalRead(pin);
    return inverted ? !state : state;
}

/**
 * @brief Update button state and detect events
 * @return Detected event or BUTTON_NONE
 */
ButtonEvent ButtonEx::update() {
    if (pin == 0) return BUTTON_NONE;
    
    bool state = readButton();
    uint32_t now = millis();
    ButtonEvent event = BUTTON_NONE;
    
    // === DEBOUNCING ===
    // If state changed, reset debounce timer
    if (state != lastState) {
        lastDebounce = now;
    }
    
    // Only process if state is stable for debounce period
    if ((now - lastDebounce) > DEBOUNCE_DELAY) {
        
        // === EDGE DETECTION ===
        if (state != currentState) {
            currentState = state;
            
            if (currentState) {
                // === BUTTON PRESSED ===
                pressed = true;
                pressTime = now;
                longPressHandled = false;
                
                // Check for double-click
                if (now - releaseTime < BUTTON_DOUBLE_CLICK_TIME) {
                    clickCount++;
                    if (clickCount >= 2) {
                        event = BUTTON_DOUBLE_CLICK;
                        clickCount = 0;  // Reset after double-click
                        
                        if (debugEnabled) {
                            DEBUG_PRINTF("Button %d: Double-click detected\n", pin);
                        }
                    }
                } else {
                    clickCount = 1;  // First click
                }
                
                if (debugEnabled && event != BUTTON_DOUBLE_CLICK) {
                    DEBUG_PRINTF("Button %d: Pressed\n", pin);
                }
                
            } else {
                // === BUTTON RELEASED ===
                pressed = false;
                releaseTime = now;
                uint32_t duration = now - pressTime;
                
                // Only generate short press if not already handled as long press
                if (!longPressHandled && duration < BUTTON_LONG_PRESS_TIME) {
                    if (clickCount == 1) {
                        event = BUTTON_SHORT_PRESS;
                        if (debugEnabled) {
                            DEBUG_PRINTF("Button %d: Short press (%lu ms)\n", pin, duration);
                        }
                    }
                }
                
                if (debugEnabled && event == BUTTON_NONE) {
                    DEBUG_PRINTF("Button %d: Released (duration: %lu ms)\n", pin, duration);
                }
            }
            
        } else if (pressed && !longPressHandled) {
            // === LONG PRESS DETECTION ===
            // Button is held, check for long press threshold
            if (now - pressTime >= BUTTON_LONG_PRESS_TIME) {
                event = BUTTON_LONG_PRESS;
                longPressHandled = true;  // Prevent multiple long press events
                clickCount = 0;  // Reset click count
                
                if (debugEnabled) {
                    DEBUG_PRINTF("Button %d: Long press detected\n", pin);
                }
            }
        }
    }
    
    // === CLICK COUNT TIMEOUT ===
    // Reset click count if too much time passed since last release
    if (!pressed && now - releaseTime > BUTTON_DOUBLE_CLICK_TIME * 2) {
        clickCount = 0;
    }
    
    // Update state tracking
    lastState = state;
    lastEvent = event;
    
    return event;
}

/**
 * @brief Check if button is currently pressed
 * @return True if pressed
 */
bool ButtonEx::isPressed() { 
    return pressed; 
}

/**
 * @brief Get button press duration
 * @return Duration in milliseconds (0 if not pressed)
 */
uint32_t ButtonEx::getPressedDuration() { 
    return pressed ? (millis() - pressTime) : 0; 
}

/**
 * @brief Get last detected event
 * @return Last event (does not clear)
 */
ButtonEvent ButtonEx::getLastEvent() { 
    return lastEvent; 
}

/**
 * @brief Reset button state
 */
void ButtonEx::reset() {
    pressed = false;
    longPressHandled = false;
    clickCount = 0;
    lastEvent = BUTTON_NONE;
    currentState = readButton();
    lastState = currentState;
}

// ============================================================================
// BUTTONHANDLER IMPLEMENTATION
// ============================================================================

/**
 * @brief Constructor - initializes button objects
 */
ButtonHandler::ButtonHandler() : 
    comboMode(false), 
    comboStartTime(0),
    lastButton1Event(BUTTON_NONE),
    lastButton2Event(BUTTON_NONE),
    lastButton3Event(BUTTON_NONE),
    lastButton4Event(BUTTON_NONE) {
    
    // Initialize button pins - static allocation, no new/delete
    buttons[0].setPin(BUTTON1_PIN);
    buttons[1].setPin(BUTTON2_PIN);
    buttons[2].setPin(BUTTON3_PIN);
    buttons[3].setPin(BUTTON4_PIN);
    
    // Initialize combo states
    memset(&comboB1B2, 0, sizeof(ComboState));
    memset(&comboB3B4, 0, sizeof(ComboState));
    memset(&comboB2B3, 0, sizeof(ComboState));
}

/**
 * @brief Initialize all buttons
 * @return True if successful
 */
bool ButtonHandler::begin() {
    DEBUG_PRINTLN("ButtonHandler: Initializing 4-button system...");
    
    // Initialize each button
    for (int i = 0; i < 4; i++) {
        buttons[i].begin();
        DEBUG_PRINTF("  Button %d on pin %d: OK\n", i+1, 
                    (i==0) ? BUTTON1_PIN : 
                    (i==1) ? BUTTON2_PIN : 
                    (i==2) ? BUTTON3_PIN : BUTTON4_PIN);
    }
    
    DEBUG_PRINTLN("ButtonHandler: Initialization complete");
    DEBUG_PRINTLN("  Combinations: B1+B2=Reset, B3+B4=Factory, B2+B3=Sound");
    
    return true;
}

/**
 * @brief Main update function - processes all buttons
 */
void ButtonHandler::update() {
    // === UPDATE ALL BUTTONS ===
    for (int i = 0; i < 4; i++) {
        ButtonEvent event = buttons[i].update();
        
        // Store events for getter functions
        if (event != BUTTON_NONE) {
            switch(i) {
                case 0: lastButton1Event = event; break;
                case 1: lastButton2Event = event; break;
                case 2: lastButton3Event = event; break;
                case 3: lastButton4Event = event; break;
            }
        }
    }
    
    // === CHECK COMBINATIONS ===
    checkCombinations();
    
    // === CLEANUP OLD EVENTS ===
    // Clear events that have been sitting for too long
    static uint32_t lastEventReset = 0;
    if (millis() - lastEventReset > BUTTON_COMBO_TIMEOUT) {
        lastEventReset = millis();
        clearProcessedEvents();
    }
}

/**
 * @brief Get and clear button 1 event
 * @return Button event (cleared after return)
 */
ButtonEvent ButtonHandler::getButton1Event() { 
    ButtonEvent event = lastButton1Event;
    lastButton1Event = BUTTON_NONE;
    return event;
}

/**
 * @brief Get and clear button 2 event
 * @return Button event (cleared after return)
 */
ButtonEvent ButtonHandler::getButton2Event() { 
    ButtonEvent event = lastButton2Event;
    lastButton2Event = BUTTON_NONE;
    return event;
}

/**
 * @brief Get and clear button 3 event
 * @return Button event (cleared after return)
 */
ButtonEvent ButtonHandler::getButton3Event() { 
    ButtonEvent event = lastButton3Event;
    lastButton3Event = BUTTON_NONE;
    return event;
}

/**
 * @brief Get and clear button 4 event
 * @return Button event (cleared after return)
 */
ButtonEvent ButtonHandler::getButton4Event() { 
    ButtonEvent event = lastButton4Event;
    lastButton4Event = BUTTON_NONE;
    return event;
}

/**
 * @brief Check if button 1 is pressed
 * @return True if pressed
 */
bool ButtonHandler::isButton1Pressed() { 
    return buttons[0].isPressed();
}

/**
 * @brief Check if button 2 is pressed
 * @return True if pressed
 */
bool ButtonHandler::isButton2Pressed() { 
    return buttons[1].isPressed();
}

/**
 * @brief Check if button 3 is pressed
 * @return True if pressed
 */
bool ButtonHandler::isButton3Pressed() { 
    return buttons[2].isPressed();
}

/**
 * @brief Check if button 4 is pressed
 * @return True if pressed
 */
bool ButtonHandler::isButton4Pressed() { 
    return buttons[3].isPressed();
}

/**
 * @brief Check if two buttons are pressed together
 * @param btn1 First button index (0-3)
 * @param btn2 Second button index (0-3)
 * @return True if both pressed
 */
bool ButtonHandler::isComboPressed(uint8_t btn1, uint8_t btn2) {
    if (btn1 >= 4 || btn2 >= 4) return false;
    return buttons[btn1].isPressed() && buttons[btn2].isPressed();
}

/**
 * @brief Get combo press duration
 * @param btn1 First button index (0-3)
 * @param btn2 Second button index (0-3)
 * @return Minimum duration of both buttons in ms
 */
uint32_t ButtonHandler::getComboDuration(uint8_t btn1, uint8_t btn2) {
    if (!isComboPressed(btn1, btn2)) return 0;
    
    // Return the shorter duration (both must be pressed)
    uint32_t duration1 = buttons[btn1].getPressedDuration();
    uint32_t duration2 = buttons[btn2].getPressedDuration();
    
    return min(duration1, duration2);
}

/**
 * @brief Update combo state machine
 * @param combo Combo state structure
 * @param isPressed Whether combo is currently pressed
 * @param requiredDuration Required hold time in ms
 * @return True if combo was just triggered
 */
bool ButtonHandler::updateComboState(ComboState& combo, bool isPressed, uint32_t requiredDuration) {
    bool triggered = false;
    
    if (isPressed && !combo.active) {
        // Combo just started
        combo.active = true;
        combo.startTime = millis();
        combo.handled = false;
        
    } else if (isPressed && combo.active && !combo.handled) {
        // Combo is held, check duration
        if (millis() - combo.startTime >= requiredDuration) {
            combo.handled = true;
            triggered = true;
        }
        
    } else if (!isPressed && combo.active) {
        // Combo released
        combo.active = false;
        combo.startTime = 0;
        combo.handled = false;
    }
    
    return triggered;
}

/**
 * @brief Check all button combinations
 */
void ButtonHandler::checkCombinations() {
    // === B1+B2: SYSTEM RESET (2 seconds) ===
    if (updateComboState(comboB1B2, isComboPressed(0, 1), 2000)) {
        DEBUG_PRINTLN("ButtonHandler: B1+B2 combo TRIGGERED - System Reset!");
        // Main code handles actual reset
    }
    
    // === B3+B4: FACTORY RESET (3 seconds) ===
    if (updateComboState(comboB3B4, isComboPressed(2, 3), 3000)) {
        DEBUG_PRINTLN("ButtonHandler: B3+B4 combo TRIGGERED - Factory Reset!");
        // Main code handles actual factory reset
    }
    
    // === B2+B3: SOUND TOGGLE (immediate) ===
    if (updateComboState(comboB2B3, isComboPressed(1, 2), 100)) {
        DEBUG_PRINTLN("ButtonHandler: B2+B3 combo TRIGGERED - Sound Toggle!");
        // Main code handles sound toggle
    }
    
    // Debug output for combo progress
    if (debugEnabled) {
        static uint32_t lastComboDebug = 0;
        if (millis() - lastComboDebug > 500) {  // Every 500ms
            lastComboDebug = millis();
            
            if (comboB1B2.active && !comboB1B2.handled) {
                uint32_t elapsed = millis() - comboB1B2.startTime;
                DEBUG_PRINTF("B1+B2 held for %lu ms (need 2000)\n", elapsed);
            }
            if (comboB3B4.active && !comboB3B4.handled) {
                uint32_t elapsed = millis() - comboB3B4.startTime;
                DEBUG_PRINTF("B3+B4 held for %lu ms (need 3000)\n", elapsed);
            }
        }
    }
}

/**
 * @brief Clear old/processed events
 */
void ButtonHandler::clearProcessedEvents() {
    // Only clear events if button is no longer pressed
    if (lastButton1Event != BUTTON_NONE && !buttons[0].isPressed()) {
        lastButton1Event = BUTTON_NONE;
    }
    if (lastButton2Event != BUTTON_NONE && !buttons[1].isPressed()) {
        lastButton2Event = BUTTON_NONE;
    }
    if (lastButton3Event != BUTTON_NONE && !buttons[2].isPressed()) {
        lastButton3Event = BUTTON_NONE;
    }
    if (lastButton4Event != BUTTON_NONE && !buttons[3].isPressed()) {
        lastButton4Event = BUTTON_NONE;
    }
}

/**
 * @brief Check if combo was just triggered
 * @param btn1 First button index
 * @param btn2 Second button index
 * @return True if combo was just triggered
 */
bool ButtonHandler::wasComboTriggered(uint8_t btn1, uint8_t btn2) {
    // Map button indices to combo states
    if (btn1 == 0 && btn2 == 1) return comboB1B2.handled && comboB1B2.active;
    if (btn1 == 2 && btn2 == 3) return comboB3B4.handled && comboB3B4.active;
    if (btn1 == 1 && btn2 == 2) return comboB2B3.handled && comboB2B3.active;
    return false;
}

/**
 * @brief Placeholder for factory reset
 * @note Actual reset is performed by main code
 */
void ButtonHandler::performFactoryReset() {
    DEBUG_PRINTLN("ButtonHandler: Factory Reset helper called");
    // Reset any button-specific settings if needed
    // Main code handles actual EEPROM clearing
}

/**
 * @brief Print current button states for debugging
 */
void ButtonHandler::printButtonStates() {
    DEBUG_PRINTLN("\n=== Button States ===");
    
    // Individual buttons
    for (int i = 0; i < 4; i++) {
        DEBUG_PRINTF("Button %d: %s", 
                     i + 1, 
                     buttons[i].isPressed() ? "PRESSED" : "released");
        
        if (buttons[i].isPressed()) {
            DEBUG_PRINTF(" (held for %lu ms)", buttons[i].getPressedDuration());
        }
        
        // Show last event
        ButtonEvent lastEvent = buttons[i].getLastEvent();
        if (lastEvent != BUTTON_NONE) {
            DEBUG_PRINTF(" [Last: %s]", 
                        lastEvent == BUTTON_SHORT_PRESS ? "SHORT" :
                        lastEvent == BUTTON_LONG_PRESS ? "LONG" :
                        lastEvent == BUTTON_DOUBLE_CLICK ? "DOUBLE" : "?");
        }
        
        DEBUG_PRINTLN();
    }
    
    // Active combinations
    DEBUG_PRINTLN("\nActive Combinations:");
    
    if (isComboPressed(0, 1)) {
        DEBUG_PRINTF("  B1+B2: %lu ms (Reset at 2000ms)\n", getComboDuration(0, 1));
    }
    if (isComboPressed(2, 3)) {
        DEBUG_PRINTF("  B3+B4: %lu ms (Factory at 3000ms)\n", getComboDuration(2, 3));
    }
    if (isComboPressed(1, 2)) {
        DEBUG_PRINTF("  B2+B3: %lu ms (Sound toggle)\n", getComboDuration(1, 2));
    }
    
    DEBUG_PRINTLN("==================\n");
}

/**
 * @brief Get button object for advanced operations
 * @param index Button index (0-3)
 * @return Button object pointer or nullptr
 */
ButtonEx* ButtonHandler::getButton(uint8_t index) {
    if (index < 4) {
        return &buttons[index];  // Return address of static object
    }
    return nullptr;
}

/**
 * @brief Enable/disable debug output
 * @param enable True to enable debug
 */
void ButtonHandler::setDebugEnabled(bool enable) {
    debugEnabled = enable;
    DEBUG_PRINTF("ButtonHandler: Debug output %s\n", enable ? "enabled" : "disabled");
}