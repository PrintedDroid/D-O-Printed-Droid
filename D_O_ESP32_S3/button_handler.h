/**
 * @file button_handler.h
 * @brief Button Handler for D-O Droid - CORRECTED VERSION
 * @version 2.1
 * @date 2024
 * 
 * @details Corrected version with static memory allocation
 *          and configurable timing constants
 */

#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>
#include "config.h"

// Button timing constants (moved from implementation)
#define BUTTON_DOUBLE_CLICK_TIME 300    // Time window for double-click detection
#define BUTTON_HOLD_REPEAT_TIME 500     // Time between hold repeat events
#define BUTTON_COMBO_TIMEOUT 100        // Timeout for clearing combo state

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @enum ButtonEvent
 * @brief Possible button events that can be detected
 */
enum ButtonEvent {
    BUTTON_NONE,          ///< No event
    BUTTON_SHORT_PRESS,   ///< Button pressed and released quickly (<1s)
    BUTTON_LONG_PRESS,    ///< Button held for long duration (>=1s)
    BUTTON_DOUBLE_CLICK,  ///< Button clicked twice rapidly
    BUTTON_HOLD           ///< Button is being held (continuous event)
};

// ============================================================================
// BUTTON CLASS
// ============================================================================

/**
 * @class ButtonEx
 * @brief Extended button class with advanced event detection
 */
class ButtonEx {
private:
    // Pin configuration
    uint8_t pin;              ///< GPIO pin number
    bool inverted;            ///< True for active-low (typical with pull-up)
    
    // State tracking
    bool currentState;        ///< Current debounced state
    bool lastState;           ///< Previous raw state (for edge detection)
    bool pressed;             ///< True while button is pressed
    bool longPressHandled;    ///< Flag to prevent multiple long press events
    
    // Timing
    uint32_t pressTime;       ///< Timestamp when button was pressed
    uint32_t releaseTime;     ///< Timestamp when button was released
    uint32_t lastDebounce;    ///< Last time state changed (for debouncing)
    
    // Event detection
    uint8_t clickCount;       ///< Click counter for multi-click detection
    ButtonEvent lastEvent;    ///< Last detected event
    
public:
    /**
     * @brief Constructor for ButtonEx
     * @param buttonPin GPIO pin number for the button
     * @param activelow True if button connects to GND when pressed (default)
     */
    ButtonEx(uint8_t buttonPin = 0, bool activelow = true);
    
    /**
     * @brief Initialize the button (configure pin mode)
     */
    void begin();
    
    /**
     * @brief Read raw button state
     * @return True if button is pressed (after inversion if needed)
     */
    bool readButton();
    
    /**
     * @brief Update button state and check for events
     * @return Detected button event (if any)
     */
    ButtonEvent update();
    
    /**
     * @brief Check if button is currently pressed
     * @return True if button is pressed
     */
    bool isPressed();
    
    /**
     * @brief Get duration button has been pressed
     * @return Press duration in milliseconds (0 if not pressed)
     */
    uint32_t getPressedDuration();
    
    /**
     * @brief Get last detected event
     * @return Last button event
     */
    ButtonEvent getLastEvent();
    
    /**
     * @brief Reset button state
     */
    void reset();
    
    /**
     * @brief Set button pin (for initialization)
     * @param buttonPin GPIO pin number
     */
    void setPin(uint8_t buttonPin) { pin = buttonPin; }
};

// ============================================================================
// BUTTON HANDLER CLASS
// ============================================================================

/**
 * @class ButtonHandler
 * @brief Manages multiple buttons with combination detection
 */
class ButtonHandler {
private:
    // Static allocation of button objects
    ButtonEx buttons[4];
    
    // Combination tracking
    bool comboMode;           ///< True when a combination is being tracked
    uint32_t comboStartTime;  ///< When current combo started
    
    // Event storage
    ButtonEvent lastButton1Event;  ///< Stored event for button 1
    ButtonEvent lastButton2Event;  ///< Stored event for button 2
    ButtonEvent lastButton3Event;  ///< Stored event for button 3
    ButtonEvent lastButton4Event;  ///< Stored event for button 4
    
    // Combination state tracking
    struct ComboState {
        uint32_t startTime;   ///< When this combo was first detected
        bool handled;         ///< Whether the combo has been triggered
        bool active;          ///< Whether the combo is currently pressed
    };
    
    ComboState comboB1B2;     ///< State for Button1+Button2 combo
    ComboState comboB3B4;     ///< State for Button3+Button4 combo
    ComboState comboB2B3;     ///< State for Button2+Button3 combo
    
    /**
     * @brief Check all button combinations
     */
    void checkCombinations();
    
    /**
     * @brief Clear processed events
     */
    void clearProcessedEvents();
    
    /**
     * @brief Update combo state
     * @param combo Reference to combo state structure
     * @param isPressed Whether the combo is currently pressed
     * @param requiredDuration How long combo must be held (ms)
     * @return True if combo was just triggered
     */
    bool updateComboState(ComboState& combo, bool isPressed, uint32_t requiredDuration);
    
public:
    /**
     * @brief Constructor - initializes button objects
     */
    ButtonHandler();
    
    /**
     * @brief Destructor - no dynamic allocation to clean up
     */
    ~ButtonHandler() {}
    
    /**
     * @brief Initialize all buttons
     * @return True if initialization successful
     */
    bool begin();
    
    /**
     * @brief Update all buttons and check for events
     * @note Must be called regularly from main loop
     */
    void update();
    
    // ========== Individual Button Event Getters ==========
    /**
     * @brief Get and clear event for button 1
     * @return Button event (cleared after return)
     */
    ButtonEvent getButton1Event();
    
    /**
     * @brief Get and clear event for button 2
     * @return Button event (cleared after return)
     */
    ButtonEvent getButton2Event();
    
    /**
     * @brief Get and clear event for button 3
     * @return Button event (cleared after return)
     */
    ButtonEvent getButton3Event();
    
    /**
     * @brief Get and clear event for button 4
     * @return Button event (cleared after return)
     */
    ButtonEvent getButton4Event();
    
    // ========== Button State Checks ==========
    /**
     * @brief Check if button 1 is currently pressed
     * @return True if pressed
     */
    bool isButton1Pressed();
    
    /**
     * @brief Check if button 2 is currently pressed
     * @return True if pressed
     */
    bool isButton2Pressed();
    
    /**
     * @brief Check if button 3 is currently pressed
     * @return True if pressed
     */
    bool isButton3Pressed();
    
    /**
     * @brief Check if button 4 is currently pressed
     * @return True if pressed
     */
    bool isButton4Pressed();
    
    // ========== Combination Detection ==========
    /**
     * @brief Check if two buttons are pressed together
     * @param btn1 First button index (0-3)
     * @param btn2 Second button index (0-3)
     * @return True if both buttons are pressed
     */
    bool isComboPressed(uint8_t btn1, uint8_t btn2);
    
    /**
     * @brief Get duration of button combination press
     * @param btn1 First button index (0-3)
     * @param btn2 Second button index (0-3)
     * @return Duration in milliseconds (0 if not pressed)
     */
    uint32_t getComboDuration(uint8_t btn1, uint8_t btn2);
    
    /**
     * @brief Check if a specific combo was just triggered
     * @param btn1 First button index (0-3)
     * @param btn2 Second button index (0-3)
     * @return True if combo was just triggered
     */
    bool wasComboTriggered(uint8_t btn1, uint8_t btn2);
    
    // ========== Utility Functions ==========
    /**
     * @brief Perform factory reset cleanup
     * @note Actual reset is handled by main code
     */
    void performFactoryReset();
    
    /**
     * @brief Print current button states for debugging
     */
    void printButtonStates();
    
    /**
     * @brief Get button object for advanced operations
     * @param index Button index (0-3)
     * @return Pointer to ButtonEx object (nullptr if invalid index)
     */
    ButtonEx* getButton(uint8_t index);
    
    /**
     * @brief Enable/disable debug output
     * @param enable True to enable debug messages
     */
    void setDebugEnabled(bool enable);
};

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

/**
 * @brief Global button handler instance
 * @note Automatically created, just call buttonHandler.begin() in setup()
 */
extern ButtonHandler buttonHandler;

#endif // BUTTON_HANDLER_H