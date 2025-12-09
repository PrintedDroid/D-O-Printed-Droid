/**
 * D-O Droid Controller for ESP32-S3
 * Main control program - DEBUG VERSION with extensive logging
 * 
 * Board: TENSTAR TS-ESP32-S3 (Adafruit Feather ESP32-S3 TFT Clone)
 * Version: 1.1.1 - Debug Version
 * Date: 2024
 */

#include "config.h"
#include "imu_handler.h"
#include "sound_controller.h"
#include "display_handler.h"
#include "rc_receiver.h"
#include "utilities.h"
#include "button_handler.h"
#include "pid_autotune.h"
#include <ESP32Servo.h>
#include <Preferences.h>

// For watchdog and reset reason
#include "esp_system.h"
#include "esp_task_wdt.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// System preferences
Preferences systemPrefs;

// Servos
Servo servoMainbar;
Servo servoHead1;
Servo servoHead2;
Servo servoHead3;

// Balance control variables
struct BalanceControl {
    float targetAngle = 0.0f;
    float currentAngle = 0.0f;
    float angleError = 0.0f;
    float angleErrorSum = 0.0f;
    float lastAngleError = 0.0f;
    float pidOutput = 0.0f;
    
    // PID gains
    float kp = DEFAULT_KP;
    float ki = DEFAULT_KI;
    float kd = DEFAULT_KD;
    
    // Motor outputs
    int motor1Speed = 0;
    int motor2Speed = 0;
    int motor1Dir = 0;
    int motor2Dir = 0;
};

BalanceControl balance;

// RC control variables
struct RCControl {
    int channels[10] = {0};
    bool isConnected = false;
    uint32_t lastUpdate = 0;
    
    // Processed values
    float steering = 0.0f;    // -1.0 to 1.0
    float throttle = 0.0f;    // -1.0 to 1.0
    float headTilt = 0.0f;    // -1.0 to 1.0
    float headPan = 0.0f;     // -1.0 to 1.0
    bool mute = false;
    int soundTrigger = 0;
};

RCControl rcControl;

// System state
SystemState currentState = STATE_INIT;

// Debug and display modes
bool debugOutputEnabled = false;
bool displayAlwaysOn = false;
bool soundEnabled = false;  // Sound mute control
uint8_t displayMode = 0;  // 0=Telemetry, 1=Diagnostics, 2=Off
bool inPIDMenu = false;   // PID menu state
uint8_t pidMenuSelection = 0;  // 0=Kp, 1=Ki, 2=Kd, 3=Auto-Tune
bool inAutoTune = false;  // Auto-tune state

// Timing variables
uint32_t lastIMUUpdate = 0;
uint32_t lastPIDUpdate = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t lastTelemetryUpdate = 0;
uint32_t lastStatusLEDUpdate = 0;
uint32_t lastWatchdogFeed = 0;
uint32_t systemStartTime = 0;

// Button handling
struct Button {
    bool pressed = false;
    bool lastState = false;
    uint32_t lastDebounce = 0;
};

Button button1, button2;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void updateIMU();
void updateBalanceControl();
void initializeMotors();
void updateMotors();
void stopMotors();
void initializeServos();
void updateServos();
void initializeRCReceiver();
void updateRCControl();
void handleButtons();
void onButton1Press();
void onButton2Press();
void onButton3Press();
void onButton4Press();
void handleButtonCombos();
void handleCalibration();
void updateDisplay();
void emergencyStop();
void loadPreferences();
void savePreferences();
void printResetReason();
void toggleDebugOutput();
void cycleDisplayMode();
void enterPIDMenu();
void exitPIDMenu();
void updatePIDDisplay();
void adjustPIDValue();
void toggleDisplayAlwaysOn();
void showSystemInfo();
void performFactoryReset();
void startAutoTune();

// ============================================================================
// PRINT RESET REASON - COMPLETE FUNCTION
// ============================================================================

void printResetReason() {
    esp_reset_reason_t reset_reason = esp_reset_reason();
    
    Serial.println("\n=====================================");
    Serial.print("RESET REASON: ");
    
    switch(reset_reason) {
        case ESP_RST_POWERON:
            Serial.println("Power-on reset (Normal startup)");
            break;
            
        case ESP_RST_SW:
            Serial.println("Software reset (ESP.restart() called)");
            break;
            
        case ESP_RST_PANIC:
            Serial.println("!!! PANIC/EXCEPTION RESET !!!");
            Serial.println("!!! Software crashed - check for:");
            Serial.println("!!! - Null pointer access");
            Serial.println("!!! - Stack overflow");
            Serial.println("!!! - Array out of bounds");
            Serial.println("!!! - Division by zero");
            break;
            
        case ESP_RST_INT_WDT:
            Serial.println("!!! INTERRUPT WATCHDOG RESET !!!");
            Serial.println("!!! An interrupt handler took too long!");
            break;
            
        case ESP_RST_TASK_WDT:
            Serial.println("!!! TASK WATCHDOG RESET !!!");
            Serial.println("!!! Main loop was blocked for too long!");
            Serial.println("!!! Check for infinite loops or blocking code!");
            break;
            
        case ESP_RST_WDT:
            Serial.println("!!! OTHER WATCHDOG RESET !!!");
            break;
            
        case ESP_RST_DEEPSLEEP:
            Serial.println("Reset after deep sleep");
            break;
            
        case ESP_RST_BROWNOUT:
            Serial.println("!!! BROWNOUT RESET !!!");
            Serial.println("!!! Power supply voltage dropped too low!");
            Serial.println("!!! Check:");
            Serial.println("!!! - USB cable quality");
            Serial.println("!!! - Power supply capacity");
            Serial.println("!!! - Current draw from motors/servos");
            break;
            
        case ESP_RST_SDIO:
            Serial.println("Reset over SDIO");
            break;
            
        default:
            Serial.printf("Unknown reset reason: %d\n", reset_reason);
            break;
    }
    
    Serial.println("=====================================\n");
    Serial.flush();  // Make sure it's printed
    delay(100);      // Give time to see it
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // VERY FIRST: Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 2000) { delay(10); }  // Wait max 2 seconds
    
    Serial.println("\n\n\n================================================");
    Serial.println("BOOT: ESP32-S3 Starting...");
    Serial.println("================================================");
    Serial.flush();
    delay(100);
    
    // Print reset reason IMMEDIATELY
    printResetReason();
    
    // Now regular debug serial
    DEBUG_BEGIN(SERIAL_BAUD);
    delay(500); // Wait for serial to stabilize
    
    DEBUG_PRINTLN("=== D-O Droid ESP32-S3 Starting ===");
    DEBUG_PRINTLN("Version: 1.1.1 Debug");
    DEBUG_PRINTF("Free heap at start: %d bytes\n", ESP.getFreeHeap());
    
    // Configure watchdog - 10 seconds timeout
    DEBUG_PRINTLN("CHECKPOINT 1: Configuring watchdog...");
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,  // 10 seconds
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // All cores
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    DEBUG_PRINTLN("CHECKPOINT 1: OK");
    
    // Critical: Enable I2C power first!
    DEBUG_PRINTLN("CHECKPOINT 2: Enabling I2C power...");
    pinMode(TFT_I2C_POWER_PIN, OUTPUT);
    digitalWrite(TFT_I2C_POWER_PIN, HIGH);
    delay(100);
    DEBUG_PRINTLN("CHECKPOINT 2: OK - I2C power enabled");
    
    // Initialize utilities (includes status LED)
    DEBUG_PRINTLN("CHECKPOINT 3: Initializing utilities...");
    if (utilities.begin()) {
        DEBUG_PRINTLN("CHECKPOINT 3: OK");
        utilities.setLED(LED_YELLOW); // Yellow = initializing
    } else {
        DEBUG_PRINTLN("CHECKPOINT 3: FAILED");
    }
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Initialize buttons with new handler
    DEBUG_PRINTLN("CHECKPOINT 4: Initializing buttons...");
    buttonHandler.begin();
    DEBUG_PRINTLN("CHECKPOINT 4: OK (4 buttons)");
    
    // Initialize display
    DEBUG_PRINTLN("CHECKPOINT 5: Initializing display...");
    if (displayHandler.begin()) {
        DEBUG_PRINTLN("CHECKPOINT 5: OK");
        displayHandler.showSplashScreen();
    } else {
        DEBUG_PRINTLN("CHECKPOINT 5: FAILED");
    }
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Initialize I2C
    DEBUG_PRINTLN("CHECKPOINT 6: Initializing I2C bus...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    delay(100);
    DEBUG_PRINTLN("CHECKPOINT 6: OK - I2C bus initialized");
    
    // Initialize IMU system
    DEBUG_PRINTLN("CHECKPOINT 7: Initializing IMU system...");
    IMUMode imuMode = IMU_MODE_AUTO;  // Always use AUTO mode
    
    if (imuHandler.begin(imuMode)) {
        DEBUG_PRINTLN("CHECKPOINT 7: OK");
        char imuStatusBuffer[256];
        imuHandler.getStatusString(imuStatusBuffer, sizeof(imuStatusBuffer));
        DEBUG_PRINTLN(imuStatusBuffer);
    } else {
        DEBUG_PRINTLN("CHECKPOINT 7: FAILED");
        currentState = STATE_ERROR;
        displayHandler.showError("No IMU Found");
        utilities.setLED(LED_RED);
        
        // Stay in error state but don't block
        while(1) {
            esp_task_wdt_reset();
            utilities.update();
            delay(100);
        }
    }
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Initialize sound system (can be enabled/disabled)
    DEBUG_PRINTLN("CHECKPOINT 8: Initializing sound system...");
    if (soundController.begin()) {
        DEBUG_PRINTLN("CHECKPOINT 8: OK");
        soundEnabled = true;  // Enable sound by default
        soundController.playStartupSequence();
    } else {
        DEBUG_PRINTLN("CHECKPOINT 8: FAILED - continuing without sound");
        soundEnabled = false;
    }
    DEBUG_PRINTLN("CHECKPOINT 8: Complete");
    
    // Initialize motor pins
    DEBUG_PRINTLN("CHECKPOINT 9: Initializing motors...");
    initializeMotors();
    DEBUG_PRINTLN("CHECKPOINT 9: OK");
    
    // Initialize servos
    DEBUG_PRINTLN("CHECKPOINT 10: Initializing servos...");
    initializeServos();
    DEBUG_PRINTLN("CHECKPOINT 10: OK");
    
    // Initialize RC receiver
    DEBUG_PRINTLN("CHECKPOINT 11: Initializing RC receiver...");
    initializeRCReceiver();
    DEBUG_PRINTLN("CHECKPOINT 11: OK");
    
    // Load system preferences
    DEBUG_PRINTLN("CHECKPOINT 12: Loading preferences...");
    loadPreferences();
    DEBUG_PRINTLN("CHECKPOINT 12: OK");
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Update display
    DEBUG_PRINTLN("CHECKPOINT 13: Final display update...");
    displayHandler.clearScreen();
    displayHandler.showStatus("System Ready", COLOR_GREEN);
    DEBUG_PRINTLN("CHECKPOINT 13: OK");
    
    // System ready
    systemStartTime = millis();
    currentState = STATE_READY;
    utilities.setLED(LED_GREEN); // Green = ready
    
    DEBUG_PRINTLN("================================================");
    DEBUG_PRINTLN("=== Setup Complete ===");
    DEBUG_PRINTF("Free heap: %d bytes\n", ESP.getFreeHeap());
    DEBUG_PRINTF("Setup took: %lu ms\n", millis());
    DEBUG_PRINTLN("================================================\n");
    
    // Add a delay to see if we make it past setup
    DEBUG_PRINTLN("Waiting 2 seconds before entering loop...");
    delay(2000);
    DEBUG_PRINTLN("Entering main loop now!");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    static uint32_t loopCounter = 0;
    static uint32_t lastLoopReport = 0;
    
    uint32_t currentTime = millis();
    
    // Report every 5 seconds that we're still running
    if (currentTime - lastLoopReport > 5000) {
        lastLoopReport = currentTime;
        DEBUG_PRINTF("Main loop running - Counter: %lu, Uptime: %lu ms, Free heap: %d\n", 
                     loopCounter, currentTime, ESP.getFreeHeap());
    }
    
    loopCounter++;
    
    // Static variables moved outside switch
    static bool displayDisabled = false;
    static uint32_t lastServoUpdate = 0;
    static uint32_t lastRCUpdate = 0;
    static uint32_t lastSoundUpdate = 0;
    
    // Feed watchdog every 100ms
    if (currentTime - lastWatchdogFeed >= 100) {
        lastWatchdogFeed = currentTime;
        esp_task_wdt_reset();
        yield();
    }
    
    // Handle button inputs - but not in PID menu during certain states
    if (!inPIDMenu || currentState == STATE_READY) {
        handleButtons();
    }
    
    // Exit PID menu with Button 1
    if (inPIDMenu && buttonHandler.getButton1Event() == BUTTON_SHORT_PRESS) {
        exitPIDMenu();
    }
    
    // Update display non-blocking
    displayHandler.update();
    
    // Handle different states
    switch (currentState) {
        case STATE_READY:
            // Re-enable display when back to ready
            displayHandler.setBacklight(true);
            displayDisabled = false;
            
            // Minimal work in READY state
            if (button1.pressed || (rcControl.isConnected && rcControl.throttle > 0.1f)) {
                currentState = STATE_RUNNING;
                DEBUG_PRINTLN("Starting balance mode");
            }
            break;
            
        case STATE_RUNNING:
            // Turn off display for maximum performance (unless override)
            if (!displayAlwaysOn && !displayDisabled && !inAutoTune) {
                displayHandler.setBacklight(false);
                displayDisabled = true;
                DEBUG_PRINTLN("Display off for max performance");
            } else if ((displayAlwaysOn || inAutoTune) && displayDisabled) {
                displayHandler.setBacklight(true);
                displayDisabled = false;
            }
            
            // CRITICAL: Update IMU and PID as fast as possible!
            updateIMU();
            
            // Handle Auto-Tune if active
            if (inAutoTune) {
                if (pidAutoTune.update(balance.currentAngle)) {
                    // Auto-tune is running
                    balance.motor1Speed = pidAutoTune.getOutput();
                    balance.motor2Speed = pidAutoTune.getOutput();
                    
                    // Update display with progress
                    static uint32_t lastAutoTuneDisplay = 0;
                    if (millis() - lastAutoTuneDisplay > 500) {
                        lastAutoTuneDisplay = millis();
                        
                        displayHandler.clearScreen();
                        displayHandler.drawHeader("AUTO-TUNING");
                        
                        displayHandler.getDisplay()->setTextSize(1);  // ADD THIS LINE
                        
                        displayHandler.getDisplay()->setCursor(10, 30);  // Changed from 40
                        displayHandler.getDisplay()->print("Status: ");
                        displayHandler.getDisplay()->print(pidAutoTune.getStatusString());
                        
                        displayHandler.getDisplay()->setCursor(10, 45);  // Changed from 60
                        displayHandler.getDisplay()->print("Progress: ");
                        displayHandler.getDisplay()->print((int)pidAutoTune.getProgress());
                        displayHandler.getDisplay()->print("%");
                        
                        displayHandler.drawFooter("B1 to STOP", COLOR_RED);
                    }
                    
                    // Check if complete
                    if (pidAutoTune.isComplete()) {
                        // Get results
                        AutoTuneResults results = pidAutoTune.getResults();
                        if (results.valid) {
                            // Apply new PID values
                            balance.kp = results.Kp;
                            balance.ki = results.Ki;
                            balance.kd = results.Kd;
                            
                            // Save to preferences
                            savePreferences();
                            
                            DEBUG_PRINTF("Auto-Tune Complete! Kp=%.1f Ki=%.2f Kd=%.2f\n",
                                        balance.kp, balance.ki, balance.kd);
                        }
                        
                        inAutoTune = false;
                        currentState = STATE_READY;
                        stopMotors();
                    }
                } else {
                    // Auto-tune not running or error
                    if (pidAutoTune.getState() == AUTOTUNE_ERROR) {
                        inAutoTune = false;
                        currentState = STATE_READY;
                        stopMotors();
                    }
                }
            } else {
                // Normal balance control
                updateBalanceControl();
            }
            
            updateMotors();
            
            // Servos at 50Hz (still responsive)
            if (currentTime - lastServoUpdate >= 20) { // 50Hz
                lastServoUpdate = currentTime;
                updateServos();
            }
            
            // RC less frequently
            if (currentTime - lastRCUpdate >= 50) { // 20Hz is fine for RC
                lastRCUpdate = currentTime;
                updateRCControl();
            }
            
            // Check for emergency stop
            if (abs(balance.currentAngle) > ANGLE_LIMIT) {
                emergencyStop();
            }
            break;
            
        case STATE_CALIBRATING:
            handleCalibration();
            break;
            
        case STATE_ERROR:
        case STATE_EMERGENCY_STOP:
            stopMotors();
            break;
    }
    
    // Update sound controller
    if (soundEnabled && currentTime - lastSoundUpdate >= 50) { // 20Hz
        lastSoundUpdate = currentTime;
        soundController.update();
    }
    
    // Display update - queue updates for non-blocking operation
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        lastDisplayUpdate = currentTime;
        updateDisplay();
    }
    
    // LED status even less frequent
    if (currentTime - lastStatusLEDUpdate >= 500) {
        lastStatusLEDUpdate = currentTime;
        utilities.showStatus(currentState);
    }
    
    // Update utilities
    utilities.update();
}

// ============================================================================
// CORE FUNCTIONS
// ============================================================================

// ============================================================================
// IMU FUNCTIONS
// ============================================================================

void updateIMU() {
    if (!imuHandler.update()) {
        return;
    }
    
    // Get orientation
    balance.currentAngle = imuHandler.getPitch();
    
    // Optional debug output - controlled by button
    if (debugOutputEnabled) {
        static uint32_t lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 100) { // 10Hz debug
            lastDebugPrint = millis();
            DEBUG_PRINTF("P:%6.1f R:%6.1f Y:%6.1f M1:%4d M2:%4d Hz:%3.0f\n", 
                         balance.currentAngle, 
                         imuHandler.getRoll(),
                         imuHandler.getYaw(),
                         balance.motor1Speed,
                         balance.motor2Speed,
                         imuHandler.getStatus().update_rate);
        }
    }
}

// ============================================================================
// BALANCE CONTROL
// ============================================================================

void updateBalanceControl() {
    // Calculate error
    balance.angleError = balance.targetAngle - balance.currentAngle;
    
    // Integrate error (with anti-windup)
    balance.angleErrorSum += balance.angleError;
    balance.angleErrorSum = constrain(balance.angleErrorSum, -100, 100);
    
    // Calculate derivative
    float angleErrorDiff = balance.angleError - balance.lastAngleError;
    balance.lastAngleError = balance.angleError;
    
    // PID calculation
    balance.pidOutput = (balance.kp * balance.angleError) +
                       (balance.ki * balance.angleErrorSum) +
                       (balance.kd * angleErrorDiff);
    
    // Limit output
    balance.pidOutput = constrain(balance.pidOutput, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    
    // Apply to motor speeds
    int baseSpeed = balance.pidOutput;
    
    // Add RC control inputs
    int steeringDiff = rcControl.steering * 100;
    int throttleAdd = rcControl.throttle * 50;
    
    balance.motor1Speed = baseSpeed + throttleAdd + steeringDiff;
    balance.motor2Speed = baseSpeed + throttleAdd - steeringDiff;
    
    // Apply dead zone
    if (abs(balance.motor1Speed) < MOTOR_DEADZONE) balance.motor1Speed = 0;
    if (abs(balance.motor2Speed) < MOTOR_DEADZONE) balance.motor2Speed = 0;
    
    // Limit speeds
    balance.motor1Speed = constrain(balance.motor1Speed, -255, 255);
    balance.motor2Speed = constrain(balance.motor2Speed, -255, 255);
    
    // Set directions
    balance.motor1Dir = (balance.motor1Speed >= 0) ? 1 : 0;
    balance.motor2Dir = (balance.motor2Speed >= 0) ? 1 : 0;
}

// ============================================================================
// MOTOR CONTROL - WITH DEBUG
// ============================================================================

void initializeMotors() {
    DEBUG_PRINTLN("  Motor init: Configuring pins...");
    
    // Configure motor pins
    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR1_PWM_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_PWM_PIN, OUTPUT);
    
    DEBUG_PRINTLN("  Motor init: Attaching PWM channels...");
    
    // Configure PWM channels
    DEBUG_PRINTF("  Motor1 PWM on pin %d...\n", MOTOR1_PWM_PIN);
    ledcAttach(MOTOR1_PWM_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    
    DEBUG_PRINTF("  Motor2 PWM on pin %d...\n", MOTOR2_PWM_PIN);
    ledcAttach(MOTOR2_PWM_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    
    DEBUG_PRINTLN("  Motor init: Stopping motors...");
    
    // Stop motors
    stopMotors();
    
    DEBUG_PRINTLN("  Motor init: Complete");
}

void updateMotors() {
    // Set directions
    digitalWrite(MOTOR1_DIR_PIN, balance.motor1Dir);
    digitalWrite(MOTOR2_DIR_PIN, balance.motor2Dir);
    
    // Set speeds
    ledcWrite(MOTOR1_PWM_PIN, abs(balance.motor1Speed));
    ledcWrite(MOTOR2_PWM_PIN, abs(balance.motor2Speed));
}

void stopMotors() {
    ledcWrite(MOTOR1_PWM_PIN, 0);
    ledcWrite(MOTOR2_PWM_PIN, 0);
    balance.motor1Speed = 0;
    balance.motor2Speed = 0;
}

// ============================================================================
// SERVO CONTROL - WITH DEBUG
// ============================================================================

void initializeServos() {
    DEBUG_PRINTLN("  Servo init: Attaching servos...");
    
    // Attach servos
    DEBUG_PRINTF("  Mainbar servo on pin %d...\n", SERVO_MAINBAR_PIN);
    servoMainbar.attach(SERVO_MAINBAR_PIN);
    delay(50);
    
    DEBUG_PRINTF("  Head1 servo on pin %d...\n", SERVO_HEAD1_PIN);
    servoHead1.attach(SERVO_HEAD1_PIN);
    delay(50);
    
    DEBUG_PRINTF("  Head2 servo on pin %d...\n", SERVO_HEAD2_PIN);
    servoHead2.attach(SERVO_HEAD2_PIN);
    delay(50);
    
    DEBUG_PRINTF("  Head3 servo on pin %d...\n", SERVO_HEAD3_PIN);
    servoHead3.attach(SERVO_HEAD3_PIN);
    delay(50);
    
    DEBUG_PRINTLN("  Servo init: Centering all servos...");
    
    // Center all servos
    servoMainbar.write(90);
    servoHead1.write(90);
    servoHead2.write(90);
    servoHead3.write(90);
    
    DEBUG_PRINTLN("  Servo init: Complete");
}

void updateServos() {
    // Update head position based on RC input
    int headTiltAngle = 90 + (rcControl.headTilt * 45);
    int headPanAngle = 90 + (rcControl.headPan * 45);
    
    servoHead1.write(headTiltAngle);
    servoHead2.write(headPanAngle);
    
    // Mainbar can be used for additional stabilization
    int mainbarAngle = 90 + (balance.currentAngle * 2);
    mainbarAngle = constrain(mainbarAngle, 45, 135);
    servoMainbar.write(mainbarAngle);
}

// ============================================================================
// RC RECEIVER - WITH DEBUG
// ============================================================================

void initializeRCReceiver() {
    DEBUG_PRINTLN("  RC init: Starting iBus receiver...");
    
    // Initialize RC receiver
    if (!rcReceiver.begin(RC_PROTOCOL_IBUS)) {
        DEBUG_PRINTLN("  RC init: FAILED - continuing without RC");
    } else {
        DEBUG_PRINTLN("  RC init: Success");
    }
}

void updateRCControl() {
    // Update RC receiver
    if (rcReceiver.update()) {
        rcControl.isConnected = rcReceiver.isConnected();
        rcControl.lastUpdate = millis();
        
        // Get normalized values
        rcControl.steering = rcReceiver.getSteering();
        rcControl.throttle = rcReceiver.getThrottle();
        rcControl.headTilt = rcReceiver.getHeadTilt();
        rcControl.headPan = rcReceiver.getHeadPan();
        
        // Sound control
        rcControl.mute = rcReceiver.getMute();
        if (!rcControl.mute && soundEnabled) {
            soundController.setMute(false);
            
            // Handle sound triggers from RC
            int mode = rcReceiver.getMode();
            if (mode != rcControl.soundTrigger) {
                rcControl.soundTrigger = mode;
                
                switch (mode) {
                    case 0:
                        soundController.playRandomFromCategory(SOUND_CAT_NEGATIVE);
                        break;
                    case 1:
                        soundController.playRandomFromCategory(SOUND_CAT_GREETING);
                        break;
                    case 2:
                        soundController.playRandomFromCategory(SOUND_CAT_POSITIVE);
                        break;
                }
            }
        } else if (rcControl.mute) {
            soundController.setMute(true);
        }
    } else {
        // Check connection timeout
        if (millis() - rcControl.lastUpdate > 1000) {
            rcControl.isConnected = false;
        }
    }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handleButtons() {
    // Update button handler
    buttonHandler.update();
    
    // Get button events
    ButtonEvent btn1 = buttonHandler.getButton1Event();
    ButtonEvent btn2 = buttonHandler.getButton2Event();
    ButtonEvent btn3 = buttonHandler.getButton3Event();
    ButtonEvent btn4 = buttonHandler.getButton4Event();
    
    // Check button combinations first
    handleButtonCombos();
    
    // Handle individual buttons only if not in combo
    if (!buttonHandler.isComboPressed(0, 1) && 
        !buttonHandler.isComboPressed(2, 3) && 
        !buttonHandler.isComboPressed(1, 2)) {
        
        // Button 1: Start/Stop
        if (btn1 == BUTTON_SHORT_PRESS) {
            onButton1Press();
        } else if (btn1 == BUTTON_LONG_PRESS) {
            emergencyStop();
        }
        
        // Button 2: Calibrate/Setup
        if (btn2 == BUTTON_SHORT_PRESS) {
            onButton2Press();
        } else if (btn2 == BUTTON_LONG_PRESS) {
            enterPIDMenu();
        }
        
        // Button 3: Display Control
        if (btn3 == BUTTON_SHORT_PRESS) {
            onButton3Press();
        } else if (btn3 == BUTTON_LONG_PRESS) {
            toggleDisplayAlwaysOn();
        }
        
        // Button 4: Debug/Select/Info
        if (btn4 == BUTTON_SHORT_PRESS) {
            onButton4Press();
        } else if (btn4 == BUTTON_LONG_PRESS) {
            showSystemInfo();
        }
    }
}

void onButton1Press() {
    DEBUG_PRINTLN("Button 1 pressed");
    
    // Test timing variables
    static uint32_t testStart = 0;
    static uint32_t updateCountStart = 0;
    
    // Stop auto-tune if running
    if (inAutoTune) {
        pidAutoTune.stop();
        inAutoTune = false;
        currentState = STATE_READY;
        stopMotors();
        return;
    }
    
    switch (currentState) {
        case STATE_READY:
            currentState = STATE_RUNNING;
            testStart = millis();
            updateCountStart = imuHandler.getStatus().update_count;
            if (soundEnabled) {
                soundController.playSound(SOUND_DEFAULT);
            }
            break;
            
        case STATE_RUNNING: {
            currentState = STATE_READY;
            stopMotors();
            // Print average rate after stop
            uint32_t runtime = millis() - testStart;
            uint32_t updates = imuHandler.getStatus().update_count - updateCountStart;
            if (runtime > 0) {
                DEBUG_PRINTF("Test complete: %lu updates in %lu ms = %.1f Hz average\n", 
                            updates, runtime, (float)updates * 1000.0f / runtime);
            }
            break;
        }
            
        case STATE_ERROR:
        case STATE_EMERGENCY_STOP:
            // Reset system
            ESP.restart();
            break;
    }
}

void onButton2Press() {
    DEBUG_PRINTLN("Button 2 pressed");
    
    // Start calibration
    if (currentState == STATE_READY && !inPIDMenu) {
        currentState = STATE_CALIBRATING;
        displayHandler.showStatus("Calibrating IMU...", COLOR_YELLOW);
    }
}

void onButton3Press() {
    DEBUG_PRINTLN("Button 3 pressed - Display mode");
    
    if (inPIDMenu) {
        // In PID menu - navigate
        pidMenuSelection = (pidMenuSelection + 1) % 4;  // Now 4 options
        updatePIDDisplay();
        return;
    }
    
    // Cycle through: Telemetry → Diagnostics → Off
    displayMode = (displayMode + 1) % 3;
    
    switch (displayMode) {
        case 0: // Telemetry
            displayHandler.setBacklight(true);
            displayHandler.setMode(DISPLAY_MODE_TELEMETRY);
            displayHandler.showStatus("Telemetry Mode", COLOR_GREEN);
            break;
        case 1: // Diagnostics
            displayHandler.setBacklight(true);
            displayHandler.setMode(DISPLAY_MODE_DIAGNOSTICS);
            displayHandler.showStatus("Diagnostics Mode", COLOR_GREEN);
            break;
        case 2: // Display Off
            displayHandler.showStatus("Display OFF", COLOR_YELLOW);
            delay(500);
            displayHandler.setBacklight(false);
            break;
    }
}

void onButton4Press() {
    DEBUG_PRINTLN("Button 4 pressed - Multi-function");
    
    if (inPIDMenu) {
        // In PID menu - adjust value
        adjustPIDValue();
    } else {
        // Toggle debug output
        toggleDebugOutput();
    }
}

void handleButtonCombos() {
    static uint32_t comboStart = 0;
    static uint32_t factoryStart = 0;
    static bool soundComboHandled = false;
    
    // B1+B2: System Reset
    if (buttonHandler.isComboPressed(0, 1)) {
        if (comboStart == 0) {
            comboStart = millis();
            displayHandler.showStatus("Hold for Reset...", COLOR_YELLOW);
        } else if (millis() - comboStart > 2000) {
            displayHandler.showStatus("SYSTEM RESET", COLOR_RED);
            delay(1000);
            ESP.restart();
        }
    } else {
        comboStart = 0;
    }
    
    // B3+B4: Factory Reset
    if (buttonHandler.isComboPressed(2, 3)) {
        if (factoryStart == 0) {
            factoryStart = millis();
            displayHandler.showStatus("Hold 3s for Factory Reset", COLOR_RED);
        } else if (millis() - factoryStart > 3000) {
            performFactoryReset();
        }
    } else {
        factoryStart = 0;
    }
    
    // B2+B3: Sound Toggle
    if (buttonHandler.isComboPressed(1, 2)) {
        if (!soundComboHandled) {
            soundEnabled = !soundEnabled;
            displayHandler.showStatus(soundEnabled ? "Sound ON" : "Sound MUTED", COLOR_CYAN);
            soundController.setMute(!soundEnabled);
            soundComboHandled = true;
        }
    } else {
        soundComboHandled = false;
    }
}

// ============================================================================
// CALIBRATION
// ============================================================================

void handleCalibration() {
    static bool calibrationStarted = false;
    static uint32_t calibrationStartTime = 0;
    static bool messageShown = false;
    
    if (!calibrationStarted) {
        calibrationStarted = true;
        calibrationStartTime = millis();
        messageShown = false;
        imuHandler.calibrate(1000);
    }
    
    // Show progress
    float progress = min(100.0f, (millis() - calibrationStartTime) * 100.0f / 5000.0f);
    displayHandler.showCalibrationProgress(progress);
    
    // Check if calibration is complete (about 5 seconds)
    if (millis() - calibrationStartTime > 5000) {
        if (!messageShown) {
            displayHandler.showStatus("Calibration Complete", COLOR_GREEN);
            savePreferences();
            if (soundEnabled) {
                soundController.playSuccessSound();
            }
            messageShown = true;
        }
        
        // Wait another 2 seconds to show the message
        if (millis() - calibrationStartTime > 7000) {
            calibrationStarted = false;
            messageShown = false;
            currentState = STATE_READY;
            
            // Return to telemetry mode
            displayHandler.setMode(DISPLAY_MODE_TELEMETRY);
        }
    }
}

// ============================================================================
// PID MENU FUNCTIONS
// ============================================================================

void enterPIDMenu() {
    if (currentState != STATE_READY) return;
    
    DEBUG_PRINTLN("Entering PID Menu");
    inPIDMenu = true;
    pidMenuSelection = 0;
    displayHandler.setBacklight(true);
    updatePIDDisplay();
}

void exitPIDMenu() {
    inPIDMenu = false;
    displayMode = 0; // Back to telemetry
    displayHandler.setMode(DISPLAY_MODE_TELEMETRY);
    savePreferences();
    displayHandler.showStatus("PID Saved", COLOR_GREEN);
}

void updatePIDDisplay() {
    displayHandler.clearScreen();
    displayHandler.drawHeader("PID TUNING");
    
    // Smaller text and tighter spacing
    displayHandler.getDisplay()->setTextSize(1);
    
    int y = 25;  // Start higher
    const char* labels[] = {"Kp:", "Ki:", "Kd:", "Auto-Tune"};
    float* values[] = {&balance.kp, &balance.ki, &balance.kd, nullptr};
    
    for (int i = 0; i < 4; i++) {
        displayHandler.getDisplay()->setCursor(10, y);
        
        // Highlight selected
        if (i == pidMenuSelection) {
            displayHandler.getDisplay()->fillRect(5, y-2, 230, 12, COLOR_YELLOW);
            displayHandler.getDisplay()->setTextColor(COLOR_BLACK);
        } else {
            displayHandler.getDisplay()->setTextColor(COLOR_WHITE);
        }
        
        displayHandler.getDisplay()->print(labels[i]);
        
        if (i < 3) {
            // PID values
            displayHandler.getDisplay()->setCursor(50, y);
            displayHandler.getDisplay()->print(*values[i], 2);
        } else {
            // Auto-Tune option
            displayHandler.getDisplay()->setCursor(80, y);
            displayHandler.getDisplay()->print("[Run]");
        }
        
        y += 15;  // Reduced spacing
    }
    
    displayHandler.getDisplay()->setTextColor(COLOR_WHITE);
    displayHandler.drawFooter("B3:Sel B4:Chg B1:Exit", COLOR_CYAN);
}

void adjustPIDValue() {
    if (pidMenuSelection == 3) {
        // Auto-Tune selected
        exitPIDMenu();
        startAutoTune();
        return;
    }
    
    float* values[] = {&balance.kp, &balance.ki, &balance.kd};
    float increments[] = {0.5f, 0.1f, 0.1f};
    
    *values[pidMenuSelection] += increments[pidMenuSelection];
    
    // Wrap around at reasonable limits
    if (balance.kp > 50.0f) balance.kp = 0.0f;
    if (balance.ki > 5.0f) balance.ki = 0.0f;
    if (balance.kd > 5.0f) balance.kd = 0.0f;
    
    updatePIDDisplay();
}

// ============================================================================
// AUTO-TUNE
// ============================================================================

void startAutoTune() {
    if (currentState != STATE_READY || inAutoTune) return;
    
    DEBUG_PRINTLN("Starting Auto-Tune");
    
    // Show warning
    displayHandler.clearScreen();
    displayHandler.drawHeader("AUTO-TUNE");
    displayHandler.getDisplay()->setTextSize(1);
    displayHandler.getDisplay()->setCursor(10, 40);
    displayHandler.getDisplay()->print("D-O must be upright");
    displayHandler.getDisplay()->setCursor(10, 55);
    displayHandler.getDisplay()->print("WITHOUT head mounted!");
    displayHandler.getDisplay()->setCursor(10, 70);
    displayHandler.getDisplay()->print("Will oscillate ~30sec");
    displayHandler.getDisplay()->setCursor(10, 90);
    displayHandler.getDisplay()->print("Press B1 to START");
    displayHandler.getDisplay()->setCursor(10, 105);
    displayHandler.getDisplay()->print("Press B2 to CANCEL");
    
    // Wait for button
    uint32_t waitStart = millis();
    while (millis() - waitStart < 10000) {  // 10 second timeout
        buttonHandler.update();
        
        if (buttonHandler.getButton1Event() == BUTTON_SHORT_PRESS) {
            // Start auto-tune
            if (pidAutoTune.start(balance.currentAngle)) {
                inAutoTune = true;
                currentState = STATE_RUNNING;
                displayHandler.clearScreen();
                return;
            }
        }
        
        if (buttonHandler.getButton2Event() == BUTTON_SHORT_PRESS) {
            // Cancel
            displayHandler.showStatus("Auto-Tune Cancelled", COLOR_YELLOW);
            delay(1000);
            displayHandler.clearScreen();
            return;
        }
        
        esp_task_wdt_reset();
        delay(10);
    }
    
    // Timeout
    displayHandler.showStatus("Auto-Tune Timeout", COLOR_RED);
    delay(1000);
    displayHandler.clearScreen();
}

// ============================================================================
// DISPLAY UPDATE
// ============================================================================

void updateDisplay() {
    // Don't update if display is off (mode 2) or busy
    if ((displayMode == 2 && !displayAlwaysOn && !inPIDMenu) || displayHandler.isBusy()) {
        return;
    }
    
    // Handle PID menu separately
    if (inPIDMenu) {
        updatePIDDisplay();
        return;
    }
    
    // Queue display updates based on current mode
    switch (displayMode) {
        case 0: // Telemetry
            displayHandler.queueTelemetryUpdate(
                balance.currentAngle,
                imuHandler.getRoll(),
                imuHandler.getYaw(),
                balance.motor1Speed,
                balance.motor2Speed,
                rcControl.isConnected,
                imuHandler.getStatus().update_rate
            );
            break;
            
        case 1: { // Diagnostics
            char imuStatus[32];
            char soundStatus[32];
            
            snprintf(imuStatus, sizeof(imuStatus), "%s", 
                     imuHandler.getStatus().qmi_initialized ? "QMI OK" : "QMI FAIL");
            snprintf(soundStatus, sizeof(soundStatus), "%s",
                     soundEnabled ? "ENABLED" : "MUTED");
            
            displayHandler.queueDiagnosticsUpdate(
                imuStatus,
                soundStatus,
                0,  // CPU temp not available on ESP32-S3
                imuHandler.getTemperature(),
                ESP.getFreeHeap(),
                millis()
            );
            break;
        }
            
        case 2: // Display off - handled above
            break;
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void emergencyStop() {
    currentState = STATE_EMERGENCY_STOP;
    stopMotors();
    displayHandler.showStatus("EMERGENCY STOP", COLOR_RED);
    if (soundEnabled) {
        soundController.playErrorSound();
    }
    DEBUG_PRINTLN("!!! EMERGENCY STOP - Angle limit exceeded !!!");
}

void toggleDebugOutput() {
    debugOutputEnabled = !debugOutputEnabled;
    
    const char* msg = debugOutputEnabled ? "Debug ON (Hz drop!)" : "Debug OFF";
    displayHandler.showStatus(msg, debugOutputEnabled ? COLOR_YELLOW : COLOR_GREEN);
    DEBUG_PRINTLN(msg);
}

void toggleDisplayAlwaysOn() {
    displayAlwaysOn = !displayAlwaysOn;
    displayHandler.showStatus(displayAlwaysOn ? "Display Always ON" : "Display AUTO", COLOR_YELLOW);
    
    if (displayAlwaysOn) {
        displayHandler.setBacklight(true);
    }
}

void showSystemInfo() {
    displayHandler.clearScreen();
    displayHandler.setBacklight(true);
    
    char buffer[100];
    char timeBuffer[32];
    char heapBuffer[32];
    char totalBuffer[32];
    int y = 5;  // Start higher
    
    // Title
    displayHandler.drawHeader("SYSTEM INFO");
    
    // Smaller text
    displayHandler.getDisplay()->setTextSize(1);
    
    // Version
    displayHandler.getDisplay()->setCursor(5, y += 20);
    displayHandler.getDisplay()->print("Ver: 1.1.1 Debug");
    
    // Uptime
    displayHandler.getDisplay()->setCursor(5, y += 12);
    utilities.formatTime(millis(), timeBuffer, sizeof(timeBuffer));
    sprintf(buffer, "Up: %s", timeBuffer);
    displayHandler.getDisplay()->print(buffer);
    
    // Memory
    displayHandler.getDisplay()->setCursor(5, y += 12);
    utilities.formatBytes(ESP.getFreeHeap(), heapBuffer, sizeof(heapBuffer));
    sprintf(buffer, "Free: %s", heapBuffer);
    displayHandler.getDisplay()->print(buffer);
    
    // IMU Status
    displayHandler.getDisplay()->setCursor(5, y += 12);
    sprintf(buffer, "IMU: %.0f Hz", imuHandler.getStatus().update_rate);
    displayHandler.getDisplay()->print(buffer);
    
    // PID Values
    displayHandler.getDisplay()->setCursor(5, y += 12);
    sprintf(buffer, "PID: %.1f/%.1f/%.1f", balance.kp, balance.ki, balance.kd);
    displayHandler.getDisplay()->print(buffer);
    
    // Sound Status
    displayHandler.getDisplay()->setCursor(5, y += 12);
    sprintf(buffer, "Sound: %s", soundEnabled ? "ON" : "OFF");
    displayHandler.getDisplay()->print(buffer);
    
    // Footer
    displayHandler.drawFooter("Any button to exit", COLOR_CYAN);
    
    // Wait for button press
    delay(3000);
}

void performFactoryReset() {
    displayHandler.showStatus("FACTORY RESET!", COLOR_RED);
    
    // Clear all preferences
    Preferences prefs;
    prefs.begin("d-o-droid", false);
    prefs.clear();
    prefs.end();
    
    prefs.begin("imu_calib", false);
    prefs.clear();
    prefs.end();
    
    prefs.begin("rc_calib", false);
    prefs.clear();
    prefs.end();
    
    delay(2000);
    ESP.restart();
}

// ============================================================================
// PREFERENCES
// ============================================================================

void loadPreferences() {
    systemPrefs.begin("d-o-droid", true);
    
    // Load PID gains
    balance.kp = systemPrefs.getFloat("kp", DEFAULT_KP);
    balance.ki = systemPrefs.getFloat("ki", DEFAULT_KI);
    balance.kd = systemPrefs.getFloat("kd", DEFAULT_KD);
    
    // Load sound preference
    soundEnabled = systemPrefs.getBool("sound", true);
    
    systemPrefs.end();
    
    DEBUG_PRINTF("Loaded PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                 balance.kp, balance.ki, balance.kd);
    DEBUG_PRINTF("Sound: %s\n", soundEnabled ? "Enabled" : "Disabled");
}

void savePreferences() {
    systemPrefs.begin("d-o-droid", false);
    
    // Save PID gains
    systemPrefs.putFloat("kp", balance.kp);
    systemPrefs.putFloat("ki", balance.ki);
    systemPrefs.putFloat("kd", balance.kd);
    
    // Save sound preference
    systemPrefs.putBool("sound", soundEnabled);
    
    systemPrefs.end();
    
    DEBUG_PRINTLN("Preferences saved");
}