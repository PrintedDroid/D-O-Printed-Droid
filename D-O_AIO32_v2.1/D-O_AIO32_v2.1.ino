/**
 * D-O AIO32 v2.1 Controller — ESP32-S3 firmware
 *
 * Target board: D-O AIO32 v2.1 (Printed-Droid.com) with a plugged-in
 *               TENSTAR TS-ESP32-S3 carrier module.
 * Version:      2.1.0 (baseline aligned with Mega-sketch v3.4 conventions)
 * Date:         2026-04
 *
 * iBus-only RC (Mega-compatible 10-channel map), 6 buttons on two ADC
 * resistor ladders, 4S LiPo with optional battery monitor via solder-
 * jumpered GPIO 16.
 */

#include "config.h"
#include "imu_handler.h"
#include "sound_controller.h"
#include "display_handler.h"
#include "rc_receiver.h"
#include "utilities.h"
#include "button_handler.h"
#include "pid_autotune.h"
#include "pid_controller.h"
#include "cli.h"
#include <Preferences.h>
// ESP32Servo removed: on Arduino Core 3.x for ESP32-S3 the library drives
// servos via the legacy MCPWM path (see ESP32PWM.cpp), which hard-faults in
// attach() on this chip. Hardware probe confirmed that native LEDC on GPIO 6
// works cleanly at 50 Hz / 14-bit resolution. 16-bit @ 50 Hz is out of spec
// for Core 3.x LEDC and is refused (but does not crash — ledcAttach returns
// false). Servos are driven via ledcAttach() + ledcWrite() below.

// For watchdog and reset reason
#include "esp_system.h"
#include "esp_task_wdt.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// System preferences
Preferences systemPrefs;

// Servos are driven via native LEDC — no persistent objects required. See
// servoWrite() helper and initializeServos() below. Pin numbers come from
// config.h: SERVO_MAINBAR_PIN / SERVO_HEAD1_PIN / SERVO_HEAD2_PIN / SERVO_HEAD3_PIN.

// Balance control variables. The error-accumulator fields are gone since
// the PIDController class owns its own integral/derivative state — we
// only keep the input (targetAngle, currentAngle) and the final output
// (pidOutput) at struct level.
struct BalanceControl {
    float targetAngle  = 0.0f;
    float currentAngle = 0.0f;
    float pidOutput    = 0.0f;

    // --- Base PID gains ---
    // Used directly when ADAPTIVE_PID_ENABLED=false, and also as the target
    // that Auto-Tune writes into. When adaptive is on, the three bands below
    // override at runtime based on current throttle speed.
    float kp = DEFAULT_KP;
    float ki = DEFAULT_KI;
    float kd = DEFAULT_KD;

    // --- Adaptive PID (3 bands, ported from BBDroids / Mega v3.4) ---
    // kp/kd per band; ki stays constant (see config.h rationale).
    float kp_slow   = KP_SLOW;
    float kd_slow   = KD_SLOW;
    float kp_medium = KP_MEDIUM;
    float kd_medium = KD_MEDIUM;
    float kp_fast   = KP_FAST;
    float kd_fast   = KD_FAST;

    // Motor outputs
    int motor1Speed = 0;
    int motor2Speed = 0;
    int motor1Dir = 0;
    int motor2Dir = 0;
};

BalanceControl balance;

// Balance PID controller (ported from BBDroids via pid_controller.h/.cpp).
// Configured once in setup() / loadPreferences(), then driven every balance cycle
// by updateBalanceControl() which picks the speed-band gains and calls update().
PIDController balancePid;

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

// State-reaction feature flags (persistent — see loadPreferences).
// Seeded from config.h compile-time defaults but can be toggled at runtime.
bool stateReactionsEnabled = STATE_REACTIONS_ENABLED;
bool idleActionsEnabled    = IDLE_ACTIONS_ENABLED;
bool idleAnimationsEnabled = IDLE_ANIMATIONS_ENABLED;

// --- Driving dynamics runtime flags + values (Mega v3.4 parity) ----------
// All seeded from config.h compile-time defaults, toggle-able via CLI
// ('drive mode arcade|tank', 'drive deadband 0.05', etc.) and persisted
// in NVS so they survive reboot. See load/savePreferences.
uint8_t driveMode           = DEFAULT_DRIVE_MODE;       // 0 arcade / 1 tank
bool    rcShapingEnabled    = RC_SHAPING_ENABLED;
float   rcDeadband          = RC_DEADBAND;              // 0..1 stick normalised
float   rcExpo              = RC_EXPO;                  // 0=linear .. 1=pure x³
bool    motorRampingEnabled = MOTOR_RAMPING_ENABLED;
float   motorRampRate       = MOTOR_RAMP_RATE;          // 0..1 per tick
bool    dynamicAngleEnabled = DYNAMIC_ANGLE_ENABLED;
float   maxLeanAngle        = MAX_LEAN_ANGLE;           // degrees at full throttle

// Idle-animation state. Drives non-blocking head-servo motion when the
// droid is idle (same gate conditions as idle-sounds). When an animation
// is active, updateServos() overrides the RC-driven head targets with
// the animation's computed pose.
enum IdleAnimType {
    IDLE_ANIM_NONE = 0,
    IDLE_ANIM_NOD,          // head1 (pitch) — one sine dip
    IDLE_ANIM_LOOK_AROUND,  // head2 (yaw)   — left-center-right sweep
    IDLE_ANIM_SHAKE,        // head1 (pitch) — fast 3-cycle oscillation
    IDLE_ANIM_TILT,         // head3 (roll)  — half sine, one side
    IDLE_ANIM_COUNT
};
struct IdleAnimState {
    IdleAnimType  current        = IDLE_ANIM_NONE;
    uint32_t      startMs        = 0;
    uint32_t      durationMs     = 0;
    uint32_t      nextCheckMs    = 0;
};
IdleAnimState idleAnim;
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
uint32_t lastBatteryCheck = 0;
uint32_t systemStartTime = 0;

// Battery monitoring state
float    batteryVoltage       = 0.0f;  ///< latest measured pack voltage (V)
bool     batteryWarnActive    = false;
bool     batteryCriticalActive = false;

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
void updateBatteryMonitor();
void updateStateReactions();
void updateIdleAnimations();
void updateLEDForState();
void runBaselineCalibration();
static bool idleAnimActive();
static void computeIdleAnimTargets(int& pitchAngle, int& yawAngle, int& rollAngle);
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
void onButton5Press();       // board label "mode"
void onButton5LongPress();
void onButton6Press();       // board label "back/emerg"
void onButton6LongPress();
void cycleDriveMode();       // placeholder until Arcade/Tank flag is implemented
void cycleIMUMode();
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
static void initializeBalancePid();
static void pickAdaptiveGains(float speed, float& outKp, float& outKd);

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
    Serial.println("BOOT: ESP32-S3 Starting... [diagnostic build]");
    Serial.println("================================================");
    Serial.flush();
    delay(100);
    
    // Print reset reason IMMEDIATELY
    printResetReason();

    // NOTE: do NOT call DEBUG_BEGIN()/Serial.begin() a second time here.
    // The ESP32-S3 native USB-CDC gets re-enumerated on a second Serial.begin(),
    // which drops the USB connection and in some configurations triggers a
    // soft reset before we even reach CHECKPOINT 1. Serial is already running
    // from the unconditional Serial.begin(115200) above — nothing more to do.
    delay(500); // small settle delay
    
    // DIAGNOSTIC: unconditional markers so we see progress regardless of DEBUG_MODE
    Serial.println("[SETUP] past printResetReason, entering main init");
    Serial.flush();
    DEBUG_PRINTLN("=== D-O AIO32 v2.1 Starting ===");
    DEBUG_PRINTLN("Firmware: 2.1.0 (ESP32-S3 / TENSTAR TS-ESP32-S3)");
    DEBUG_PRINTF("Free heap at start: %d bytes\n", ESP.getFreeHeap());

    // Configure watchdog - 10 seconds timeout
    Serial.println("[CP1] watchdog config"); Serial.flush();
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,  // 10 seconds
        // Only watch the current task. Watching all IDLE tasks as well makes
        // startup fragile when third-party init code blocks briefly.
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    Serial.println("[CP1] OK"); Serial.flush();

    // Critical: Enable I2C power first!
    Serial.println("[CP2] I2C power HIGH"); Serial.flush();
    pinMode(TFT_I2C_POWER_PIN, OUTPUT);
    digitalWrite(TFT_I2C_POWER_PIN, HIGH);
    delay(100);
    Serial.println("[CP2] OK"); Serial.flush();

    // Initialize utilities (includes status LED)
    Serial.println("[CP3] utilities.begin()"); Serial.flush();
    if (utilities.begin()) {
        Serial.println("[CP3] OK"); Serial.flush();
        utilities.setLED(LED_YELLOW); // Yellow = initializing
    } else {
        Serial.println("[CP3] FAILED"); Serial.flush();
    }

    // Feed watchdog
    esp_task_wdt_reset();

    // Initialize buttons with new handler
    Serial.println("[CP4] buttonHandler.begin()"); Serial.flush();
    buttonHandler.begin();
    Serial.println("[CP4] OK (6 buttons via ADC ladder)"); Serial.flush();

    // Initialize display
    Serial.println("[CP5] displayHandler.begin()"); Serial.flush();
    if (displayHandler.begin()) {
        DEBUG_PRINTLN("CHECKPOINT 5: OK");
        displayHandler.showSplashScreen();
    } else {
        DEBUG_PRINTLN("CHECKPOINT 5: FAILED");
    }
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    // Initialize I2C
    Serial.println("[CP6] Wire.begin + setClock"); Serial.flush();
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    delay(100);
    Serial.println("[CP6] OK"); Serial.flush();

    // Initialize IMU system
    Serial.println("[CP7] imuHandler.begin()"); Serial.flush();
    IMUMode imuMode = IMU_MODE_AUTO;  // Always use AUTO mode

    if (imuHandler.begin(imuMode)) {
        Serial.println("[CP7] OK"); Serial.flush();
        char imuStatusBuffer[256];
        imuHandler.getStatusString(imuStatusBuffer, sizeof(imuStatusBuffer));
        Serial.println(imuStatusBuffer); Serial.flush();
    } else {
        Serial.println("[CP7] FAILED"); Serial.flush();
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

    // Initialize sound system (gated by SOUND_CONTROLLER_ENABLED in config.h).
    // With no DFPlayer wired up, begin() hangs inside the library handshake,
    // which looks like a bootloop from the serial monitor. Flip the flag
    // once the DFPlayer + SD card are physically connected.
    Serial.println("[CP8] soundController.begin()"); Serial.flush();
#if SOUND_CONTROLLER_ENABLED
    if (soundController.begin()) {
        Serial.println("[CP8] OK"); Serial.flush();
        soundEnabled = true;  // Enable sound by default
        soundController.playStartupSequence();
    } else {
        Serial.println("[CP8] FAILED - continuing without sound"); Serial.flush();
        soundEnabled = false;
    }
#else
    Serial.println("[CP8] SKIPPED (SOUND_CONTROLLER_ENABLED=false)"); Serial.flush();
    soundEnabled = false;
#endif

    // ORDER MATTERS ON ESP32-S3 / Arduino Core 3.x:
    // ESP32Servo allocates its LEDC timers on its own schedule and is NOT
    // aware of timers/channels grabbed by the new ledcAttach() API. When
    // ledcAttach() runs FIRST (motor init) and then servoX.attach() runs,
    // the library's timer allocation clashes with the already-bound LEDC
    // setup on ESP32-S3 and a servo.attach() call hard-faults.
    // Safe order: servos first (ESP32Servo picks its LEDC timers), THEN
    // motors (ledcAttach finds whatever is still free).
    Serial.println("[CP9] initializeServos() (moved before motors)"); Serial.flush();
    initializeServos();
    Serial.println("[CP9] OK"); Serial.flush();

    Serial.println("[CP10] initializeMotors()"); Serial.flush();
    initializeMotors();
    Serial.println("[CP10] OK"); Serial.flush();

    // Initialize RC receiver
    Serial.println("[CP11] initializeRCReceiver()"); Serial.flush();
    initializeRCReceiver();
    Serial.println("[CP11] OK"); Serial.flush();

    // Load system preferences
    Serial.println("[CP12] loadPreferences()"); Serial.flush();
    loadPreferences();
    initializeBalancePid();      // config Balance-PID with loaded gains + bounds
    Serial.println("[CP12] OK"); Serial.flush();

    // Feed watchdog
    esp_task_wdt_reset();

    // Update display
    Serial.println("[CP13] final display"); Serial.flush();
    displayHandler.clearScreen();
    displayHandler.showStatus("System Ready", COLOR_GREEN);
    Serial.println("[CP13] OK"); Serial.flush();
    
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

    // CLI ready — print banner + first prompt. Calling this last so the
    // banner sits visually after the CHECKPOINT output.
    cliBegin();
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

    // CLI: drain any pending serial input + dispatch line-terminated commands.
    // Non-blocking; safe to call every loop iteration.
    cliUpdate();

    // Handle button inputs - but not in PID menu during certain states
    if (!inPIDMenu || currentState == STATE_READY) {
        handleButtons();
    }

    // In the PID menu, the dedicated BACK button (B6) exits back to the
    // normal UI. buttonHandler.update() was already called from within
    // handleButtons() for the READY case; when we are in the menu and NOT
    // in READY, handleButtons() was skipped so update the handler here
    // explicitly before polling B6.
    if (inPIDMenu) {
        if (currentState != STATE_READY) {
            buttonHandler.update();
        }
        if (buttonHandler.getButtonEvent(6) == BUTTON_SHORT_PRESS) {
            exitPIDMenu();
        }
    }
    
    // Update display non-blocking
    displayHandler.update();
    
    // Handle different states
    switch (currentState) {
        case STATE_READY:
            // Re-enable display when back to ready
            displayHandler.setBacklight(true);
            displayDisabled = false;

            // Light-weight IMU polling in READY so Madgwick converges and the
            // telemetry/diagnostics displays have live Roll/Pitch/Yaw data.
            // Full 100 Hz is overkill here (no balance control running) — use
            // ~50 Hz which is still fast enough for Madgwick to track and plenty
            // for display update. STATE_RUNNING keeps its own tighter loop.
            {
                static uint32_t lastReadyIMUUpdate   = 0;
                static uint32_t lastReadyServoUpdate = 0;
                static uint32_t lastReadyRCUpdate    = 0;

                if (currentTime - lastReadyIMUUpdate >= 20) {  // 50 Hz
                    lastReadyIMUUpdate = currentTime;
                    updateIMU();
                }
                // Poll RC at 20 Hz so rcControl.isConnected + the
                // rcReceiver failsafe values are current.
                if (currentTime - lastReadyRCUpdate >= 50) {
                    lastReadyRCUpdate = currentTime;
                    updateRCControl();
                }
                // Drive the servos at 50 Hz even in READY so they
                // follow the sender live and fall back to neutral
                // (90 deg) when the receiver reports failsafe. Without
                // this, a remote that was connected and then went idle
                // would leave the head servos stuck at the last live
                // position instead of returning to centre.
                if (currentTime - lastReadyServoUpdate >= 20) {
                    lastReadyServoUpdate = currentTime;
                    updateServos();
                }
            }

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
                            // Apply new base gains
                            balance.kp = results.Kp;
                            balance.ki = results.Ki;
                            balance.kd = results.Kd;

                            // Scale the adaptive 3-band gains proportionally
                            // to the new base. Mega v3.4 uses a roughly-1.25x
                            // slow / 1.0x medium / 0.75x fast split — we apply
                            // the same ratio to both Kp and Kd. This keeps
                            // Auto-Tune's characteristic baseline and lets the
                            // adaptive machinery continue with comparable
                            // relative gains at different throttle speeds.
                            const float SLOW_SCALE = 1.25f;
                            const float MED_SCALE  = 1.00f;
                            const float FAST_SCALE = 0.75f;

                            balance.kp_slow   = results.Kp * SLOW_SCALE;
                            balance.kd_slow   = results.Kd * SLOW_SCALE;
                            balance.kp_medium = results.Kp * MED_SCALE;
                            balance.kd_medium = results.Kd * MED_SCALE;
                            balance.kp_fast   = results.Kp * FAST_SCALE;
                            balance.kd_fast   = results.Kd * FAST_SCALE;

                            // Save everything to preferences
                            savePreferences();

                            DEBUG_PRINTF("Auto-Tune Complete! Base: Kp=%.1f Ki=%.2f Kd=%.2f\n",
                                        balance.kp, balance.ki, balance.kd);
                            DEBUG_PRINTF("  Adaptive scaled: slow(%.1f/%.2f) med(%.1f/%.2f) fast(%.1f/%.2f)\n",
                                         balance.kp_slow,   balance.kd_slow,
                                         balance.kp_medium, balance.kd_medium,
                                         balance.kp_fast,   balance.kd_fast);
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
    
    // Battery monitor (no-op unless BATTERY_MONITOR_ENABLED = true)
    updateBatteryMonitor();

    // State-reactions (Tilt-Warning / Recovery / Idle-Sounds). Internal
    // cooldowns + probability filter — safe to call every loop.
    updateStateReactions();

    // Idle head-servo animations. Non-blocking state machine; same
    // idle-gate as the sound-reactions. updateServos() reads back the
    // animation pose via computeIdleAnimTargets().
    updateIdleAnimations();

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
        updateLEDForState();
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

// Helper: pick PID gains for the current throttle-speed band.
// speed is expected as |rcControl.throttle| in [0, 1]. Returns blended
// (kp, kd) so transitions between bands are smooth, not step-functions.
static void pickAdaptiveGains(float speed, float& outKp, float& outKd) {
#if ADAPTIVE_PID_ENABLED
    if (speed <= ADAPTIVE_THRESH_SLOW) {
        outKp = balance.kp_slow;
        outKd = balance.kd_slow;
        return;
    }
    if (speed >= ADAPTIVE_THRESH_FAST) {
        outKp = balance.kp_fast;
        outKd = balance.kd_fast;
        return;
    }
    if (speed <= ADAPTIVE_THRESH_MED) {
        // Blend slow -> medium
        float t = (speed - ADAPTIVE_THRESH_SLOW) /
                  (ADAPTIVE_THRESH_MED - ADAPTIVE_THRESH_SLOW);
        outKp = balance.kp_slow + t * (balance.kp_medium - balance.kp_slow);
        outKd = balance.kd_slow + t * (balance.kd_medium - balance.kd_slow);
    } else {
        // Blend medium -> fast
        float t = (speed - ADAPTIVE_THRESH_MED) /
                  (ADAPTIVE_THRESH_FAST - ADAPTIVE_THRESH_MED);
        outKp = balance.kp_medium + t * (balance.kp_fast - balance.kp_medium);
        outKd = balance.kd_medium + t * (balance.kd_fast - balance.kd_medium);
    }
#else
    // Non-adaptive: use the base gains directly.
    outKp = balance.kp;
    outKd = balance.kd;
#endif
}

void updateBalanceControl() {
    // ---- Adaptive gain selection based on current throttle speed ----
    float speed = fabsf(rcControl.throttle);
    float kpNow, kdNow;
    pickAdaptiveGains(speed, kpNow, kdNow);
    balancePid.setGains(kpNow, balance.ki, kdNow);

    // ---- Dynamic lean angle (Mega v3.4 feature) ----
    // When the user pushes throttle, shift the target angle slightly in the
    // direction of motion. This offloads the PID: instead of fighting the
    // droid's centre of mass at acceleration, the chassis itself already
    // leans into the move. Subtle — MAX_LEAN_ANGLE is ~3° by default.
    float effectiveTarget = balance.targetAngle;
    if (dynamicAngleEnabled) {
        effectiveTarget += rcControl.throttle * maxLeanAngle;
    }
    balancePid.setGoal(effectiveTarget);

    // ---- PID update ----
    // The PIDController computes dt internally via micros(), handles
    // I-anti-windup, control bounds, D-term low-pass filtering and
    // control-deadband in one call.
    balance.pidOutput = balancePid.update(balance.currentAngle);

    // ---- Motor mixing ----
    // Arcade: CH1 = steering, CH2 = throttle, both mixed onto both motors.
    //         Motor1 = base + throttle + steering,  Motor2 = base + throttle - steering
    // Tank:   CH1 directly -> Motor1 magnitude, CH2 directly -> Motor2 magnitude
    //         (plus the balance PID output on both so the droid stays upright)
    int baseSpeed = (int)balance.pidOutput;
    int motor1Cmd, motor2Cmd;

    if (driveMode == DRIVE_MODE_TANK) {
        // Direct per-channel mapping. rcControl.steering is CH1, .throttle is CH2.
        int cmd1 = (int)(rcControl.steering * 100.0f);
        int cmd2 = (int)(rcControl.throttle * 100.0f);
        motor1Cmd = baseSpeed + cmd1;
        motor2Cmd = baseSpeed + cmd2;
    } else {
        // Arcade (default)
        int steeringDiff = (int)(rcControl.steering * 100.0f);
        int throttleAdd  = (int)(rcControl.throttle *  50.0f);
        motor1Cmd = baseSpeed + throttleAdd + steeringDiff;
        motor2Cmd = baseSpeed + throttleAdd - steeringDiff;
    }

    // ---- Motor ramping (low-pass, Mega v3.4 feature) ----
    // Smooth transitions prevent jerky starts/stops. The filter state is
    // kept as static floats so it survives between calls. At ramp_rate=0.15
    // a step goes from 0 to 98 % in ~25 updates (~250 ms at 100 Hz main loop).
    static float motor1Filtered = 0.0f;
    static float motor2Filtered = 0.0f;

    if (motorRampingEnabled) {
        motor1Filtered += ((float)motor1Cmd - motor1Filtered) * motorRampRate;
        motor2Filtered += ((float)motor2Cmd - motor2Filtered) * motorRampRate;
        balance.motor1Speed = (int)motor1Filtered;
        balance.motor2Speed = (int)motor2Filtered;
    } else {
        balance.motor1Speed = motor1Cmd;
        balance.motor2Speed = motor2Cmd;
        // Keep the filtered state in sync so ramping engages smoothly
        // when it gets re-enabled at runtime.
        motor1Filtered = (float)motor1Cmd;
        motor2Filtered = (float)motor2Cmd;
    }

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

// Initialize the balance PID (bounds, I-limit, D-term low-pass). Call once
// after loadPreferences() so any persisted gains are in place first.
static void initializeBalancePid() {
    balancePid.reset();
    balancePid.setControlBounds(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    balancePid.setIBounds(-PID_I_BOUND, PID_I_BOUND);
    // Initial gains; the adaptive-band picker overrides these every loop.
    balancePid.setGains(balance.kp, balance.ki, balance.kd);
    balancePid.setGoal(balance.targetAngle);
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

// LEDC-based servo helper — replaces ESP32Servo::write().
//
// 50 Hz, 14-bit: period = 20000 us, 16384 counts per period.
// Standard hobby servo pulse widths:
//   500 us  = 0 deg   -> duty  409
//   1500 us = 90 deg  -> duty 1229
//   2500 us = 180 deg -> duty 2048
// 14-bit was chosen after a hardware probe: Core 3.x LEDC on ESP32-S3
// refuses 16-bit @ 50 Hz (ledcAttach returns false without crashing).
static inline void servoWrite(uint8_t pin, int angle) {
    angle = constrain(angle, 0, 180);
    uint32_t pulse_us = 500UL + (uint32_t)angle * (2500UL - 500UL) / 180UL;
    uint32_t duty     = pulse_us * 16384UL / 20000UL;  // 14-bit duty at 50 Hz
    ledcWrite(pin, duty);
}

void initializeServos() {
    Serial.println("  [CP9a] ledcAttach 4 servo pins (50 Hz, 14-bit)..."); Serial.flush();
    bool ok1 = ledcAttach(SERVO_MAINBAR_PIN, 50, 14);
    bool ok2 = ledcAttach(SERVO_HEAD1_PIN,   50, 14);
    bool ok3 = ledcAttach(SERVO_HEAD2_PIN,   50, 14);
    bool ok4 = ledcAttach(SERVO_HEAD3_PIN,   50, 14);
    Serial.printf("  [CP9a] Mainbar=%s Head1=%s Head2=%s Head3=%s\n",
                  ok1 ? "ok" : "FAIL", ok2 ? "ok" : "FAIL",
                  ok3 ? "ok" : "FAIL", ok4 ? "ok" : "FAIL");
    Serial.flush();

    Serial.println("  [CP9b] centering all servos to 90..."); Serial.flush();
    servoWrite(SERVO_MAINBAR_PIN, 90);
    servoWrite(SERVO_HEAD1_PIN,   90);
    servoWrite(SERVO_HEAD2_PIN,   90);
    servoWrite(SERVO_HEAD3_PIN,   90);
    Serial.println("  [CP9b] OK"); Serial.flush();
}

void updateServos() {
    // Mainbar from CH3 (Mega-compatible: user controls this, not PID)
    float mainbarNorm = rcReceiver.getMainbar();                // -1..+1
    int mainbarAngle = 90 + (int)(mainbarNorm * 90.0f);         // 0..180
    servoWrite(SERVO_MAINBAR_PIN, mainbarAngle);

    // Head servos: CH4 pitch, CH5 yaw, CH6 roll
    // Source directly from rcReceiver so CH4/CH5 also enter the receiver's
    // failsafe position on iBus signal loss. Using rcControl.* here froze the
    // last live values because that struct is only refreshed on new packets.
    int headPitchAngle = 90 + (int)(rcReceiver.getHead1() * 45);  // CH4
    int headYawAngle   = 90 + (int)(rcReceiver.getHead2() * 45);  // CH5
    int headRollAngle  = 90 + (int)(rcReceiver.getHead3() * 45);  // CH6

    // Idle-Animation override: when an idle head-animation is running,
    // its sinusoidal targets replace the RC-driven pose for the duration
    // of the animation. Animation is non-blocking and self-terminates in
    // updateIdleAnimations() after its durationMs elapses.
    if (idleAnimActive()) {
        computeIdleAnimTargets(headPitchAngle, headYawAngle, headRollAngle);
    }

    servoWrite(SERVO_HEAD1_PIN, headPitchAngle);
    servoWrite(SERVO_HEAD2_PIN, headYawAngle);
    servoWrite(SERVO_HEAD3_PIN, headRollAngle);
}

// ============================================================================
// RC RECEIVER - WITH DEBUG
// ============================================================================

void initializeRCReceiver() {
    // Load the stored protocol from NVS — defaults to iBus on first boot.
    // Switch via CLI `rc protocol <ibus|sbus>` then `save`.
    RCProtocol storedProto = RCReceiver::loadStoredProtocol();
    DEBUG_PRINTF("  RC init: Starting receiver (protocol=%s)...\n",
                 RCReceiver::protocolName(storedProto));

    if (!rcReceiver.begin(storedProto)) {
        DEBUG_PRINTLN("  RC init: FAILED - continuing without RC");
    } else {
        DEBUG_PRINTLN("  RC init: Success");
    }
}

// ============================================================================
// BATTERY MONITOR (4S LiPo via divider on BATTERY_PIN)
// ============================================================================
//
// Off by default. Enable by setting BATTERY_MONITOR_ENABLED to true in
// config.h once the AIO32 v2.1 solder-jumper (IO36 -> IO16) is populated.

void updateBatteryMonitor() {
    if (!BATTERY_MONITOR_ENABLED) return;

    uint32_t now = millis();
    if (now - lastBatteryCheck < 500) return;  // 2 Hz is plenty
    lastBatteryCheck = now;

    // ESP32-S3 ADC: 12-bit by default, 3.3 V reference with 11 dB attenuation.
    // Convert raw 0..4095 to pin voltage, then apply divider factor.
    int raw = analogRead(BATTERY_PIN);
    float pinVolts = (float)raw * (3.3f / 4095.0f);
    batteryVoltage = pinVolts * BATTERY_DIVIDER_FACTOR;

    // Critical-battery action is LATCHED by design: once the pack dips below
    // BATTERY_CRITICAL_V the motors stop and the state goes to EMERGENCY_STOP
    // and stays there until a power-cycle. Rationale: a sag that deep indicates
    // cell damage, and a "recovering" pack under no-load will sag again as soon
    // as motors draw current — we don't want to bounce in and out of safety.
    // If an opt-in recovery mode is ever needed, mirror the Mega v3.4
    // `battery_recovery` config flag.
    if (batteryVoltage < BATTERY_CRITICAL_V && !batteryCriticalActive) {
        batteryCriticalActive = true;
        DEBUG_PRINTF("[batt] CRITICAL: %.2f V  -> stopping motors (latched)\n", batteryVoltage);
        if (soundEnabled) soundController.playSound(SOUND_LOW_BATTERY);
        stopMotors();
        currentState = STATE_EMERGENCY_STOP;
    }

    if (batteryVoltage < BATTERY_WARN_V && !batteryCriticalActive) {
        if (!batteryWarnActive) {
            batteryWarnActive = true;
            DEBUG_PRINTF("[batt] WARNING: %.2f V\n", batteryVoltage);
            if (soundEnabled) soundController.playSound(SOUND_LOW_BATTERY);
        }
    } else if (batteryVoltage > BATTERY_WARN_V + 0.2f) {
        batteryWarnActive = false;
    }
}

// ============================================================================
// STATE REACTIONS — Tilt-Warning + Recovery + Idle-Sounds (Mega v3.4 parity)
// ============================================================================
//
// Tilt-Warning: triggers SOUND_TILT_WARNING once when |pitch| exceeds
//   TILT_WARN_ANGLE; cools down for TILT_COOLDOWN_MS before re-arming.
//   When pitch recovers below TILT_RECOVER_ANGLE, plays SOUND_RECOVERY
//   and re-arms.
// Idle-Sounds: triggers a random track from SOUND_IDLE_START..END when
//   RC is connected but sticks have been still for IDLE_DETECT_MS.
//   Signal-Gate: idle sounds are suppressed when RC is disconnected, so
//   the droid stays quiet when the transmitter is off.
void updateStateReactions() {
    uint32_t now = millis();

    // ---- Tilt-Warning state machine --------------------------------------
    // Requires only that at least one IMU is initialised — not the stricter
    // isHealthy() (which was filtering out 50 Hz jitter in READY and
    // intermittently suppressing the warn). Tilt is a safety feature; we
    // want it to fire as long as we have ANY angle reading.
    IMUStatus _imuS = imuHandler.getStatus();
    bool imuAvailable = _imuS.qmi_initialized || _imuS.lsm_initialized;
    if (stateReactionsEnabled && imuAvailable) {
        static bool     tiltWarned      = false;
        static uint32_t lastTiltWarn    = 0;

        float absPitch = fabsf(balance.currentAngle);

        if (!tiltWarned &&
            absPitch > TILT_WARN_ANGLE &&
            (now - lastTiltWarn) > TILT_COOLDOWN_MS) {
            // Enter warned state
            tiltWarned = true;
            lastTiltWarn = now;
            if (soundEnabled) {
                soundController.playSound(SOUND_TILT_WARNING);
            }
            DEBUG_PRINTF("[state] Tilt warning @ %.1f deg\n", balance.currentAngle);
        } else if (tiltWarned && absPitch < TILT_RECOVER_ANGLE) {
            // Recovered — reset cooldown too, otherwise a second tilt
            // within TILT_COOLDOWN_MS after recovery would be silently
            // swallowed by the anti-spam gate. The hysteresis between
            // TILT_WARN_ANGLE (15°) and TILT_RECOVER_ANGLE (5°) already
            // prevents flicker-spam around the threshold.
            tiltWarned = false;
            lastTiltWarn = 0;
            if (soundEnabled) {
                soundController.playSound(SOUND_RECOVERY);
            }
            DEBUG_PRINTLN("[state] Tilt recovered");
        }
    }

    // ---- Idle-Sound trigger ---------------------------------------------
    if (idleActionsEnabled) {
        static uint32_t lastStickActivity = 0;
        static uint32_t nextIdleCheck     = 0;
        static bool     idleInitialized   = false;

        if (!idleInitialized) {
            lastStickActivity = now;
            nextIdleCheck     = now + IDLE_INTERVAL_MIN_MS;
            idleInitialized   = true;
        }

        // Detect "stick still" — any meaningful RC axis movement resets the timer.
        bool stickActive =
            fabsf(rcControl.steering) > IDLE_STICK_DEADZONE ||
            fabsf(rcControl.throttle) > IDLE_STICK_DEADZONE ||
            fabsf(rcControl.headTilt) > IDLE_STICK_DEADZONE ||
            fabsf(rcControl.headPan ) > IDLE_STICK_DEADZONE;
        if (stickActive) {
            lastStickActivity = now;
        }

        // Signal-Gate: only roll idle dice when RC is actually connected.
        // Otherwise the droid would chatter indefinitely with the TX off.
        // Also suppress while a tilt event is active so the two don't talk
        // over each other.
        bool idleEligible = rcControl.isConnected &&
                            (now - lastStickActivity) > IDLE_DETECT_MS &&
                            fabsf(balance.currentAngle) <= TILT_WARN_ANGLE &&
                            soundEnabled;

        if (idleEligible && now >= nextIdleCheck) {
            if ((int)random(0, 100) < IDLE_PROBABILITY_PCT) {
                uint8_t track = (uint8_t)random(SOUND_IDLE_START, SOUND_IDLE_END + 1);
                soundController.playSound(track);
                DEBUG_PRINTF("[state] Idle sound track %u\n", track);
            }
            // Schedule next dice-roll with randomized interval
            uint32_t interval = (uint32_t)random(IDLE_INTERVAL_MIN_MS, IDLE_INTERVAL_MAX_MS + 1);
            nextIdleCheck = now + interval;
        }
    }
}

// ============================================================================
// IDLE-SERVO-ANIMATIONEN — Head-Bewegungen bei Stillstand (Mega v3.4 parity)
// ============================================================================
//
// Non-blocking state machine. Gate identisch zu updateStateReactions()
// (RC connected + sticks still + kein tilt + master flag true). Wenn eine
// Animation aktiv ist, setzt computeIdleAnimTargets() die Head-Servo-Winkel
// aus einer sinus-basierten Zeitfunktion — das wird später in updateServos()
// als Override auf die RC-gesteuerten Head-Werte angewandt.

static bool idleAnimActive() {
    return idleAnim.current != IDLE_ANIM_NONE;
}

// Smooth easing: returns sin(t*pi) in [0,1] shape (0 at t=0, peak at 0.5, 0 at t=1)
static float halfSine(float t) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return sinf(t * (float)M_PI);
}
// Full sine (0→+1→0→-1→0 over t=0..1), good for nod-down-and-back
static float fullSine(float t) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return sinf(t * 2.0f * (float)M_PI);
}
// Fast oscillation: 3 full cycles across t=0..1
static float fastSine(float t, int cycles = 3) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return sinf(t * 2.0f * (float)M_PI * (float)cycles);
}

static void computeIdleAnimTargets(int& pitchAngle, int& yawAngle, int& rollAngle) {
    if (idleAnim.current == IDLE_ANIM_NONE) return;
    uint32_t now = millis();
    uint32_t elapsed = now - idleAnim.startMs;
    float t = (idleAnim.durationMs > 0)
                ? (float)elapsed / (float)idleAnim.durationMs
                : 1.0f;
    if (t > 1.0f) t = 1.0f;

    switch (idleAnim.current) {
        case IDLE_ANIM_NOD: {
            // Head nickt einmal: 90 + sine dip nach unten und zurück
            // (negatives sin: erst runter, dann rauf, dann zurück)
            float offset = -IDLE_ANIM_NOD_DEG * halfSine(t);
            pitchAngle = 90 + (int)offset;
        } break;

        case IDLE_ANIM_LOOK_AROUND: {
            // Yaw schaut erst nach rechts, dann zurück, dann nach links, zurück
            float offset = IDLE_ANIM_LOOK_DEG * fullSine(t);
            yawAngle = 90 + (int)offset;
        } break;

        case IDLE_ANIM_SHAKE: {
            // Schnelle Pitch-Oszillation, Amplitude klingt ab via halfSine envelope
            float envelope = halfSine(t);
            float wobble   = fastSine(t, 3);
            pitchAngle = 90 + (int)(IDLE_ANIM_SHAKE_DEG * envelope * wobble);
        } break;

        case IDLE_ANIM_TILT: {
            // Roll neigt einmal zur Seite und zurück
            float offset = IDLE_ANIM_TILT_DEG * halfSine(t);
            rollAngle = 90 + (int)offset;
        } break;

        case IDLE_ANIM_NONE:
        case IDLE_ANIM_COUNT:
            break;
    }

    // Clamp
    pitchAngle = constrain(pitchAngle, 0, 180);
    yawAngle   = constrain(yawAngle,   0, 180);
    rollAngle  = constrain(rollAngle,  0, 180);
}

void updateIdleAnimations() {
    uint32_t now = millis();

    // Animation in flight? Let it finish naturally.
    if (idleAnimActive()) {
        if (now - idleAnim.startMs >= idleAnim.durationMs) {
            DEBUG_PRINTF("[idle-anim] done (%d)\n", (int)idleAnim.current);
            idleAnim.current = IDLE_ANIM_NONE;
        }
        return;
    }

    if (!idleAnimationsEnabled) return;

    // Gate: same idle-detection as sound-reactions
    // Using the idle-stick-activity tracker kept locally — avoid touching
    // the sound side's static. updateStateReactions() uses its own
    // `lastStickActivity`; we replicate the gate here so the animation
    // timing is independent and deterministic.
    static uint32_t lastStickActivity = 0;
    static bool     initialized       = false;
    if (!initialized) {
        lastStickActivity = now;
        idleAnim.nextCheckMs = now + IDLE_ANIM_INTERVAL_MIN_MS;
        initialized = true;
    }

    bool stickActive =
        fabsf(rcControl.steering) > IDLE_STICK_DEADZONE ||
        fabsf(rcControl.throttle) > IDLE_STICK_DEADZONE ||
        fabsf(rcControl.headTilt) > IDLE_STICK_DEADZONE ||
        fabsf(rcControl.headPan ) > IDLE_STICK_DEADZONE;
    if (stickActive) {
        lastStickActivity = now;
    }

    bool gateOk = rcControl.isConnected &&
                  (now - lastStickActivity) > IDLE_DETECT_MS &&
                  fabsf(balance.currentAngle) <= TILT_WARN_ANGLE;

    if (!gateOk || now < idleAnim.nextCheckMs) return;

    // Dice roll
    if ((int)random(0, 100) < IDLE_ANIM_PROBABILITY_PCT) {
        IdleAnimType pick = (IdleAnimType)random(1, IDLE_ANIM_COUNT);  // skip NONE
        idleAnim.current = pick;
        idleAnim.startMs = now;
        switch (pick) {
            case IDLE_ANIM_NOD:         idleAnim.durationMs = IDLE_ANIM_NOD_MS;   break;
            case IDLE_ANIM_LOOK_AROUND: idleAnim.durationMs = IDLE_ANIM_LOOK_MS;  break;
            case IDLE_ANIM_SHAKE:       idleAnim.durationMs = IDLE_ANIM_SHAKE_MS; break;
            case IDLE_ANIM_TILT:        idleAnim.durationMs = IDLE_ANIM_TILT_MS;  break;
            default:                    idleAnim.durationMs = 2000;               break;
        }
        DEBUG_PRINTF("[idle-anim] start type=%d duration=%lu\n",
                     (int)pick, (unsigned long)idleAnim.durationMs);
    }

    // Schedule next dice-roll regardless of hit/miss
    uint32_t interval = (uint32_t)random(IDLE_ANIM_INTERVAL_MIN_MS, IDLE_ANIM_INTERVAL_MAX_MS + 1);
    idleAnim.nextCheckMs = now + interval;
}

// Deadband + expo shaping for a single stick axis in [-1, +1].
// Deadband (Totzone): maps |x| < rcDeadband to 0, rescales the rest to
//                     use the full [0, 1] range above the zone.
// Expo:               y = (1 - e)*x + e*x³  — more resolution near centre,
//                     more aggressive near full stick. Applied AFTER
//                     deadband so the curve operates on the normalised
//                     post-deadband signal.
// Disabled when rcShapingEnabled is false (passes x through unchanged).
static float applyRCShaping(float x) {
    if (!rcShapingEnabled) return x;
    float sign  = (x >= 0.0f) ? 1.0f : -1.0f;
    float absx  = fabsf(x);
    if (absx <= rcDeadband) return 0.0f;
    float scaled = (absx - rcDeadband) / (1.0f - rcDeadband);
    if (scaled > 1.0f) scaled = 1.0f;
    float shaped = (1.0f - rcExpo) * scaled + rcExpo * scaled * scaled * scaled;
    return sign * shaped;
}

// ============================================================================
// STATUS-LED — Onboard NeoPixel (TENSTAR, GPIO 33)
// ============================================================================
//
// Maps the high-level SystemState + battery flags onto a single NeoPixel
// color/pattern. Called every ~500 ms from the main loop. Uses internal
// change-detection so pulse/blink animations are not reset every call
// (which would cause visible flicker).
//
// Priority:
//   1. batteryCriticalActive  -> fast red blink  (overrides all states)
//   2. STATE_ERROR            -> solid red
//   3. STATE_EMERGENCY_STOP   -> slow red blink
//   4. STATE_CALIBRATING      -> cyan pulse
//   5. STATE_RUNNING          -> blue pulse
//   6. STATE_READY + battWarn -> orange pulse (low-battery)
//   7. STATE_READY            -> solid green
//   8. STATE_INIT             -> solid yellow
void updateLEDForState() {
    static SystemState lastState = (SystemState)-1;
    static bool        lastBattWarn = false;
    static bool        lastBattCrit = false;

    if (currentState     == lastState    &&
        batteryWarnActive == lastBattWarn &&
        batteryCriticalActive == lastBattCrit) {
        return;  // no change — leave the running pattern alone
    }
    lastState    = currentState;
    lastBattWarn = batteryWarnActive;
    lastBattCrit = batteryCriticalActive;

    // 1. Critical battery wins over everything
    if (batteryCriticalActive) {
        utilities.blinkLED(LED_RED, 200);   // fast blink
        return;
    }

    // 2..8. State-driven
    switch (currentState) {
        case STATE_ERROR:
            utilities.setLED(LED_RED);
            break;
        case STATE_EMERGENCY_STOP:
            utilities.blinkLED(LED_RED, 400);
            break;
        case STATE_CALIBRATING:
            utilities.pulseLED(LED_CYAN, 1200);
            break;
        case STATE_RUNNING:
            utilities.pulseLED(LED_BLUE, 2000);
            break;
        case STATE_READY:
            if (batteryWarnActive) {
                utilities.pulseLED(LED_ORANGE, 1500);
            } else {
                utilities.setLED(LED_GREEN);
            }
            break;
        case STATE_INIT:
        default:
            utilities.setLED(LED_YELLOW);
            break;
    }
}

// ============================================================================
// AUTO-BASELINE-KALIBRIERUNG — Button-Long-Press auf B1 (start/stop).
// ============================================================================
//
// Droid still hinstellen, B1 lang drücken. Der Sketch misst 2 Sekunden
// lang den Pitch-Winkel, bildet den Mittelwert und schreibt den als
// neuen balance.targetAngle in NVS (Key "targetAngle"). Der Balance-PID
// regelt ab dann auf genau diese Nulllage ein.
//
// Abbruch-Gate: wenn der Min/Max-Range über die 2 Sekunden > 5° wird,
// bricht die Funktion ab und speichert NICHT — verhindert eine kaputte
// Baseline wenn man den Droiden während der Messung wackelt oder wenn
// er sich noch bewegt.
void runBaselineCalibration() {
    if (currentState == STATE_RUNNING) {
        displayHandler.showStatus("Stop balance first", COLOR_YELLOW, 2000);
        return;
    }

    DEBUG_PRINTLN("[baseline] start — hold still for ~2 s");
    displayHandler.showStatus("Calibrating baseline...", COLOR_CYAN, 3000);
    utilities.pulseLED(LED_CYAN, 400);

    const uint32_t DURATION_MS         = 2000;
    const uint32_t SAMPLE_INTERVAL_MS  = 20;     // 50 Hz
    const float    MAX_RANGE_DEG       = 5.0f;

    float    sumPitch    = 0.0f;
    uint32_t sampleCount = 0;
    float    minPitch    =  180.0f;
    float    maxPitch    = -180.0f;
    bool     aborted     = false;

    uint32_t startMs      = millis();
    uint32_t nextSampleMs = startMs;

    while (millis() - startMs < DURATION_MS) {
        esp_task_wdt_reset();
        if (millis() >= nextSampleMs) {
            updateIMU();                         // fresh pitch reading
            float p = balance.currentAngle;
            sumPitch += p;
            sampleCount++;
            if (p < minPitch) minPitch = p;
            if (p > maxPitch) maxPitch = p;
            if ((maxPitch - minPitch) > MAX_RANGE_DEG && sampleCount > 5) {
                aborted = true;
                break;
            }
            nextSampleMs += SAMPLE_INTERVAL_MS;
        }
        delay(2);
    }

    if (aborted || sampleCount < 10) {
        char msg[48];
        snprintf(msg, sizeof(msg), "Baseline aborted (range %.1f deg)", maxPitch - minPitch);
        DEBUG_PRINTF("[baseline] %s (n=%lu)\n", msg, (unsigned long)sampleCount);
        displayHandler.showStatus(msg, COLOR_RED, 3000);
        return;
    }

    float mean = sumPitch / (float)sampleCount;
    balance.targetAngle = mean;
    balancePid.setGoal(mean);

    // Persist so the next boot starts from the same baseline
    systemPrefs.begin("d-o-droid", false);
    systemPrefs.putFloat("tgtAngle", mean);
    systemPrefs.end();

    char msg[48];
    snprintf(msg, sizeof(msg), "Baseline: %.2f deg", mean);
    DEBUG_PRINTF("[baseline] %s  (%lu samples, range %.2f deg)\n",
                 msg, (unsigned long)sampleCount, maxPitch - minPitch);
    displayHandler.showStatus(msg, COLOR_GREEN, 3000);
}

void updateRCControl() {
    // Drain the iBus receiver. newData == true means a fresh packet landed;
    // false means no packet since the last call (could be mid-gap or truly
    // disconnected). The rcReceiver itself switches to failsafe (neutral)
    // values on packet loss, so we copy its getters unconditionally — that
    // way a signal loss immediately drives the drive axes and head-servo
    // targets back to neutral instead of freezing on the last live value.
    // That was the safety bug fixed here (Codex finding P1): previously
    // the value-copy block sat inside the newData branch and a receiver
    // timeout left motor-mixing happily consuming the last stick command.
    bool     newData = rcReceiver.update();
    uint32_t now     = millis();

    if (newData) {
        rcControl.lastUpdate = now;
    }

    // ALWAYS mirror receiver state into rcControl (failsafe-inclusive).
    //   Drive axes with shaping. Head axes stay linear (fine-angle control
    //   matters more than expo for look-around). CH3 mainbar + CH6 head-roll
    //   are handled directly in updateServos() from the receiver.
    rcControl.steering = applyRCShaping(rcReceiver.getSteering());  // CH1
    rcControl.throttle = applyRCShaping(rcReceiver.getThrottle());  // CH2
    rcControl.headTilt = rcReceiver.getHead1();                     // CH4
    rcControl.headPan  = rcReceiver.getHead2();                     // CH5
    rcControl.mute     = rcReceiver.getMute();                      // CH7

    // isConnected: both the receiver lib's own flag AND our millis-timeout
    // backup must agree. Defends against libs that forget to lower their
    // own flag on rare failure modes.
    bool libConnected = rcReceiver.isConnected();
    bool stale        = (now - rcControl.lastUpdate) > 1000;
    rcControl.isConnected = libConnected && !stale;

    // Sound-mood trigger: only on fresh packets + unmuted + sound enabled.
    // Deliberately gated behind newData — we don't want a timeout-induced
    // "mood change" to fire a random negative/positive sample.
    if (newData && !rcControl.mute && soundEnabled) {
        soundController.setMute(false);
        uint8_t mood = rcReceiver.getMood();                    // CH9
        if (mood != rcControl.soundTrigger) {
            rcControl.soundTrigger = mood;
            switch (mood) {
                case 0: soundController.playRandomFromCategory(SOUND_CAT_NEGATIVE); break;
                case 2: soundController.playRandomFromCategory(SOUND_CAT_POSITIVE); break;
                default: /* mid = silent */                                         break;
            }
        }
        // CH8 mode (greet) and CH10 squeak could be wired up here later.
    } else if (rcControl.mute) {
        soundController.setMute(true);
    }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handleButtons() {
    // Update button handler
    buttonHandler.update();

    // Get button events. Button numbers correspond to PCB silkscreen labels:
    //   B1 start/stop   B2 calibrate    B3 display
    //   B4 debug/select B5 mode         B6 back/emerg
    ButtonEvent btn1 = buttonHandler.getButtonEvent(1);
    ButtonEvent btn2 = buttonHandler.getButtonEvent(2);
    ButtonEvent btn3 = buttonHandler.getButtonEvent(3);
    ButtonEvent btn4 = buttonHandler.getButtonEvent(4);
    ButtonEvent btn5 = buttonHandler.getButtonEvent(5);
    ButtonEvent btn6 = buttonHandler.getButtonEvent(6);

    // Unconditional serial trace — any button event prints its id + event
    // type so the user can see exactly which physical button the ADC
    // ladder decoded. Useful for wiring checks, button-label sanity checks,
    // and debugging missing-press reports.
    auto traceBtn = [](uint8_t id, ButtonEvent ev) {
        if (ev == BUTTON_NONE) return;
        const char* evStr = "?";
        switch (ev) {
            case BUTTON_SHORT_PRESS:  evStr = "short"; break;
            case BUTTON_LONG_PRESS:   evStr = "long";  break;
            case BUTTON_DOUBLE_CLICK: evStr = "double"; break;
            case BUTTON_HOLD:         evStr = "hold";  break;
            default: break;
        }
        Serial.printf("[btn] B%u %s\n", id, evStr);
    };
    traceBtn(1, btn1); traceBtn(2, btn2); traceBtn(3, btn3);
    traceBtn(4, btn4); traceBtn(5, btn5); traceBtn(6, btn6);

    // Check button combinations first
    handleButtonCombos();

    // Handle individual buttons only if not in combo (indices are 1-based)
    if (!buttonHandler.isComboPressed(1, 2) &&
        !buttonHandler.isComboPressed(3, 4) &&
        !buttonHandler.isComboPressed(2, 3)) {

        // Button 1 (label: start/stop):
        //   short -> toggle balance / clear emergency
        //   long  -> Auto-Baseline calibration (droid must be still)
        if (btn1 == BUTTON_SHORT_PRESS) {
            onButton1Press();
        } else if (btn1 == BUTTON_LONG_PRESS) {
            runBaselineCalibration();
        }

        // Button 2: Calibrate (short) / PID Menu (long)
        if (btn2 == BUTTON_SHORT_PRESS) {
            onButton2Press();
        } else if (btn2 == BUTTON_LONG_PRESS) {
            enterPIDMenu();
        }

        // Button 3: Display control (cycle mode / toggle always-on)
        if (btn3 == BUTTON_SHORT_PRESS) {
            onButton3Press();
        } else if (btn3 == BUTTON_LONG_PRESS) {
            toggleDisplayAlwaysOn();
        }

        // Button 4: Debug output (short) / System info (long)
        // Inside PID menu, B4-short is also used to adjust the selected value.
        if (btn4 == BUTTON_SHORT_PRESS) {
            onButton4Press();
        } else if (btn4 == BUTTON_LONG_PRESS) {
            showSystemInfo();
        }

        // Button 5 (label: mode): cycle drive-mode (short) / IMU-mode (long)
        if (btn5 == BUTTON_SHORT_PRESS) {
            onButton5Press();
        } else if (btn5 == BUTTON_LONG_PRESS) {
            onButton5LongPress();
        }

        // Button 6 (label: back/emerg): menu-back (short) / Emergency Stop (long)
        if (btn6 == BUTTON_SHORT_PRESS) {
            onButton6Press();
        } else if (btn6 == BUTTON_LONG_PRESS) {
            onButton6LongPress();
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

// ---------------------------------------------------------------------------
// Button 5 — board label "mode"
// Short: cycle the drive-mode (Arcade / Tank). Placeholder until the
//        mixing-mode flag is ported from the Mega v3.4 sketch.
// Long : cycle the IMU mode (QMI_ONLY / LSM_ONLY / FUSION).
// ---------------------------------------------------------------------------
void onButton5Press() {
    DEBUG_PRINTLN("Button 5 (mode) short press — cycleDriveMode()");
    cycleDriveMode();
}

void onButton5LongPress() {
    DEBUG_PRINTLN("Button 5 (mode) long press — cycleIMUMode()");
    cycleIMUMode();
}

// ---------------------------------------------------------------------------
// Button 6 — board label "back/emerg"
// Short: generic "back" in menus. Currently only the PID menu exists; its
//        exit is driven from loop() (see the `if (inPIDMenu)` block). If
//        invoked outside any menu, we fall back to showing the system info
//        page as a gentle "you pressed back but there is nothing to back
//        out of" acknowledgement. Wire additional menus into this pathway
//        as they appear.
// Long : Emergency Stop — moved from B1-long so B6's "emerg" silkscreen
//        label matches its behaviour.
// ---------------------------------------------------------------------------
void onButton6Press() {
    DEBUG_PRINTLN("Button 6 (back) short press");
    if (inPIDMenu) {
        // Defensive: loop() already handles this, but covers any path
        // where handleButtons() also runs while the menu is active.
        exitPIDMenu();
    } else {
        // No active menu — treat as a generic "show me where I am"
        showSystemInfo();
    }
}

void onButton6LongPress() {
    DEBUG_PRINTLN("Button 6 (emerg) long press — Emergency Stop");
    emergencyStop();
}

// ---------------------------------------------------------------------------
// Mode cycles — invoked by B5
// ---------------------------------------------------------------------------

// Cycle the TFT rotation 0 -> 1 -> 2 -> 3 -> 0... and persist the choice
// in NVS so the next boot comes up in the same orientation.
// Board labels for B5 are "mode" — display orientation is a display mode,
// so we repurpose this slot until Arcade/Tank drive-mode flag lands.
// (When the drive-mode feature is ported, move it to another button or
// to the CLI menu; the user asked for CLI access to rotation as well.)
void cycleDriveMode() {
    static uint8_t rot = TFT_ROTATION;   // seeded at first call
    // Seed from saved preference on first invocation so cycle starts at the
    // current live rotation, not the compile-time default.
    static bool seeded = false;
    if (!seeded) {
        systemPrefs.begin("d-o-droid", true);
        rot = systemPrefs.getUChar("tftRot", (uint8_t)TFT_ROTATION) & 0x03;
        systemPrefs.end();
        seeded = true;
    }

    rot = (rot + 1) & 0x03;
    displayHandler.setRotation(rot);

    // Persist so the next boot honors the user's choice
    systemPrefs.begin("d-o-droid", false);
    systemPrefs.putUChar("tftRot", rot);
    systemPrefs.end();

    char buf[24];
    snprintf(buf, sizeof(buf), "TFT rotation: %u", rot);
    DEBUG_PRINTLN(buf);
    displayHandler.showStatus(buf, COLOR_CYAN, 1500);
}

// Cycle QMI_ONLY -> LSM_ONLY -> FUSION, skipping any mode whose sensor
// isn't currently initialised. Applied live via IMUHandler::setMode().
void cycleIMUMode() {
    IMUStatus s = imuHandler.getStatus();
    IMUMode  current = imuHandler.getMode();
    IMUMode  next    = current;

    // Build a small ordered list of usable modes so we rotate through
    // only the ones the hardware actually supports right now.
    IMUMode candidates[3];
    uint8_t nCand = 0;
    if (s.qmi_initialized) candidates[nCand++] = IMU_MODE_QMI_ONLY;
    if (s.lsm_initialized) candidates[nCand++] = IMU_MODE_LSM_ONLY;
    if (s.qmi_initialized && s.lsm_initialized) candidates[nCand++] = IMU_MODE_FUSION;

    if (nCand == 0) {
        DEBUG_PRINTLN("cycleIMUMode: no IMUs initialised, nothing to cycle");
        displayHandler.showStatus("No IMU available", COLOR_RED);
        return;
    }

    // Find current in the candidate list and step to the next entry
    uint8_t idx = 0;
    for (uint8_t i = 0; i < nCand; ++i) {
        if (candidates[i] == current) { idx = i; break; }
    }
    next = candidates[(idx + 1) % nCand];

    if (imuHandler.setMode(next)) {
        const char* label = "?";
        switch (next) {
            case IMU_MODE_QMI_ONLY: label = "IMU: QMI only"; break;
            case IMU_MODE_LSM_ONLY: label = "IMU: LSM only"; break;
            case IMU_MODE_FUSION:   label = "IMU: Fusion";   break;
            default: break;
        }
        DEBUG_PRINTF("cycleIMUMode: %s\n", label);
        displayHandler.showStatus(label, COLOR_GREEN);
    } else {
        DEBUG_PRINTLN("cycleIMUMode: setMode() returned false");
        displayHandler.showStatus("IMU mode: failed", COLOR_RED);
    }
}

void handleButtonCombos() {
    static uint32_t comboStart = 0;
    static uint32_t factoryStart = 0;
    static bool soundComboHandled = false;
    
    // B1+B2: System Reset (1-based indices)
    if (buttonHandler.isComboPressed(1, 2)) {
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
    if (buttonHandler.isComboPressed(3, 4)) {
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
    if (buttonHandler.isComboPressed(2, 3)) {
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

    // Base PID gains (used directly when ADAPTIVE_PID_ENABLED=false)
    balance.kp = systemPrefs.getFloat("kp", DEFAULT_KP);
    balance.ki = systemPrefs.getFloat("ki", DEFAULT_KI);
    balance.kd = systemPrefs.getFloat("kd", DEFAULT_KD);

    // Baseline target angle (Auto-Baseline-Kalibrierung, persistent)
    balance.targetAngle = systemPrefs.getFloat("tgtAngle", 0.0f);

    // Adaptive PID 3-band gains (used when ADAPTIVE_PID_ENABLED=true).
    // Fall back to config.h compile-time defaults on first boot / after
    // factory reset so the droid always has a sane set.
    balance.kp_slow   = systemPrefs.getFloat("kp_slow",   KP_SLOW);
    balance.kd_slow   = systemPrefs.getFloat("kd_slow",   KD_SLOW);
    balance.kp_medium = systemPrefs.getFloat("kp_med",    KP_MEDIUM);
    balance.kd_medium = systemPrefs.getFloat("kd_med",    KD_MEDIUM);
    balance.kp_fast   = systemPrefs.getFloat("kp_fast",   KP_FAST);
    balance.kd_fast   = systemPrefs.getFloat("kd_fast",   KD_FAST);

    // Sound preference
    soundEnabled = systemPrefs.getBool("sound", true);

    // State-reaction toggles
    stateReactionsEnabled = systemPrefs.getBool("stateRx", STATE_REACTIONS_ENABLED);
    idleActionsEnabled    = systemPrefs.getBool("idleRx",  IDLE_ACTIONS_ENABLED);
    idleAnimationsEnabled = systemPrefs.getBool("idleAnim", IDLE_ANIMATIONS_ENABLED);

    // Driving-dynamics (Mega v3.4 parity)
    driveMode           = systemPrefs.getUChar("drvMode",  (uint8_t)DEFAULT_DRIVE_MODE);
    rcShapingEnabled    = systemPrefs.getBool ("drvShape", RC_SHAPING_ENABLED);
    rcDeadband          = systemPrefs.getFloat("drvDb",    RC_DEADBAND);
    rcExpo              = systemPrefs.getFloat("drvExpo",  RC_EXPO);
    motorRampingEnabled = systemPrefs.getBool ("drvRampOn", MOTOR_RAMPING_ENABLED);
    motorRampRate       = systemPrefs.getFloat("drvRamp",  MOTOR_RAMP_RATE);
    dynamicAngleEnabled = systemPrefs.getBool ("drvLeanOn", DYNAMIC_ANGLE_ENABLED);
    maxLeanAngle        = systemPrefs.getFloat("drvLeanMx", MAX_LEAN_ANGLE);

    // Saved TFT rotation (0..3). Defaults to the compile-time TFT_ROTATION
    // if nothing saved yet. Applied below via displayHandler.setRotation().
    uint8_t savedRotation = systemPrefs.getUChar("tftRot", (uint8_t)TFT_ROTATION) & 0x03;

    // Saved Madgwick beta (float). Defaults to compile-time IMU_MADGWICK_BETA.
    float savedBeta = systemPrefs.getFloat("imuBeta", (float)IMU_MADGWICK_BETA);

    systemPrefs.end();

    // Apply saved rotation now so the first telemetry frame draws in the
    // user's chosen orientation. setRotation() invalidates caches and
    // forces a full redraw internally.
    displayHandler.setRotation(savedRotation);

#if IMU_USE_MADGWICK
    // Apply saved Madgwick beta — overrides the compile-time seed that
    // imuHandler.begin() already set during CP7.
    imuHandler.setMadgwickBeta(savedBeta);
#endif

    DEBUG_PRINTF("Loaded base PID: Kp=%.2f Ki=%.2f Kd=%.2f\n",
                 balance.kp, balance.ki, balance.kd);
    DEBUG_PRINTF("Loaded adaptive bands  slow: Kp=%.2f Kd=%.2f\n",
                 balance.kp_slow,   balance.kd_slow);
    DEBUG_PRINTF("                      med : Kp=%.2f Kd=%.2f\n",
                 balance.kp_medium, balance.kd_medium);
    DEBUG_PRINTF("                      fast: Kp=%.2f Kd=%.2f\n",
                 balance.kp_fast,   balance.kd_fast);
    DEBUG_PRINTF("Sound: %s\n", soundEnabled ? "Enabled" : "Disabled");
}

void savePreferences() {
    systemPrefs.begin("d-o-droid", false);

    // Base PID
    systemPrefs.putFloat("kp", balance.kp);
    systemPrefs.putFloat("ki", balance.ki);
    systemPrefs.putFloat("kd", balance.kd);

    // Target angle (from Auto-Baseline or manual 'pid target' CLI)
    systemPrefs.putFloat("tgtAngle", balance.targetAngle);

    // Adaptive PID 3-band
    systemPrefs.putFloat("kp_slow",   balance.kp_slow);
    systemPrefs.putFloat("kd_slow",   balance.kd_slow);
    systemPrefs.putFloat("kp_med",    balance.kp_medium);
    systemPrefs.putFloat("kd_med",    balance.kd_medium);
    systemPrefs.putFloat("kp_fast",   balance.kp_fast);
    systemPrefs.putFloat("kd_fast",   balance.kd_fast);

    // Sound
    systemPrefs.putBool("sound", soundEnabled);

    // State-reaction toggles
    systemPrefs.putBool("stateRx", stateReactionsEnabled);
    systemPrefs.putBool("idleRx",  idleActionsEnabled);
    systemPrefs.putBool("idleAnim", idleAnimationsEnabled);

    // Driving-dynamics (Mega v3.4 parity)
    systemPrefs.putUChar("drvMode",   driveMode);
    systemPrefs.putBool ("drvShape",  rcShapingEnabled);
    systemPrefs.putFloat("drvDb",     rcDeadband);
    systemPrefs.putFloat("drvExpo",   rcExpo);
    systemPrefs.putBool ("drvRampOn", motorRampingEnabled);
    systemPrefs.putFloat("drvRamp",   motorRampRate);
    systemPrefs.putBool ("drvLeanOn", dynamicAngleEnabled);
    systemPrefs.putFloat("drvLeanMx", maxLeanAngle);

    systemPrefs.end();

    DEBUG_PRINTLN("Preferences saved (base PID + adaptive bands)");
}
