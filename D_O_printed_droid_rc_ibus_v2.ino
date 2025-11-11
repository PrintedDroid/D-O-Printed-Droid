/********************************************************************************
 * PROJECT: D-O Self-Balancing Droid with iBus Control
 * ORIGINAL: Reinhard Stockinger 2020/11
 * ENHANCED: Optimized version from Printed-Droid.com
 * VERSION: 2.0 (Full Feature Set with Configuration Menu)
 * DATE:    June 2025
 * 
 * DESCRIPTION:
 * Complete control system for a self-balancing D-O droid replica featuring:
 * - MPU6050 IMU-based self-balancing with advanced PID control
 * - iBus RC receiver support (10 channels)
 * - DFPlayer Mini sound system with personality features
 * - Multi-servo control for head and mainbar movement
 * - Battery monitoring with low voltage protection
 * - Interactive configuration menu via Serial Monitor
 * - Idle animations and state-based reactions
 * - Advanced driving dynamics (ramping, adaptive PID, etc.)
 *
 * NEW IN V2.0:
 * - Serial configuration menu with EEPROM storage
 * - Battery voltage monitoring (2x 2S LiPo in series)
 * - Motor ramping for smooth acceleration
 * - Idle action system with random animations
 * - State-based sound reactions (tilt warnings, etc.)
 * - Adaptive PID based on speed
 * - Dynamic target angle for natural leaning
 * - Deadband and exponential RC control
 * - Acceleration limiting
 * - Turn rate compensation
 *
 *-------------------------------------------------------------------------------
 * 
 * HARDWARE REQUIREMENTS:
 * - Arduino Mega 2560 (required for Serial1)
 * - MPU6050 IMU (I2C)
 * - iBus-compatible RC receiver
 * - Cytron motor controller
 * - DFPlayer Mini MP3 player
 * - 4x Servo motors
 * - Speaker (8 ohm, <3W)
 * - Battery voltage divider on A15 (10k/3.3k for 8.4V max)
 * - Micro SD Card with sound files
 *
 *-------------------------------------------------------------------------------
 *
 * SD CARD FILE STRUCTURE:
 * /mp3/
 *   0001.mp3 - Startup sound ("battery charged")
 *   0002.mp3 - Default sound ("I am D-O")
 *   0003.mp3 to 0005.mp3 - Greeting sounds
 *   0006.mp3 to 0009.mp3 - Negative sounds
 *   0010.mp3 to 0014.mp3 - Positive sounds
 *   0015.mp3 to 0020.mp3 - Squeaky wheel sounds
 *   0021.mp3 - Tilt warning sound (new)
 *   0022.mp3 - Recovery/relief sound (new)
 *   0023.mp3 - Low battery warning (new)
 *   0024.mp3 to 0030.mp3 - Idle sounds (new)
 *
 *-------------------------------------------------------------------------------
 *
 * CONFIGURATION MENU:
 * Connect via Serial Monitor (9600 baud) and send:
 * 'c' - IMU calibration
 * 'm' - Main configuration menu
 * 
 * The menu allows configuration of all major parameters including:
 * - PID values
 * - Feature toggles (ramping, adaptive PID, etc.)
 * - Battery thresholds
 * - Sound volumes and intervals
 * All settings are saved to EEPROM
 *
 ********************************************************************************/

#include <Wire.h>
#include <IBusBM.h>
#include <Servo.h>
#include <EEPROM.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// ============================================================================
// FEATURE FLAGS - These can be changed via config menu
// ============================================================================
#define MAINBAR_CORRECTION    // Auto-correct mainbar based on tilt
#define DFPLAYER_ENABLED      // Enable sound system
#define SERVOS_ENABLED        // Enable servo control
#define SETUP_TYPE_RUNTIME    // Allow runtime setup type selection

// ============================================================================
// SETUP TYPE CONFIGURATION
// ============================================================================
// 0 = Original PWM (Balance only, Nano handles sound)
// 1 = Hybrid (iBus for CH1-6, PWM for CH7-10)
// 2 = Pure iBus (All channels via iBus) - DEFAULT
uint8_t setup_type = 2;  // Can be changed in config menu

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// Motor Controller Pins (Cytron)
const uint8_t DIR1_PIN = 13;    // Motor 1 direction
const uint8_t PWM1_PIN = 12;    // Motor 1 speed
const uint8_t DIR2_PIN = 11;    // Motor 2 direction
const uint8_t PWM2_PIN = 10;    // Motor 2 speed

// Servo Pins
const uint8_t MAINBAR_SERVO_PIN = 0;
const uint8_t HEAD1_SERVO_PIN = 1;
const uint8_t HEAD2_SERVO_PIN = 5;
const uint8_t HEAD3_SERVO_PIN = 6;

// DFPlayer Pins
const uint8_t DFPLAYER_RX_PIN = 7;
const uint8_t DFPLAYER_TX_PIN = 8;

// Battery Monitor Pin
const uint8_t BATTERY_PIN = A15;

// ============================================================================
// CONFIGURATION STRUCTURE (Saved to EEPROM)
// ============================================================================

struct Configuration {
  // Magic number to verify EEPROM validity
  uint16_t magic = 0xD042;  // "D-O 42"
  
  // Setup Type Configuration
  uint8_t setup_type = 2;  // 0=PWM, 1=Hybrid, 2=iBus (default)
  
  // PID Configuration
  float kp = 25.0;
  float ki = 0.0;
  float kd = 0.8;
  float target_angle = -0.3;
  float max_integral = 400.0;
  
  // Feature Toggles
  bool ramping_enabled = true;
  bool adaptive_pid_enabled = true;
  bool dynamic_angle_enabled = true;
  bool idle_actions_enabled = true;
  bool state_reactions_enabled = true;
  bool battery_monitor_enabled = true;
  
  // Battery Configuration (2x 2S = 8.4V full, 6.0V empty)
  float battery_warning = 6.8;    // ~3.4V per cell
  float battery_critical = 6.4;   // ~3.2V per cell
  float voltage_divider = 4.03;   // Calibration factor
  
  // Sound Configuration
  uint8_t sound_volume = 25;
  uint16_t min_sound_interval = 500;
  uint16_t idle_interval_min = 5000;
  uint16_t idle_interval_max = 15000;
  
  // Driving Dynamics
  float ramp_rate = 0.15;         // 15% per loop
  float max_acceleration = 20.0;
  float max_lean_angle = 3.0;
  uint8_t deadband = 30;
  float expo_factor = 0.5;
  
  // Adaptive PID Parameters
  float kp_slow = 25.0;
  float kp_medium = 20.0;
  float kp_fast = 15.0;
  float kd_slow = 0.8;
  float kd_medium = 0.6;
  float kd_fast = 0.4;
  
  // IMU Calibration (separate storage)
  int16_t accel_x_offset = 0;
  int16_t accel_y_offset = 0;
  int16_t accel_z_offset = 0;
  int16_t gyro_x_offset = 0;
  int16_t gyro_y_offset = 0;
  int16_t gyro_z_offset = 0;
};

Configuration config;

// ============================================================================
// CONSTANTS
// ============================================================================

// Sound Track Definitions
const uint8_t SOUND_STARTUP = 1;
const uint8_t SOUND_DEFAULT = 2;
const uint8_t SOUND_GREET_START = 3;
const uint8_t SOUND_GREET_END = 5;
const uint8_t SOUND_NEG_START = 6;
const uint8_t SOUND_NEG_END = 9;
const uint8_t SOUND_POS_START = 10;
const uint8_t SOUND_POS_END = 14;
const uint8_t SOUND_SQUEAK_START = 15;
const uint8_t SOUND_SQUEAK_END = 20;
const uint8_t SOUND_TILT_WARNING = 21;
const uint8_t SOUND_RECOVERY = 22;
const uint8_t SOUND_LOW_BATTERY = 23;
const uint8_t SOUND_IDLE_START = 24;
const uint8_t SOUND_IDLE_END = 30;

// iBus Channel Assignments
const uint8_t CH_DRIVE1 = 0;
const uint8_t CH_DRIVE2 = 1;
const uint8_t CH_MAINBAR = 2;
const uint8_t CH_HEAD1 = 3;
const uint8_t CH_HEAD2 = 4;
const uint8_t CH_HEAD3 = 5;
const uint8_t CH_SOUND_MUTE = 6;
const uint8_t CH_SOUND_MODE = 7;
const uint8_t CH_SOUND_MOOD = 8;
const uint8_t CH_SOUND_SQUEAK = 9;

// PWM Pins for Original/Hybrid Setup
const uint8_t PWM_CH7_PIN = 14;  // A0
const uint8_t PWM_CH8_PIN = 15;  // A1
const uint8_t PWM_CH9_PIN = 16;  // A2
const uint8_t PWM_CH10_PIN = 17; // A3

// RC Signal Constants
const uint16_t RC_CENTER = 1500;
const uint16_t RC_MIN_VALID = 800;
const uint16_t RC_MAX_VALID = 2200;

// Safety Configuration
const unsigned long SIGNAL_TIMEOUT = 500;
const unsigned long CALIBRATION_WAIT = 3000;

// EEPROM Addresses
const int EEPROM_CONFIG_START = 0;

// ============================================================================
// GLOBAL OBJECTS AND VARIABLES
// ============================================================================

// IBus object - only created when needed
#if defined(IBUS_ENABLED) || defined(SETUP_TYPE_RUNTIME)
  IBusBM IBus;
  #define IBUS_AVAILABLE
#endif

#ifdef SERVOS_ENABLED
  Servo mainbarServo;
  Servo head1Servo;
  Servo head2Servo;
  Servo head3Servo;
#endif

#ifdef DFPLAYER_ENABLED
  SoftwareSerial mySoftwareSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  DFRobotDFPlayerMini myDFPlayer;
#endif

// IMU Variables
int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
float accel_angle[2];
float gyro_angle[2];
float total_angle[2];
float elapsed_time, current_time, previous_time;

// PID Variables
float pid_error = 0;
float pid_previous_error = 0;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float pid_output = 0;

// Current PID values (may change with adaptive PID)
float current_kp;
float current_ki;
float current_kd;

// Motor Control Variables
float motor_target_1 = 0;
float motor_target_2 = 0;
float motor_ramped_1 = 0;
float motor_ramped_2 = 0;
int motor_speed_1 = 0;
int motor_speed_2 = 0;
int motor_direction_1 = HIGH;
int motor_direction_2 = HIGH;

// RC Input Variables
uint16_t rc_drive_1 = 1500;
uint16_t rc_drive_2 = 1500;
uint16_t rc_drive_1_processed = 1500;
uint16_t rc_drive_2_processed = 1500;

// Safety Variables
unsigned long last_valid_signal = 0;
bool emergency_stop_active = false;

// Battery Monitoring
float battery_voltage = 8.4;
unsigned long last_battery_check = 0;
unsigned long last_battery_warning = 0;
bool battery_low_warned = false;

// Idle Action System
unsigned long last_idle_action = 0;
unsigned long next_idle_interval = 10000;
unsigned long last_rc_activity = 0;
bool is_idle = false;

// State Monitoring
bool tilt_warning_active = false;
unsigned long last_tilt_warning = 0;

// Sound System State
struct SoundState {
  bool enabled = true;
  bool mute_switch_prev = false;
  uint8_t mode_position = 0;
  uint8_t mode_position_prev = 0;
  uint8_t mood_position = 1;
  uint8_t mood_position_prev = 1;
  bool squeak_active = false;
  bool squeak_active_prev = false;
  unsigned long last_sound_time = 0;
} soundState;

// System State
bool system_ready = false;
unsigned long startup_time = 0;

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  // Initialize serial
  Serial.begin(9600);
  Serial.println(F("\n=== D-O Self-Balancing Controller v3.0 ==="));
  
  // Load configuration from EEPROM
  loadConfiguration();
  
  // Update global setup type
  setup_type = config.setup_type;
  
  // Detect old PWM setup
  detectLegacySetup();
  
  // Initialize current PID values
  current_kp = config.kp;
  current_ki = config.ki;
  current_kd = config.kd;
  
  // Initialize I2C for IMU
  Wire.begin();
  initializeIMU();
  
  // Initialize motor pins
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  
  // Stop motors initially
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
  
  // Initialize battery monitoring
  pinMode(BATTERY_PIN, INPUT);
  
  // Initialize PWM pins if needed
  if (setup_type == 0 || setup_type == 1) {
    pinMode(PWM_CH7_PIN, INPUT);
    pinMode(PWM_CH8_PIN, INPUT);
    pinMode(PWM_CH9_PIN, INPUT);
    pinMode(PWM_CH10_PIN, INPUT);
  }
  
  // Check for configuration trigger
  Serial.println(F("Send 'c' for IMU calibration or 'm' for menu within 3 seconds..."));
  
  unsigned long menu_start = millis();
  bool enter_menu = false;
  bool do_calibration = false;
  
  while (millis() - menu_start < CALIBRATION_WAIT) {
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 'm' || cmd == 'M') {
        enter_menu = true;
        break;
      } else if (cmd == 'c' || cmd == 'C') {
        do_calibration = true;
        break;
      }
    }
  }
  
  if (enter_menu) {
    configurationMenu();
  } else if (do_calibration) {
    runIMUCalibration();
  }

  #ifdef SERVOS_ENABLED
    initializeServos();
  #endif

  #ifdef IBUS_ENABLED
    Serial1.begin(115200);
    IBus.begin(Serial1, IBUSBM_NOTIMER);
    Serial.println(F("iBus initialized"));
  #endif

  #ifdef DFPLAYER_ENABLED
    initializeDFPlayer();
  #endif

  // Wait for valid RC signal
  waitForRCSignal();
  
  // Initialize timers
  startup_time = millis();
  current_time = millis();
  last_valid_signal = millis();
  last_rc_activity = millis();
  last_idle_action = millis();
  next_idle_interval = random(config.idle_interval_min, config.idle_interval_max);
  
  system_ready = true;
  Serial.println(F("System ready! Configuration menu available anytime via 'm'"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'm' || cmd == 'M') {
      // Enter configuration menu
      emergency_stop_active = true;  // Stop motors for safety
      configurationMenu();
      emergency_stop_active = false;
    }
  }
  
  // Update loop timing
  previous_time = current_time;
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000.0;
  
  // Read RC inputs
  readRCInputs();
  
  // Battery monitoring
  if (config.battery_monitor_enabled) {
    checkBattery();
  }
  
  // Check for idle state
  checkIdleState();
  
  if (!emergency_stop_active) {
    // Read IMU and calculate angle
    updateIMUReadings();
    
    // Update adaptive PID if enabled
    if (config.adaptive_pid_enabled) {
      updateAdaptivePID();
    }
    
    // Calculate PID output
    calculatePID();
    
    // Apply motor control
    updateMotors();
    
    // Check for state-based reactions
    if (config.state_reactions_enabled) {
      checkStateReactions();
    }
  } else {
    // Emergency stop - disable motors
    analogWrite(PWM1_PIN, 0);
    analogWrite(PWM2_PIN, 0);
  }
  
  // Handle idle actions
  if (config.idle_actions_enabled && is_idle) {
    handleIdleActions();
  }
  
  // Handle sound system
  #ifdef DFPLAYER_ENABLED
    handleSoundSystem();
  #endif
  
  // Update servos
  #ifdef SERVOS_ENABLED
    updateServos();
  #endif
}

// ============================================================================
// CONFIGURATION MENU SYSTEM
// ============================================================================

void configurationMenu() {
  Serial.println(F("\n=== CONFIGURATION MENU ==="));
  
  while (true) {
    Serial.println(F("\n1. PID Configuration"));
    Serial.println(F("2. Feature Toggles"));
    Serial.println(F("3. Battery Settings"));
    Serial.println(F("4. Sound Settings"));
    Serial.println(F("5. Driving Dynamics"));
    Serial.println(F("6. Adaptive PID Settings"));
    Serial.println(F("7. IMU Calibration"));
    Serial.println(F("8. Setup Type (PWM/Hybrid/iBus)"));
    Serial.println(F("9. Save and Exit"));
    Serial.println(F("0. Exit without Saving"));
    Serial.print(F("\nSelect option: "));
    
    while (!Serial.available()) delay(10);
    char option = Serial.read();
    while (Serial.available()) Serial.read(); // Clear buffer
    
    Serial.println(option);
    
    switch (option) {
      case '1': configurePID(); break;
      case '2': configureFeatures(); break;
      case '3': configureBattery(); break;
      case '4': configureSound(); break;
      case '5': configureDynamics(); break;
      case '6': configureAdaptivePID(); break;
      case '7': runIMUCalibration(); break;
      case '8': configureSetupType(); break;
      case '9': 
        saveConfiguration();
        Serial.println(F("Configuration saved!"));
        if (config.setup_type != setup_type) {
          Serial.println(F("SETUP TYPE CHANGED - Please restart!"));
        }
        return;
      case '0':
        Serial.println(F("Exiting without saving..."));
        loadConfiguration(); // Reload old config
        return;
    }
  }
}

void configurePID() {
  Serial.println(F("\n--- PID Configuration ---"));
  
  config.kp = getFloatInput(F("KP"), config.kp, 0, 100);
  config.ki = getFloatInput(F("KI"), config.ki, 0, 10);
  config.kd = getFloatInput(F("KD"), config.kd, 0, 10);
  config.target_angle = getFloatInput(F("Target Angle"), config.target_angle, -10, 10);
  config.max_integral = getFloatInput(F("Max Integral"), config.max_integral, 0, 1000);
  
  // Update current values
  current_kp = config.kp;
  current_ki = config.ki;
  current_kd = config.kd;
}

void configureFeatures() {
  Serial.println(F("\n--- Feature Toggles ---"));
  
  config.ramping_enabled = getBoolInput(F("Motor Ramping"), config.ramping_enabled);
  config.adaptive_pid_enabled = getBoolInput(F("Adaptive PID"), config.adaptive_pid_enabled);
  config.dynamic_angle_enabled = getBoolInput(F("Dynamic Target Angle"), config.dynamic_angle_enabled);
  config.idle_actions_enabled = getBoolInput(F("Idle Actions"), config.idle_actions_enabled);
  config.state_reactions_enabled = getBoolInput(F("State Reactions"), config.state_reactions_enabled);
  config.battery_monitor_enabled = getBoolInput(F("Battery Monitor"), config.battery_monitor_enabled);
}

void configureBattery() {
  Serial.println(F("\n--- Battery Settings ---"));
  Serial.println(F("(For 2x 2S LiPo in series = 8.4V full)"));
  
  config.battery_warning = getFloatInput(F("Warning Voltage"), config.battery_warning, 6.0, 8.4);
  config.battery_critical = getFloatInput(F("Critical Voltage"), config.battery_critical, 6.0, 8.4);
  config.voltage_divider = getFloatInput(F("Voltage Divider Factor"), config.voltage_divider, 1.0, 10.0);
  
  // Test current reading
  float current = readBatteryVoltage();
  Serial.print(F("Current battery reading: "));
  Serial.print(current);
  Serial.println(F("V"));
}

void configureSound() {
  Serial.println(F("\n--- Sound Settings ---"));
  
  config.sound_volume = getIntInput(F("Volume (0-30)"), config.sound_volume, 0, 30);
  config.min_sound_interval = getIntInput(F("Min Sound Interval (ms)"), config.min_sound_interval, 100, 5000);
  config.idle_interval_min = getIntInput(F("Min Idle Interval (ms)"), config.idle_interval_min, 1000, 30000);
  config.idle_interval_max = getIntInput(F("Max Idle Interval (ms)"), config.idle_interval_max, config.idle_interval_min, 60000);
}

void configureDynamics() {
  Serial.println(F("\n--- Driving Dynamics ---"));
  
  config.ramp_rate = getFloatInput(F("Ramp Rate (0.01-1.0)"), config.ramp_rate, 0.01, 1.0);
  config.max_acceleration = getFloatInput(F("Max Acceleration"), config.max_acceleration, 1, 100);
  config.max_lean_angle = getFloatInput(F("Max Lean Angle"), config.max_lean_angle, 0, 10);
  config.deadband = getIntInput(F("RC Deadband"), config.deadband, 0, 100);
  config.expo_factor = getFloatInput(F("Expo Factor (0-1)"), config.expo_factor, 0, 1);
}

void configureAdaptivePID() {
  Serial.println(F("\n--- Adaptive PID Settings ---"));
  
  Serial.println(F("Slow Speed:"));
  config.kp_slow = getFloatInput(F("  KP"), config.kp_slow, 0, 100);
  config.kd_slow = getFloatInput(F("  KD"), config.kd_slow, 0, 10);
  
  Serial.println(F("Medium Speed:"));
  config.kp_medium = getFloatInput(F("  KP"), config.kp_medium, 0, 100);
  config.kd_medium = getFloatInput(F("  KD"), config.kd_medium, 0, 10);
  
  Serial.println(F("Fast Speed:"));
  config.kp_fast = getFloatInput(F("  KP"), config.kp_fast, 0, 100);
  config.kd_fast = getFloatInput(F("  KD"), config.kd_fast, 0, 10);
}

void configureSetupType() {
  Serial.println(F("\n--- Setup Type Configuration ---"));
  Serial.println(F("Current setup type: "));
  
  switch (config.setup_type) {
    case 0: Serial.println(F("PWM Only (Original - Nano handles sound)")); break;
    case 1: Serial.println(F("Hybrid (iBus CH1-6, PWM CH7-10)")); break;
    case 2: Serial.println(F("Pure iBus (All channels via iBus)")); break;
  }
  
  Serial.println(F("\nSelect new setup type:"));
  Serial.println(F("0 = PWM Only (Balance only, external Nano for sound)"));
  Serial.println(F("1 = Hybrid (iBus drive + PWM sound)"));
  Serial.println(F("2 = Pure iBus (Recommended)"));
  Serial.print(F("Choice [0-2]: "));
  
  while (!Serial.available()) delay(10);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();
  
  if (choice >= '0' && choice <= '2') {
    config.setup_type = choice - '0';
    Serial.print(F("Setup type changed to: "));
    Serial.println(config.setup_type);
    Serial.println(F("RESTART REQUIRED for changes to take effect!"));
  } else {
    Serial.println(F("Invalid choice"));
  }
}

// Helper functions for menu input
float getFloatInput(const __FlashStringHelper* prompt, float current, float min, float max) {
  Serial.print(prompt);
  Serial.print(F(" ["));
  Serial.print(current);
  Serial.print(F("] ("));
  Serial.print(min);
  Serial.print(F("-"));
  Serial.print(max);
  Serial.print(F("): "));
  
  while (!Serial.available()) delay(10);
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input.length() == 0) return current;
  
  float value = input.toFloat();
  value = constrain(value, min, max);
  Serial.println(value);
  return value;
}

int getIntInput(const __FlashStringHelper* prompt, int current, int min, int max) {
  return (int)getFloatInput(prompt, current, min, max);
}

bool getBoolInput(const __FlashStringHelper* prompt, bool current) {
  Serial.print(prompt);
  Serial.print(F(" ["));
  Serial.print(current ? F("ON") : F("OFF"));
  Serial.print(F("] (1=ON, 0=OFF): "));
  
  while (!Serial.available()) delay(10);
  char input = Serial.read();
  while (Serial.available()) Serial.read(); // Clear buffer
  
  if (input == '\n' || input == '\r') return current;
  
  bool value = (input == '1');
  Serial.println(value ? F("ON") : F("OFF"));
  return value;
}

// ============================================================================
// CONFIGURATION PERSISTENCE
// ============================================================================

void loadConfiguration() {
  Configuration temp;
  EEPROM.get(EEPROM_CONFIG_START, temp);
  
  if (temp.magic == 0xD042) {
    config = temp;
    Serial.println(F("Configuration loaded from EEPROM"));
  } else {
    Serial.println(F("No valid configuration found, using defaults"));
    saveConfiguration(); // Save defaults
  }
}

void saveConfiguration() {
  EEPROM.put(EEPROM_CONFIG_START, config);
}

// ============================================================================
// IMU FUNCTIONS
// ============================================================================

void initializeIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void updateIMUReadings() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  
  acc_x = (Wire.read() << 8 | Wire.read()) - config.accel_x_offset;
  acc_y = (Wire.read() << 8 | Wire.read()) - config.accel_y_offset;
  acc_z = (Wire.read() << 8 | Wire.read()) - config.accel_z_offset;
  
  accel_angle[0] = atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI;
  accel_angle[1] = atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI;
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  
  gyro_x = (Wire.read() << 8 | Wire.read()) - config.gyro_x_offset;
  gyro_y = (Wire.read() << 8 | Wire.read()) - config.gyro_y_offset;
  gyro_z = (Wire.read() << 8 | Wire.read()) - config.gyro_z_offset;
  
  gyro_angle[0] = gyro_x / 131.0;
  gyro_angle[1] = gyro_y / 131.0;
  
  total_angle[0] = 0.98 * (total_angle[0] + gyro_angle[0] * elapsed_time) + 0.02 * accel_angle[0];
  total_angle[1] = 0.98 * (total_angle[1] + gyro_angle[1] * elapsed_time) + 0.02 * accel_angle[1];
}

void runIMUCalibration() {
  Serial.println(F("\n=== IMU CALIBRATION ==="));
  Serial.println(F("Place robot in balanced position and keep STILL!"));
  Serial.println(F("Starting in 3 seconds..."));
  delay(3000);
  
  Serial.println(F("Calibrating..."));
  
  config.accel_x_offset = 0;
  config.accel_y_offset = 0;
  config.accel_z_offset = 0;
  config.gyro_x_offset = 0;
  config.gyro_y_offset = 0;
  config.gyro_z_offset = 0;
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    
    if (i % 100 == 0) Serial.print(F("."));
    delay(3);
  }
  
  config.accel_x_offset = ax_sum / samples;
  config.accel_y_offset = ay_sum / samples;
  config.accel_z_offset = (az_sum / samples) - 16384;
  config.gyro_x_offset = gx_sum / samples;
  config.gyro_y_offset = gy_sum / samples;
  config.gyro_z_offset = gz_sum / samples;
  
  Serial.println(F("\nCalibration complete!"));
  saveConfiguration();
}

// ============================================================================
// PID CONTROL
// ============================================================================

void calculatePID() {
  // Calculate target angle
  float target = config.target_angle;
  
  if (config.dynamic_angle_enabled) {
    // Add forward lean based on drive input
    float drive_input = ((rc_drive_1_processed + rc_drive_2_processed) / 2.0 - 1500) / 500.0;
    target += (drive_input * config.max_lean_angle);
  }
  
  // Calculate error
  pid_error = total_angle[0] - target;
  
  // PID calculations
  pid_p = current_kp * pid_error;
  pid_i = pid_i + (current_ki * pid_error * elapsed_time);
  pid_i = constrain(pid_i, -config.max_integral, config.max_integral);
  pid_d = current_kd * ((pid_error - pid_previous_error) / elapsed_time);
  
  pid_output = pid_p + pid_i + pid_d;
  pid_output = constrain(pid_output, -1000, 1000);
  
  pid_previous_error = pid_error;
}

void updateAdaptivePID() {
  int avg_speed = abs(motor_speed_1 + motor_speed_2) / 2;
  
  if (avg_speed < 50) {
    current_kp = config.kp_slow;
    current_kd = config.kd_slow;
  } else if (avg_speed < 150) {
    current_kp = config.kp_medium;
    current_kd = config.kd_medium;
  } else {
    current_kp = config.kp_fast;
    current_kd = config.kd_fast;
  }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void updateMotors() {
  // Process RC inputs
  rc_drive_1_processed = processRCInput(rc_drive_1);
  rc_drive_2_processed = processRCInput(rc_drive_2);
  
  // Calculate base speeds
  float base_speed_1 = map(rc_drive_1_processed, 1000, 2000, -255, 255);
  float base_speed_2 = map(rc_drive_2_processed, 1000, 2000, -255, 255);
  
  // Apply acceleration limiting
  base_speed_1 = applyAccelerationLimit(base_speed_1, motor_target_1);
  base_speed_2 = applyAccelerationLimit(base_speed_2, motor_target_2);
  
  motor_target_1 = base_speed_1;
  motor_target_2 = base_speed_2;
  
  // Apply ramping if enabled
  if (config.ramping_enabled) {
    motor_ramped_1 += (motor_target_1 - motor_ramped_1) * config.ramp_rate;
    motor_ramped_2 += (motor_target_2 - motor_ramped_2) * config.ramp_rate;
  } else {
    motor_ramped_1 = motor_target_1;
    motor_ramped_2 = motor_target_2;
  }
  
  // Add PID correction
  motor_speed_1 = motor_ramped_1 + pid_output;
  motor_speed_2 = motor_ramped_2 - pid_output;
  
  // Apply turn compensation
  applyTurnCompensation();
  
  // Determine directions and constrain
  motor_direction_1 = (motor_speed_1 >= 0) ? HIGH : LOW;
  motor_direction_2 = (motor_speed_2 >= 0) ? HIGH : LOW;
  
  motor_speed_1 = constrain(abs(motor_speed_1), 0, 255);
  motor_speed_2 = constrain(abs(motor_speed_2), 0, 255);
  
  // Output to motors
  digitalWrite(DIR1_PIN, motor_direction_1);
  analogWrite(PWM1_PIN, motor_speed_1);
  digitalWrite(DIR2_PIN, motor_direction_2);
  analogWrite(PWM2_PIN, motor_speed_2);
}

int processRCInput(int raw_input) {
  int centered = raw_input - 1500;
  
  // Apply deadband
  if (abs(centered) < config.deadband) return 1500;
  
  // Apply exponential curve
  float normalized = centered / 500.0;
  float curved = normalized * (abs(normalized) * config.expo_factor + (1 - config.expo_factor));
  
  return 1500 + (int)(curved * 500);
}

float applyAccelerationLimit(float target, float current) {
  float delta = target - current;
  delta = constrain(delta, -config.max_acceleration, config.max_acceleration);
  return current + delta;
}

void applyTurnCompensation() {
  int speed_diff = abs(motor_speed_1 - motor_speed_2);
  float turn_factor = map(speed_diff, 0, 255, 100, 70) / 100.0;
  
  motor_speed_1 = motor_speed_1 * turn_factor;
  motor_speed_2 = motor_speed_2 * turn_factor;
}

// ============================================================================
// LEGACY SETUP DETECTION
// ============================================================================

void detectLegacySetup() {
  Serial.println(F("Checking for legacy PWM connections..."));
  
  // Quick check for PWM signals on sound pins
  bool pwm_detected = false;
  
  for (int i = 0; i < 3; i++) {
    uint16_t test7 = pulseIn(PWM_CH7_PIN, HIGH, 5000);
    uint16_t test8 = pulseIn(PWM_CH8_PIN, HIGH, 5000);
    
    if (test7 > 500 || test8 > 500) {
      pwm_detected = true;
      break;
    }
  }
  
  if (pwm_detected && setup_type == 2) {
    Serial.println(F("WARNING: PWM signals detected on pins 14-17!"));
    Serial.println(F("This suggests old Nano sound setup is still connected."));
    Serial.println(F("Please either:"));
    Serial.println(F("1. Remove connections from pins 14-17, OR"));
    Serial.println(F("2. Change to Hybrid mode in configuration menu"));
  } else if (!pwm_detected && setup_type < 2) {
    Serial.println(F("No PWM signals detected on sound channels."));
    Serial.println(F("Consider switching to Pure iBus mode for all features."));
  }
}

// ============================================================================
// RC INPUT HANDLING
// ============================================================================

void readRCInputs() {
  if (setup_type == 0) {
    // PWM Only Mode - Read drive channels via PWM
    rc_drive_1 = pulseIn(3, HIGH, 25000);
    rc_drive_2 = pulseIn(4, HIGH, 25000);
    
    // Validate
    if (rc_drive_1 > 0 && rc_drive_2 > 0) {
      last_valid_signal = millis();
      if (abs(rc_drive_1 - 1500) > 50 || abs(rc_drive_2 - 1500) > 50) {
        last_rc_activity = millis();
      }
      if (emergency_stop_active) {
        emergency_stop_active = false;
        Serial.println(F("Signal restored"));
      }
    } else if (millis() - last_valid_signal > SIGNAL_TIMEOUT) {
      if (!emergency_stop_active) {
        emergency_stop_active = true;
        Serial.println(F("EMERGENCY STOP - Signal lost!"));
      }
    }
  } else {
    // iBus or Hybrid Mode - Read drive via iBus
    #ifdef IBUS_AVAILABLE
      IBus.loop();
      
      uint16_t temp1 = IBus.readChannel(CH_DRIVE1);
      uint16_t temp2 = IBus.readChannel(CH_DRIVE2);
      
      if (temp1 >= RC_MIN_VALID && temp1 <= RC_MAX_VALID &&
          temp2 >= RC_MIN_VALID && temp2 <= RC_MAX_VALID) {
        rc_drive_1 = temp1;
        rc_drive_2 = temp2;
        last_valid_signal = millis();
        
        if (abs(temp1 - 1500) > 50 || abs(temp2 - 1500) > 50) {
          last_rc_activity = millis();
        }
        
        if (emergency_stop_active) {
          emergency_stop_active = false;
          Serial.println(F("Signal restored"));
        }
      } else {
        if (millis() - last_valid_signal > SIGNAL_TIMEOUT) {
          if (!emergency_stop_active) {
            emergency_stop_active = true;
            Serial.println(F("EMERGENCY STOP - Signal lost!"));
          }
        }
      }
    #endif
  }
}

void waitForRCSignal() {
  Serial.println(F("Waiting for RC signal..."));
  
  bool signal_valid = false;
  unsigned long wait_start = millis();
  
  while (!signal_valid && (millis() - wait_start < 30000)) {
    if (setup_type == 0) {
      // PWM mode
      rc_drive_1 = pulseIn(3, HIGH, 25000);
      rc_drive_2 = pulseIn(4, HIGH, 25000);
    } else {
      // iBus or Hybrid mode
      #ifdef IBUS_AVAILABLE
        IBus.loop();
        rc_drive_1 = IBus.readChannel(CH_DRIVE1);
        rc_drive_2 = IBus.readChannel(CH_DRIVE2);
      #endif
    }
    
    if (rc_drive_1 >= RC_MIN_VALID && rc_drive_1 <= RC_MAX_VALID &&
        rc_drive_2 >= RC_MIN_VALID && rc_drive_2 <= RC_MAX_VALID) {
      signal_valid = true;
    }
    
    delay(100);
  }
  
  Serial.println(signal_valid ? F("RC signal detected!") : F("RC timeout - proceeding"));
}

// ============================================================================
// BATTERY MONITORING
// ============================================================================

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw * 5.0 / 1024.0) * config.voltage_divider;
  return voltage;
}

void checkBattery() {
  if (millis() - last_battery_check < 1000) return; // Check every second
  
  battery_voltage = readBatteryVoltage();
  last_battery_check = millis();
  
  if (battery_voltage < config.battery_critical) {
    // Critical - stop motors
    emergency_stop_active = true;
    Serial.print(F("CRITICAL BATTERY: "));
    Serial.print(battery_voltage);
    Serial.println(F("V - MOTORS STOPPED!"));
  } else if (battery_voltage < config.battery_warning) {
    // Warning
    if (!battery_low_warned || millis() - last_battery_warning > 30000) {
      battery_low_warned = true;
      last_battery_warning = millis();
      
      #ifdef DFPLAYER_ENABLED
        playSound(SOUND_LOW_BATTERY);
      #endif
      
      Serial.print(F("Battery warning: "));
      Serial.print(battery_voltage);
      Serial.println(F("V"));
    }
  } else {
    battery_low_warned = false;
  }
}

// ============================================================================
// IDLE ACTION SYSTEM
// ============================================================================

void checkIdleState() {
  is_idle = (millis() - last_rc_activity > 3000); // 3 seconds of no activity
}

void handleIdleActions() {
  if (millis() - last_idle_action > next_idle_interval) {
    performRandomIdleAction();
    last_idle_action = millis();
    next_idle_interval = random(config.idle_interval_min, config.idle_interval_max);
  }
}

void performRandomIdleAction() {
  if (emergency_stop_active) return;
  
  int action = random(0, 10);
  
  if (action < 7) {
    // 70% chance of sound
    #ifdef DFPLAYER_ENABLED
      uint8_t track = random(SOUND_IDLE_START, SOUND_IDLE_END + 1);
      playSound(track);
    #endif
  } else {
    // 30% chance of head movement
    #ifdef SERVOS_ENABLED
      performIdleServoMovement();
    #endif
  }
}

#ifdef SERVOS_ENABLED
void performIdleServoMovement() {
  // Random head movement
  int movement_type = random(0, 3);
  
  switch (movement_type) {
    case 0: // Nod
      for (int i = 0; i < 2; i++) {
        head1Servo.writeMicroseconds(1300);
        delay(300);
        head1Servo.writeMicroseconds(1700);
        delay(300);
      }
      head1Servo.writeMicroseconds(1500);
      break;
      
    case 1: // Look around
      head2Servo.writeMicroseconds(1200);
      delay(500);
      head2Servo.writeMicroseconds(1800);
      delay(500);
      head2Servo.writeMicroseconds(1500);
      break;
      
    case 2: // Small shake
      for (int i = 0; i < 3; i++) {
        head3Servo.writeMicroseconds(1400);
        delay(150);
        head3Servo.writeMicroseconds(1600);
        delay(150);
      }
      head3Servo.writeMicroseconds(1500);
      break;
  }
}
#endif

// ============================================================================
// STATE-BASED REACTIONS
// ============================================================================

void checkStateReactions() {
  // Tilt warning
  float tilt = abs(total_angle[0] - config.target_angle);
  
  if (tilt > 15.0 && !tilt_warning_active) {
    if (millis() - last_tilt_warning > 5000) { // Prevent spam
      tilt_warning_active = true;
      last_tilt_warning = millis();
      
      #ifdef DFPLAYER_ENABLED
        playSound(SOUND_TILT_WARNING);
      #endif
    }
  } else if (tilt < 5.0 && tilt_warning_active) {
    tilt_warning_active = false;
    
    #ifdef DFPLAYER_ENABLED
      playSound(SOUND_RECOVERY);
    #endif
  }
}

// ============================================================================
// SERVO CONTROL
// ============================================================================

#ifdef SERVOS_ENABLED
void initializeServos() {
  mainbarServo.attach(MAINBAR_SERVO_PIN);
  head1Servo.attach(HEAD1_SERVO_PIN);
  head2Servo.attach(HEAD2_SERVO_PIN);
  head3Servo.attach(HEAD3_SERVO_PIN);
  
  mainbarServo.writeMicroseconds(1500);
  head1Servo.writeMicroseconds(1500);
  head2Servo.writeMicroseconds(1500);
  head3Servo.writeMicroseconds(1500);
  
  Serial.println(F("Servos initialized"));
}

void updateServos() {
  if (setup_type > 0) {
    // iBus or Hybrid mode - servos via iBus
    #ifdef IBUS_AVAILABLE
      uint16_t mainbar_pos = IBus.readChannel(CH_MAINBAR);
      uint16_t head1_pos = IBus.readChannel(CH_HEAD1);
      uint16_t head2_pos = IBus.readChannel(CH_HEAD2);
      uint16_t head3_pos = IBus.readChannel(CH_HEAD3);
      
      #ifdef MAINBAR_CORRECTION
        int actual_angle = total_angle[0] - config.target_angle;
        int angle_correction = map(actual_angle, 40, -40, 1000, 2000) - 1500;
        mainbar_pos = constrain(mainbar_pos + angle_correction, 1000, 2000);
      #endif
      
      if (mainbar_pos >= RC_MIN_VALID && mainbar_pos <= RC_MAX_VALID) {
        mainbarServo.writeMicroseconds(mainbar_pos);
      }
      if (head1_pos >= RC_MIN_VALID && head1_pos <= RC_MAX_VALID) {
        head1Servo.writeMicroseconds(head1_pos);
      }
      if (head2_pos >= RC_MIN_VALID && head2_pos <= RC_MAX_VALID) {
        head2Servo.writeMicroseconds(head2_pos);
      }
      if (head3_pos >= RC_MIN_VALID && head3_pos <= RC_MAX_VALID) {
        head3Servo.writeMicroseconds(head3_pos);
      }
    #endif
  }
  // In PWM-only mode, servos would need separate PWM inputs
}
#endif

// ============================================================================
// SOUND SYSTEM
// ============================================================================

#ifdef DFPLAYER_ENABLED
void initializeDFPlayer() {
  mySoftwareSerial.begin(9600);
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("DFPlayer init failed!"));
    return;
  }
  
  myDFPlayer.volume(config.sound_volume);
  delay(500);
  myDFPlayer.play(SOUND_STARTUP);
  soundState.last_sound_time = millis();
  
  Serial.println(F("DFPlayer initialized"));
}

void handleSoundSystem() {
  // Only handle sound in Pure iBus mode
  if (setup_type != 2) return;
  
  if (myDFPlayer.available()) {
    myDFPlayer.readType();
    myDFPlayer.read();
  }
  
  if (millis() - soundState.last_sound_time < config.min_sound_interval) {
    return;
  }
  
  // Read sound switches based on setup type
  uint16_t sw_mute, sw_mode, sw_mood, sw_squeak;
  
  if (setup_type == 2) {
    // Pure iBus - read all from iBus
    #ifdef IBUS_AVAILABLE
      sw_mute = IBus.readChannel(CH_SOUND_MUTE);
      sw_mode = IBus.readChannel(CH_SOUND_MODE);
      sw_mood = IBus.readChannel(CH_SOUND_MOOD);
      sw_squeak = IBus.readChannel(CH_SOUND_SQUEAK);
    #else
      return; // Should not happen
    #endif
  } else if (setup_type == 1) {
    // Hybrid - read sound channels from PWM
    sw_mute = pulseIn(PWM_CH7_PIN, HIGH, 25000);
    sw_mode = pulseIn(PWM_CH8_PIN, HIGH, 25000);
    sw_mood = pulseIn(PWM_CH9_PIN, HIGH, 25000);
    sw_squeak = pulseIn(PWM_CH10_PIN, HIGH, 25000);
    
    // Validate PWM readings
    if (sw_mute == 0) sw_mute = 1000;
    if (sw_mode == 0) sw_mode = 1000;
    if (sw_mood == 0) sw_mood = 1500;
    if (sw_squeak == 0) sw_squeak = 1000;
  } else {
    // PWM only - no sound handling
    return;
  }
  
  // Process switches with state detection
  bool mute_current = (sw_mute < RC_CENTER);
  if (mute_current != soundState.mute_switch_prev) {
    soundState.mute_switch_prev = mute_current;
    soundState.enabled = !mute_current;
    
    if (soundState.enabled) {
      playSound(SOUND_DEFAULT);
    }
  }
  
  if (!soundState.enabled) return;
  
  // Mode switch
  uint8_t mode_current = (sw_mode > RC_CENTER) ? 1 : 0;
  if (mode_current != soundState.mode_position_prev) {
    soundState.mode_position_prev = mode_current;
    
    if (mode_current == 1) {
      uint8_t track = random(SOUND_GREET_START, SOUND_GREET_END + 1);
      playSound(track);
    } else {
      playSound(SOUND_DEFAULT);
    }
  }
  
  // Mood switch (3-position)
  uint8_t mood_current;
  if (sw_mood < 1300) {
    mood_current = 0;
  } else if (sw_mood > 1700) {
    mood_current = 2;
  } else {
    mood_current = 1;
  }
  
  if (mood_current != soundState.mood_position_prev) {
    soundState.mood_position_prev = mood_current;
    
    if (mood_current == 0) {
      uint8_t track = random(SOUND_NEG_START, SOUND_NEG_END + 1);
      playSound(track);
    } else if (mood_current == 2) {
      uint8_t track = random(SOUND_POS_START, SOUND_POS_END + 1);
      playSound(track);
    }
  }
  
  // Squeak switch
  bool squeak_current = (sw_squeak > RC_CENTER);
  if (squeak_current != soundState.squeak_active_prev) {
    soundState.squeak_active_prev = squeak_current;
    
    if (squeak_current) {
      uint8_t track = random(SOUND_SQUEAK_START, SOUND_SQUEAK_END + 1);
      playSound(track);
    }
  }
}

void playSound(uint8_t track) {
  myDFPlayer.play(track);
  soundState.last_sound_time = millis();
}
#endif

// ============================================================================
// End of D-O Self-Balancing Controller v2.0
// ============================================================================