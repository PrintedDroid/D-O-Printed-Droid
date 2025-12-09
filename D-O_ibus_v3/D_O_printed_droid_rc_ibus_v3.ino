/********************************************************************************
 * PROJECT: D-O Self-Balancing Droid - Universal Controller
 * VERSION: 3.3.2 (Fixed watchdog timing, added 45° tilt safety cutoff)
 * DATE:    December 2025
 *
 * DESCRIPTION:
 * Universal control system for D-O droid with flexible input configuration:
 * - MPU6050 IMU-based self-balancing with PID control
 * - Two simple setup modes: PWM Only or iBus
 * - DFPlayer Mini sound system (ALL MODES - always on Mega!)
 * - Multi-servo control for head and mainbar
 * - Battery monitoring with protection
 * - Interactive configuration menu
 * - Optimized and clean code structure
 * - NO external Arduino Nano needed anymore!
 * - Pin-compatible with v1.1 and v2.1!
 *
 * SETUP MODES:
 * Mode 0 - PWM Only (Classic):
 *   - PWM input on pins 3, 4 for drive (tank mixed)
 *   - PWM input on pins 14-17 (A0-A3) for sound switches
 *   - DFPlayer ENABLED on Mega (no external Nano needed!)
 *   - Compatible with original D-Ov2_Mega2560_sketch.ino drive setup
 *   - For standard PWM receivers
 *
 * Mode 1 - iBus (Recommended):
 *   - All 10 channels via iBus protocol
 *   - DFPlayer ENABLED on Mega
 *   - Cleanest wiring, most features
 *   - For FlySky iBus receivers
 *
 *-------------------------------------------------------------------------------
 *
 * HARDWARE:
 * - Arduino Mega 2560
 * - MPU6050 IMU (I2C: SDA=20, SCL=21)
 * - iBus RC receiver (Serial1: RX=19, TX=18) - Mode 1
 * - OR: PWM RC receiver - Mode 0
 * - Cytron motor driver
 * - DFPlayer Mini MP3 player (ALL MODES - always on Mega!)
 * - 4x Servo motors
 * - Battery voltage divider on A15
 *
 *-------------------------------------------------------------------------------
 *
 * PIN ASSIGNMENTS:
 *
 * Motors (All Modes):
 *   Pin 13: Motor 1 Direction
 *   Pin 12: Motor 1 Speed (PWM)
 *   Pin 11: Motor 2 Direction
 *   Pin 10: Motor 2 Speed (PWM)
 *
 * PWM Input (Mode 0 Only):
 *   Pin 3: Drive Channel 1
 *   Pin 4: Drive Channel 2
 *   Pin 14 (A0): Sound Mute Switch
 *   Pin 15 (A1): Sound Mode Switch
 *   Pin 16 (A2): Sound Mood Switch (3-pos)
 *   Pin 17 (A3): Sound Squeak Switch
 *
 * iBus Input (Mode 1 Only):
 *   Pin 19 (Serial1 RX): iBus signal from receiver (all 10 channels)
 *
 * DFPlayer (ALL MODES):
 *   Pin 7: DFPlayer RX
 *   Pin 8: DFPlayer TX
 *
 * Servos (v1.1/v2.1 compatible):
 *   Pin 0: Mainbar Servo
 *   Pin 1: Head Servo 1
 *   Pin 5: Head Servo 2
 *   Pin 6: Head Servo 3
 *
 * Battery Monitor:
 *   Pin A15: Voltage divider input
 *
 *-------------------------------------------------------------------------------
 *
 * CONFIGURATION:
 * Connect via Serial Monitor (9600 baud), send 'm' to enter menu
 * All settings stored in EEPROM
 *
 ********************************************************************************/

#include <Wire.h>
#include <IBusBM.h>
#include <Servo.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// ============================================================================
// CONFIGURATION STRUCTURE
// ============================================================================

struct Configuration {
  uint16_t magic = 0xD042;

  // Setup Mode: 0=PWM Only, 1=iBus
  uint8_t setup_mode = 1;
  uint32_t ibus_baudrate = 115200;  // iBus baudrate (9600 or 115200)

  // PID Configuration
  float kp = 25.0;
  float ki = 0.0;
  float kd = 0.8;
  float target_angle = -0.3;
  float max_integral = 400.0;

  // Feature Toggles
  bool mainbar_correction = true;
  bool ramping_enabled = true;
  bool adaptive_pid_enabled = true;
  bool dynamic_angle_enabled = true;
  bool idle_actions_enabled = true;
  bool state_reactions_enabled = true;
  bool battery_monitor = false;
  bool battery_recovery = false;
  bool servos_enabled = true;
  bool watchdog_enabled = true;

  // Battery (2x 2S = 8.4V full, 6.0V empty)
  float battery_warning = 6.8;
  float battery_critical = 6.4;
  float voltage_divider = 4.03;

  // Sound (all modes)
  uint8_t sound_volume = 25;
  uint16_t min_sound_interval = 500;
  uint16_t idle_interval_min = 5000;
  uint16_t idle_interval_max = 15000;

  // Driving Dynamics
  float ramp_rate = 0.15;
  float max_acceleration = 20.0;
  float max_lean_angle = 3.0;
  uint8_t deadband = 30;
  float expo_factor = 0.5;
  uint8_t mixing_mode = 1;  // 0=Tank (CH1=Motor1, CH2=Motor2), 1=Arcade (CH1=Steering, CH2=Throttle)

  // Adaptive PID Parameters
  float kp_slow = 25.0;
  float kp_medium = 20.0;
  float kp_fast = 15.0;
  float kd_slow = 0.8;
  float kd_medium = 0.6;
  float kd_fast = 0.4;

  // IMU Calibration
  int16_t accel_x_offset = 0;
  int16_t accel_y_offset = 0;
  int16_t accel_z_offset = 0;
  int16_t gyro_x_offset = 0;
  int16_t gyro_y_offset = 0;
  int16_t gyro_z_offset = 0;
};

Configuration config;

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Motors
const uint8_t DIR1_PIN = 13;
const uint8_t PWM1_PIN = 12;
const uint8_t DIR2_PIN = 11;
const uint8_t PWM2_PIN = 10;

// PWM Input (Mode 0) - matched to v1.1/v2.1 pinout
const uint8_t PWM_DRIVE1_PIN = 3;
const uint8_t PWM_DRIVE2_PIN = 4;

// PWM Sound Input (Mode 0)
const uint8_t PWM_SOUND_CH1_PIN = 14;  // A0 - Mute
const uint8_t PWM_SOUND_CH2_PIN = 15;  // A1 - Mode
const uint8_t PWM_SOUND_CH3_PIN = 16;  // A2 - Mood
const uint8_t PWM_SOUND_CH4_PIN = 17;  // A3 - Squeak

// Servos - matched to v1.1/v2.1 pinout
const uint8_t MAINBAR_SERVO_PIN = 0;
const uint8_t HEAD1_SERVO_PIN = 1;
const uint8_t HEAD2_SERVO_PIN = 5;
const uint8_t HEAD3_SERVO_PIN = 6;

// DFPlayer
const uint8_t DFPLAYER_RX_PIN = 7;
const uint8_t DFPLAYER_TX_PIN = 8;

// Battery
const uint8_t BATTERY_PIN = A15;

// ============================================================================
// CONSTANTS
// ============================================================================

// Sound Track Numbers
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
const uint8_t SOUND_SYSTEM_READY = 31;
const uint8_t SOUND_SIGNAL_LOST = 32;

// iBus Channels (Mode 1)
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

// RC Constants
const uint16_t RC_CENTER = 1500;
const uint16_t RC_MIN = 1000;
const uint16_t RC_MAX = 2000;
const uint16_t RC_MIN_VALID = 800;
const uint16_t RC_MAX_VALID = 2200;

// Sound Mood Thresholds
const uint16_t MOOD_LOW_THRESHOLD = 1300;    // Below this = Negative mood
const uint16_t MOOD_HIGH_THRESHOLD = 1700;   // Above this = Positive mood

// Safety
const unsigned long SIGNAL_TIMEOUT = 500;
const unsigned long CALIBRATION_WAIT = 3000;
const uint8_t IMU_MAX_ERRORS = 10;            // Max consecutive IMU errors before reinit
const unsigned long IMU_ERROR_TIMEOUT = 5000; // Time to wait before reinit attempt

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

IBusBM IBus;

Servo mainbarServo;
Servo head1Servo;
Servo head2Servo;
Servo head3Servo;

SoftwareSerial dfplayerSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
DFRobotDFPlayerMini myDFPlayer;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// IMU
uint8_t imu_address = 0x68;  // Default address, may change to 0x69 for some clones
bool imu_found = false;
uint8_t imu_type = 0;  // 0=unknown, 1=MPU6050, 2=MPU6500, 3=MPU9250
int16_t acc_x, acc_y, acc_z;
int16_t gyro_x, gyro_y, gyro_z;
float accel_angle[2];
float gyro_angle[2];
float total_angle[2];
float elapsed_time;
unsigned long current_time, previous_time;  // Changed to unsigned long for micros()

// PID
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

// Motors
float motor_target_1 = 0;
float motor_target_2 = 0;
float motor_ramped_1 = 0;
float motor_ramped_2 = 0;
int motor_speed_1 = 0;
int motor_speed_2 = 0;

// RC Input
uint16_t rc_drive_1 = 1500;
uint16_t rc_drive_2 = 1500;

// Safety
unsigned long last_valid_signal = 0;
bool emergency_stop = false;

// Battery
float battery_voltage = 8.4;
unsigned long last_battery_check = 0;
unsigned long last_battery_warning = 0;
bool battery_critical_triggered = false;

// IMU Error Handling
uint8_t imu_error_count = 0;
unsigned long last_imu_error_time = 0;
bool imu_healthy = true;

// Sound System (All modes)
bool sound_enabled = false;
struct SoundState {
  bool mute_prev = false;
  uint8_t mode_prev = 0;
  uint8_t mood_prev = 1;
  bool squeak_prev = false;
  unsigned long last_sound_time = 0;
} soundState;

// Idle Action System
unsigned long last_idle_action = 0;
unsigned long next_idle_interval = 10000;
unsigned long last_rc_activity = 0;
bool is_idle = false;

// State Monitoring
bool tilt_warning_active = false;
unsigned long last_tilt_warning = 0;

// System
bool system_ready = false;

// Loop Frequency Monitoring
unsigned long loop_count = 0;
unsigned long last_freq_print = 0;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println(F("\n=== D-O Universal Controller v3.3.2 ==="));

  // Load configuration
  loadConfiguration();

  // Initialize current PID values
  current_kp = config.kp;
  current_ki = config.ki;
  current_kd = config.kd;

  // NOTE: Watchdog is enabled AFTER menu wait to prevent reset loop
  // (menu wait is 3 seconds, watchdog timeout is 2 seconds)

  // Initialize I2C and IMU
  Wire.begin();
  Wire.setClock(400000);  // Set I2C to 400kHz (Fast Mode) for faster IMU readings
  initializeIMU();

  // Setup motor pins
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);

  // Setup battery monitor
  pinMode(BATTERY_PIN, INPUT);

  // Print current mode
  printSetupMode();

  // Check for configuration menu (only wait if Serial Monitor connected)
  if (Serial) {
    Serial.println(F("Send 'm' for menu or 'c' for IMU calibration (3 sec)..."));
    unsigned long menu_start = millis();
    while (millis() - menu_start < CALIBRATION_WAIT) {
      if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'm' || cmd == 'M') {
          configurationMenu();
          break;
        } else if (cmd == 'c' || cmd == 'C') {
          runIMUCalibration();
          break;
        }
      }
    }
  }

  // Enable Watchdog Timer AFTER menu wait (2 second timeout)
  // This prevents reset loop since menu wait is 3 seconds
  if (config.watchdog_enabled) {
    wdt_enable(WDTO_2S);
    Serial.println(F("Watchdog enabled (2s timeout)"));
  }

  // Mode-specific initialization
  initializeForMode();

  // Wait for valid RC signal
  waitForRCSignal();

  // Initialize timers (micros for PID, millis for other timing)
  current_time = micros();
  previous_time = current_time;
  last_valid_signal = millis();
  last_rc_activity = millis();
  last_idle_action = millis();
  next_idle_interval = random(config.idle_interval_min, config.idle_interval_max);

  system_ready = true;
  Serial.println(F("System ready!"));

  // Play ready sound
  if (sound_enabled) {
    delay(200);  // Short delay to ensure DFPlayer is ready
    playSound(SOUND_SYSTEM_READY);
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Reset watchdog timer
  if (config.watchdog_enabled) {
    wdt_reset();
  }

  // Loop frequency monitoring (Serial Monitor only)
  if (Serial) {
    loop_count++;
    if (millis() - last_freq_print > 1000) {
      Serial.print(F("Loop Hz: "));
      Serial.println(loop_count);
      loop_count = 0;
      last_freq_print = millis();
    }
  }

  // Check for menu command
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'm' || cmd == 'M') {
      emergency_stop = true;
      if (config.watchdog_enabled) {
        wdt_disable();  // Disable watchdog in menu
      }
      configurationMenu();
      if (config.watchdog_enabled) {
        wdt_enable(WDTO_2S);  // Re-enable after menu
      }
      emergency_stop = false;
    }
  }

  // Update timing using micros() for precise PID control
  previous_time = current_time;
  current_time = micros();
  elapsed_time = (current_time - previous_time) / 1000000.0;  // Convert microseconds to seconds

  // Read RC inputs
  readRCInputs();

  // Battery monitoring
  if (config.battery_monitor) {
    checkBattery();
  }

  // Check for idle state
  checkIdleState();

  // Main control (if not in emergency stop)
  if (!emergency_stop) {
    updateIMUReadings();

    // Update adaptive PID if enabled
    if (config.adaptive_pid_enabled) {
      updateAdaptivePID();
    }

    calculatePID();
    updateMotors();

    // Check for state-based reactions
    if (config.state_reactions_enabled) {
      checkStateReactions();
    }
  } else {
    analogWrite(PWM1_PIN, 0);
    analogWrite(PWM2_PIN, 0);
  }

  // Handle idle actions
  if (config.idle_actions_enabled && is_idle) {
    handleIdleActions();
  }

  // Update servos
  if (config.servos_enabled) {
    updateServos();
  }

  // Handle sound system (only Modes 1 & 2)
  if (sound_enabled) {
    handleSoundSystem();
  }
}

// ============================================================================
// MODE INITIALIZATION
// ============================================================================

void printSetupMode() {
  Serial.print(F("Setup Mode: "));
  switch (config.setup_mode) {
    case 0:
      Serial.println(F("PWM Only"));
      Serial.println(F("  Drive: PWM pins 2-3"));
      Serial.println(F("  Sound: PWM pins 14-17"));
      Serial.println(F("  DFPlayer on Mega"));
      break;
    case 1:
      Serial.println(F("iBus (Recommended)"));
      Serial.println(F("  All 10 channels via iBus"));
      Serial.println(F("  DFPlayer on Mega"));
      break;
    default:
      Serial.println(F("Unknown - defaulting to iBus"));
      config.setup_mode = 1;
      break;
  }
}

void initializeForMode() {
  switch (config.setup_mode) {
    case 0:  // PWM Only
      Serial.println(F("Initializing PWM drive + PWM sound..."));
      pinMode(PWM_DRIVE1_PIN, INPUT);
      pinMode(PWM_DRIVE2_PIN, INPUT);
      pinMode(PWM_SOUND_CH1_PIN, INPUT);
      pinMode(PWM_SOUND_CH2_PIN, INPUT);
      pinMode(PWM_SOUND_CH3_PIN, INPUT);
      pinMode(PWM_SOUND_CH4_PIN, INPUT);
      initializeDFPlayer();
      sound_enabled = true;
      break;

    case 1:  // iBus
      Serial.println(F("Initializing iBus..."));
      Serial1.begin(config.ibus_baudrate);
      IBus.begin(Serial1, IBUSBM_NOTIMER);
      Serial.print(F("iBus @ "));
      Serial.print(config.ibus_baudrate);
      Serial.println(F(" baud"));
      initializeDFPlayer();
      sound_enabled = true;
      break;

    default:
      Serial.println(F("Unknown mode - defaulting to iBus"));
      config.setup_mode = 1;
      Serial1.begin(config.ibus_baudrate);
      IBus.begin(Serial1, IBUSBM_NOTIMER);
      initializeDFPlayer();
      sound_enabled = true;
      break;
  }

  // Initialize servos for all modes
  if (config.servos_enabled) {
    initializeServos();
  }
}

// ============================================================================
// RC INPUT READING
// ============================================================================

void readRCInputs() {
  uint16_t temp1, temp2;

  switch (config.setup_mode) {
    case 0:  // PWM Only
      temp1 = pulseIn(PWM_DRIVE1_PIN, HIGH, 10000);
      temp2 = pulseIn(PWM_DRIVE2_PIN, HIGH, 10000);

      if (temp1 >= RC_MIN_VALID && temp1 <= RC_MAX_VALID &&
          temp2 >= RC_MIN_VALID && temp2 <= RC_MAX_VALID) {
        rc_drive_1 = temp1;
        rc_drive_2 = temp2;
        last_valid_signal = millis();
        // Track RC activity for idle detection
        if (abs(temp1 - RC_CENTER) > 50 || abs(temp2 - RC_CENTER) > 50) {
          last_rc_activity = millis();
        }
        if (emergency_stop) {
          emergency_stop = false;
          Serial.println(F("Signal restored"));
        }
      }
      break;

    case 1:  // iBus
      IBus.loop();
      temp1 = IBus.readChannel(CH_DRIVE1);
      temp2 = IBus.readChannel(CH_DRIVE2);

      if (temp1 >= RC_MIN_VALID && temp1 <= RC_MAX_VALID &&
          temp2 >= RC_MIN_VALID && temp2 <= RC_MAX_VALID) {
        rc_drive_1 = temp1;
        rc_drive_2 = temp2;
        last_valid_signal = millis();
        // Track RC activity for idle detection
        if (abs(temp1 - RC_CENTER) > 50 || abs(temp2 - RC_CENTER) > 50) {
          last_rc_activity = millis();
        }
        if (emergency_stop) {
          emergency_stop = false;
          Serial.println(F("Signal restored"));
        }
      }
      break;

    default:
      Serial.println(F("Unknown mode in readRCInputs"));
      break;
  }

  // Check for signal timeout - set to neutral and keep balancing
  static bool signal_lost_announced = false;
  if (millis() - last_valid_signal > SIGNAL_TIMEOUT) {
    if (!signal_lost_announced) {
      signal_lost_announced = true;
      Serial.println(F("Signal lost - holding neutral position"));
      if (sound_enabled) {
        playSound(SOUND_SIGNAL_LOST);
      }
    }
    // Keep balancing but set drive to neutral (no emergency stop!)
    rc_drive_1 = RC_CENTER;
    rc_drive_2 = RC_CENTER;
  } else {
    signal_lost_announced = false;
  }
}

void waitForRCSignal() {
  Serial.println(F("Waiting for RC signal..."));

  bool signal_valid = false;
  unsigned long wait_start = millis();

  while (!signal_valid && (millis() - wait_start < 10000)) {
    switch (config.setup_mode) {
      case 0:
        rc_drive_1 = pulseIn(PWM_DRIVE1_PIN, HIGH, 10000);
        rc_drive_2 = pulseIn(PWM_DRIVE2_PIN, HIGH, 10000);
        break;
      case 1:
        IBus.loop();
        rc_drive_1 = IBus.readChannel(CH_DRIVE1);
        rc_drive_2 = IBus.readChannel(CH_DRIVE2);
        break;
    }

    if (rc_drive_1 >= RC_MIN_VALID && rc_drive_1 <= RC_MAX_VALID &&
        rc_drive_2 >= RC_MIN_VALID && rc_drive_2 <= RC_MAX_VALID) {
      signal_valid = true;
    }

    delay(100);
  }

  Serial.println(signal_valid ? F("RC signal detected!") : F("Timeout - proceeding anyway"));
}

// ============================================================================
// IMU FUNCTIONS
// ============================================================================

bool initializeIMU() {
  Serial.println(F("Searching for IMU..."));

  // Try primary address 0x68
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    imu_address = 0x68;
    imu_found = true;
    Serial.println(F("IMU found at 0x68"));
  } else {
    // Try alternate address 0x69 (AD0 pin high on some clones)
    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() == 0) {
      imu_address = 0x69;
      imu_found = true;
      Serial.println(F("IMU found at 0x69 (clone/alternate)"));
    }
  }

  if (!imu_found) {
    Serial.println(F("ERROR: No IMU found! Check wiring."));
    return false;
  }

  // Read WHO_AM_I register to identify chip type
  Wire.beginTransmission(imu_address);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(imu_address, (uint8_t)1);
  uint8_t who_am_i = Wire.read();

  Serial.print(F("WHO_AM_I: 0x"));
  Serial.print(who_am_i, HEX);

  switch (who_am_i) {
    case 0x68:
      imu_type = 1;
      Serial.println(F(" (MPU6050)"));
      break;
    case 0x70:
      imu_type = 2;
      Serial.println(F(" (MPU6500)"));
      break;
    case 0x71:
      imu_type = 3;
      Serial.println(F(" (MPU9250)"));
      break;
    case 0x19:
      imu_type = 4;
      Serial.println(F(" (MPU6886)"));
      break;
    default:
      imu_type = 0;
      Serial.println(F(" (Unknown - trying anyway)"));
      break;
  }

  // Wake up IMU (clear sleep bit)
  Wire.beginTransmission(imu_address);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Clear sleep bit
  Wire.endTransmission(true);
  delay(100);  // Wait for wake up

  // Configure Gyroscope (±250°/s)
  Wire.beginTransmission(imu_address);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // FS_SEL = 0 (±250°/s)
  Wire.endTransmission(true);

  // Configure Accelerometer (±2g)
  Wire.beginTransmission(imu_address);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // AFS_SEL = 0 (±2g)
  Wire.endTransmission(true);

  // Verify IMU is responding with valid data
  delay(50);
  Wire.beginTransmission(imu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  uint8_t bytes_received = Wire.requestFrom(imu_address, (uint8_t)6);

  if (bytes_received != 6) {
    Serial.println(F("ERROR: IMU not responding properly!"));
    imu_found = false;
    return false;
  }

  // Flush the buffer
  while (Wire.available()) Wire.read();

  imu_healthy = true;
  Serial.println(F("IMU initialized successfully"));
  return true;
}

void updateIMUReadings() {
  if (!imu_found) return;  // Skip if no IMU

  // Read accelerometer
  Wire.beginTransmission(imu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  uint8_t accel_bytes = Wire.requestFrom(imu_address, (uint8_t)6, (uint8_t)true);

  // Check if IMU responded correctly
  if (accel_bytes != 6) {
    handleIMUError();
    return;
  }

  acc_x = (Wire.read() << 8 | Wire.read()) - config.accel_x_offset;
  acc_y = (Wire.read() << 8 | Wire.read()) - config.accel_y_offset;
  acc_z = (Wire.read() << 8 | Wire.read()) - config.accel_z_offset;

  // Use multiplication instead of pow() for much better performance
  accel_angle[0] = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)) * 180 / PI;
  accel_angle[1] = atan(-1.0 * acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / PI;

  // Read gyroscope
  Wire.beginTransmission(imu_address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  uint8_t gyro_bytes = Wire.requestFrom(imu_address, (uint8_t)6, (uint8_t)true);

  // Check if IMU responded correctly
  if (gyro_bytes != 6) {
    handleIMUError();
    return;
  }

  gyro_x = (Wire.read() << 8 | Wire.read()) - config.gyro_x_offset;
  gyro_y = (Wire.read() << 8 | Wire.read()) - config.gyro_y_offset;
  gyro_z = (Wire.read() << 8 | Wire.read()) - config.gyro_z_offset;

  gyro_angle[0] = gyro_x / 131.0;
  gyro_angle[1] = gyro_y / 131.0;

  // Complementary filter
  total_angle[0] = 0.98 * (total_angle[0] + gyro_angle[0] * elapsed_time) + 0.02 * accel_angle[0];
  total_angle[1] = 0.98 * (total_angle[1] + gyro_angle[1] * elapsed_time) + 0.02 * accel_angle[1];

  // IMU read successful - reset error counter
  if (imu_error_count > 0) {
    imu_error_count = 0;
    if (!imu_healthy) {
      imu_healthy = true;
      Serial.println(F("IMU recovered"));
    }
  }
}

void handleIMUError() {
  imu_error_count++;

  if (imu_error_count >= IMU_MAX_ERRORS && imu_healthy) {
    imu_healthy = false;
    emergency_stop = true;
    Serial.println(F("IMU ERROR - Too many failures!"));
    last_imu_error_time = millis();
  }

  // Try to reinitialize IMU after timeout
  if (!imu_healthy && (millis() - last_imu_error_time > IMU_ERROR_TIMEOUT)) {
    Serial.println(F("Attempting IMU reinit..."));
    initializeIMU();
    imu_error_count = 0;
    last_imu_error_time = millis();
  }
}

void runIMUCalibration() {
  Serial.println(F("\n=== IMU CALIBRATION ==="));

  if (!imu_found) {
    Serial.println(F("ERROR: No IMU found! Run IMU init first."));
    return;
  }

  Serial.println(F("Place robot balanced and keep STILL!"));
  Serial.println(F("Starting in 3 seconds..."));
  delay(3000);

  Serial.println(F("Calibrating..."));

  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 1000;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(imu_address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(imu_address, (uint8_t)14, (uint8_t)true);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // Skip temperature
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
    float drive_input = ((rc_drive_1 + rc_drive_2) / 2.0 - RC_CENTER) / 500.0;
    target += (drive_input * config.max_lean_angle);
  }

  // Calculate error
  pid_error = total_angle[0] - target;

  // PID calculations using current (adaptive) values
  pid_p = current_kp * pid_error;
  pid_i = pid_i + (current_ki * pid_error * elapsed_time);
  pid_i = constrain(pid_i, -config.max_integral, config.max_integral);
  pid_d = current_kd * ((pid_error - pid_previous_error) / elapsed_time);

  pid_output = pid_p + pid_i + pid_d;
  pid_output = constrain(pid_output, -1000, 1000);

  pid_previous_error = pid_error;
}

// ============================================================================
// ADAPTIVE PID
// ============================================================================

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
  // Process RC inputs with deadband
  int centered1 = rc_drive_1 - RC_CENTER;
  int centered2 = rc_drive_2 - RC_CENTER;

  if (abs(centered1) < config.deadband) centered1 = 0;
  if (abs(centered2) < config.deadband) centered2 = 0;

  uint16_t processed1 = RC_CENTER + centered1;
  uint16_t processed2 = RC_CENTER + centered2;

  // Calculate base speeds based on mixing mode
  float base_speed_1, base_speed_2;

  if (config.mixing_mode == 1) {
    // Arcade Mode: CH1=Steering, CH2=Throttle (recommended for standard FlySky config)
    int throttle = map(processed2, RC_MIN, RC_MAX, -255, 255);  // CH2 = forward/back
    int steering = map(processed1, RC_MIN, RC_MAX, -255, 255);  // CH1 = left/right
    base_speed_1 = throttle + steering;  // Left motor
    base_speed_2 = throttle - steering;  // Right motor
  } else {
    // Tank Mode: CH1=Motor1, CH2=Motor2 (original behavior)
    base_speed_1 = map(processed1, RC_MIN, RC_MAX, -255, 255);
    base_speed_2 = map(processed2, RC_MIN, RC_MAX, -255, 255);
  }

  motor_target_1 = base_speed_1;
  motor_target_2 = base_speed_2;

  // Apply ramping
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

  // Determine directions
  bool dir1 = (motor_speed_1 >= 0);
  bool dir2 = (motor_speed_2 >= 0);

  motor_speed_1 = constrain(abs(motor_speed_1), 0, 255);
  motor_speed_2 = constrain(abs(motor_speed_2), 0, 255);

  // SAFETY: Tilt cutoff - stop motors if fallen over (>45 degrees)
  // This prevents motor burnout from stalled motors
  float current_tilt = abs(total_angle[0] - config.target_angle);
  if (current_tilt > 45.0) {
    motor_speed_1 = 0;
    motor_speed_2 = 0;
    if (!emergency_stop) {
      Serial.println(F("TILT CUTOFF: Motors stopped (>45 degrees)"));
      emergency_stop = true;
    }
  }

  // Output to motors
  digitalWrite(DIR1_PIN, dir1 ? HIGH : LOW);
  analogWrite(PWM1_PIN, motor_speed_1);
  digitalWrite(DIR2_PIN, dir2 ? HIGH : LOW);
  analogWrite(PWM2_PIN, motor_speed_2);
}

// ============================================================================
// SERVO CONTROL
// ============================================================================

void initializeServos() {
  mainbarServo.attach(MAINBAR_SERVO_PIN);
  head1Servo.attach(HEAD1_SERVO_PIN);
  head2Servo.attach(HEAD2_SERVO_PIN);
  head3Servo.attach(HEAD3_SERVO_PIN);

  mainbarServo.writeMicroseconds(RC_CENTER);
  head1Servo.writeMicroseconds(RC_CENTER);
  head2Servo.writeMicroseconds(RC_CENTER);
  head3Servo.writeMicroseconds(RC_CENTER);

  Serial.println(F("Servos initialized"));
}

void updateServos() {
  uint16_t mainbar_pos = RC_CENTER;
  uint16_t head1_pos = RC_CENTER;
  uint16_t head2_pos = RC_CENTER;
  uint16_t head3_pos = RC_CENTER;

  // Read servo positions based on mode
  if (config.setup_mode == 1) {  // iBus mode
    mainbar_pos = IBus.readChannel(CH_MAINBAR);
    head1_pos = IBus.readChannel(CH_HEAD1);
    head2_pos = IBus.readChannel(CH_HEAD2);
    head3_pos = IBus.readChannel(CH_HEAD3);
  }
  // Mode 0 (PWM): Would need additional PWM inputs for servos (not implemented)

  // Apply mainbar correction
  if (config.mainbar_correction) {
    int actual_angle = total_angle[0] - config.target_angle;
    int correction = map(actual_angle, 40, -40, 1000, 2000) - RC_CENTER;
    mainbar_pos = constrain(mainbar_pos + correction, RC_MIN, RC_MAX);
  }

  // Write to servos
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
}

// ============================================================================
// BATTERY MONITORING
// ============================================================================

void checkBattery() {
  if (millis() - last_battery_check < 1000) return;

  int raw = analogRead(BATTERY_PIN);
  battery_voltage = (raw * 5.0 / 1024.0) * config.voltage_divider;
  last_battery_check = millis();

  // Critical battery handling with optional recovery (hysteresis)
  if (battery_voltage < config.battery_critical) {
    if (!battery_critical_triggered) {
      battery_critical_triggered = true;
      emergency_stop = true;
      Serial.print(F("CRITICAL BATTERY: "));
      Serial.print(battery_voltage);
      Serial.println(F("V - STOPPED!"));
    }
  } else if (battery_voltage > config.battery_critical + 0.2) {
    // Recovery with hysteresis: 0.2V above critical
    if (battery_critical_triggered && config.battery_recovery) {
      battery_critical_triggered = false;
      emergency_stop = false;
      Serial.println(F("Battery recovered - resuming"));
    }
  }

  // Battery warning (non-critical)
  if (battery_voltage < config.battery_warning && !battery_critical_triggered) {
    if (millis() - last_battery_warning > 30000) {
      last_battery_warning = millis();
      if (sound_enabled) {
        playSound(SOUND_LOW_BATTERY);
      }
      Serial.print(F("Battery warning: "));
      Serial.print(battery_voltage);
      Serial.println(F("V"));
    }
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
  if (emergency_stop) return;

  int action = random(0, 10);

  if (action < 7) {
    // 70% chance of sound
    if (sound_enabled) {
      uint8_t track = random(SOUND_IDLE_START, SOUND_IDLE_END + 1);
      playSound(track);
    }
  } else {
    // 30% chance of head movement
    if (config.servos_enabled) {
      performIdleServoMovement();
    }
  }
}

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
      head1Servo.writeMicroseconds(RC_CENTER);
      break;

    case 1: // Look around
      head2Servo.writeMicroseconds(1200);
      delay(500);
      head2Servo.writeMicroseconds(1800);
      delay(500);
      head2Servo.writeMicroseconds(RC_CENTER);
      break;

    case 2: // Small shake
      for (int i = 0; i < 3; i++) {
        head3Servo.writeMicroseconds(1400);
        delay(150);
        head3Servo.writeMicroseconds(1600);
        delay(150);
      }
      head3Servo.writeMicroseconds(RC_CENTER);
      break;
  }
}

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

      if (sound_enabled) {
        playSound(SOUND_TILT_WARNING);
      }
    }
  } else if (tilt < 5.0 && tilt_warning_active) {
    tilt_warning_active = false;

    if (sound_enabled) {
      playSound(SOUND_RECOVERY);
    }
  }
}

// ============================================================================
// SOUND SYSTEM (All modes)
// ============================================================================

void initializeDFPlayer() {
  dfplayerSerial.begin(9600);

  if (!myDFPlayer.begin(dfplayerSerial)) {
    Serial.println(F("DFPlayer init failed!"));
    sound_enabled = false;
    return;
  }

  myDFPlayer.volume(config.sound_volume);
  delay(500);
  myDFPlayer.play(SOUND_STARTUP);
  soundState.last_sound_time = millis();

  Serial.println(F("DFPlayer initialized"));
}

void handleSoundSystem() {
  // Handle DFPlayer messages
  if (myDFPlayer.available()) {
    myDFPlayer.readType();
    myDFPlayer.read();
  }

  // Respect minimum sound interval
  if (millis() - soundState.last_sound_time < config.min_sound_interval) {
    return;
  }

  // Read sound switches
  uint16_t sw_mute, sw_mode, sw_mood, sw_squeak;

  if (config.setup_mode == 0) {
    // Mode 0 (PWM Only) - read from PWM pins
    sw_mute = pulseIn(PWM_SOUND_CH1_PIN, HIGH, 10000);
    sw_mode = pulseIn(PWM_SOUND_CH2_PIN, HIGH, 10000);
    sw_mood = pulseIn(PWM_SOUND_CH3_PIN, HIGH, 10000);
    sw_squeak = pulseIn(PWM_SOUND_CH4_PIN, HIGH, 10000);

    // Default to low if no signal
    if (sw_mute == 0) sw_mute = RC_MIN;
    if (sw_mode == 0) sw_mode = RC_MIN;
    if (sw_mood == 0) sw_mood = RC_CENTER;
    if (sw_squeak == 0) sw_squeak = RC_MIN;
  } else {
    // Mode 1 (iBus) - read from iBus
    sw_mute = IBus.readChannel(CH_SOUND_MUTE);
    sw_mode = IBus.readChannel(CH_SOUND_MODE);
    sw_mood = IBus.readChannel(CH_SOUND_MOOD);
    sw_squeak = IBus.readChannel(CH_SOUND_SQUEAK);
  }

  // Process mute switch
  bool mute_current = (sw_mute < RC_CENTER);
  if (mute_current != soundState.mute_prev) {
    soundState.mute_prev = mute_current;
    if (!mute_current) {
      playSound(SOUND_DEFAULT);
    }
  }

  if (mute_current) return;  // Muted

  // Mode switch
  uint8_t mode_current = (sw_mode > RC_CENTER) ? 1 : 0;
  if (mode_current != soundState.mode_prev) {
    soundState.mode_prev = mode_current;
    if (mode_current == 1) {
      playSound(random(SOUND_GREET_START, SOUND_GREET_END + 1));
    } else {
      playSound(SOUND_DEFAULT);
    }
  }

  // Mood switch (3-position)
  uint8_t mood_current;
  if (sw_mood < MOOD_LOW_THRESHOLD) {
    mood_current = 0;  // Low - Negative
  } else if (sw_mood > MOOD_HIGH_THRESHOLD) {
    mood_current = 2;  // High - Positive
  } else {
    mood_current = 1;  // Mid - Neutral
  }

  if (mood_current != soundState.mood_prev) {
    soundState.mood_prev = mood_current;
    if (mood_current == 0) {
      playSound(random(SOUND_NEG_START, SOUND_NEG_END + 1));
    } else if (mood_current == 2) {
      playSound(random(SOUND_POS_START, SOUND_POS_END + 1));
    }
  }

  // Squeak switch
  bool squeak_current = (sw_squeak > RC_CENTER);
  if (squeak_current != soundState.squeak_prev) {
    soundState.squeak_prev = squeak_current;
    if (squeak_current) {
      playSound(random(SOUND_SQUEAK_START, SOUND_SQUEAK_END + 1));
    }
  }
}

void playSound(uint8_t track) {
  myDFPlayer.play(track);
  soundState.last_sound_time = millis();
}

// ============================================================================
// CONFIGURATION MENU
// ============================================================================

void configurationMenu() {
  Serial.println(F("\n=== CONFIGURATION MENU ==="));

  while (true) {
    Serial.println(F("\n1. Setup Mode (PWM/iBus)"));
    Serial.println(F("2. PID Configuration"));
    Serial.println(F("3. Adaptive PID Settings"));
    Serial.println(F("4. Driving Dynamics"));
    Serial.println(F("5. Battery Settings"));
    Serial.println(F("6. Sound Settings"));
    Serial.println(F("7. Feature Toggles"));
    Serial.println(F("8. IMU Calibration"));
    Serial.println(F("9. Show Current Status"));
    Serial.println(F("s. Save and Exit"));
    Serial.println(F("0. Exit without Saving"));
    Serial.print(F("Select: "));

    while (!Serial.available()) delay(10);
    char option = Serial.read();
    while (Serial.available()) Serial.read();

    Serial.println(option);

    switch (option) {
      case '1': configureSetupMode(); break;
      case '2': configurePID(); break;
      case '3': configureAdaptivePID(); break;
      case '4': configureDynamics(); break;
      case '5': configureBattery(); break;
      case '6': configureSound(); break;
      case '7': configureFeatures(); break;
      case '8': runIMUCalibration(); break;
      case '9': showStatus(); break;
      case 's':
      case 'S':
        saveConfiguration();
        Serial.println(F("Saved! RESTART required if mode changed."));
        return;
      case '0':
        Serial.println(F("Exiting without saving..."));
        loadConfiguration();
        return;
    }
  }
}

void configureSetupMode() {
  Serial.println(F("\n--- Setup Mode ---"));
  Serial.println(F("0 = PWM Only (Standard PWM receiver)"));
  Serial.println(F("1 = iBus (Recommended - FlySky receivers)"));
  Serial.print(F("Current mode: "));
  Serial.println(config.setup_mode);
  Serial.print(F("Current iBus baudrate: "));
  Serial.println(config.ibus_baudrate);

  Serial.println(F("\nOptions:"));
  Serial.println(F("0-1 = Change mode"));
  Serial.println(F("b   = Change iBus baudrate"));
  Serial.print(F("Choice: "));

  while (!Serial.available()) delay(10);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(choice);

  if (choice >= '0' && choice <= '1') {
    config.setup_mode = choice - '0';
    Serial.print(F("Changed to mode "));
    Serial.println(config.setup_mode);
    Serial.println(F("RESTART REQUIRED!"));
  } else if (choice == 'b' || choice == 'B') {
    configureIbusBaudrate();
  } else {
    Serial.println(F("Invalid choice"));
  }
}

void configureIbusBaudrate() {
  Serial.println(F("\n--- iBus Baudrate ---"));
  Serial.print(F("Current: "));
  Serial.println(config.ibus_baudrate);
  Serial.println(F("1 = 9600 (non-standard)"));
  Serial.println(F("2 = 115200 (standard)"));
  Serial.print(F("Choice [1-2]: "));

  while (!Serial.available()) delay(10);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(choice);

  if (choice == '1') {
    config.ibus_baudrate = 9600;
    Serial.println(F("Set to 9600 - RESTART REQUIRED!"));
  } else if (choice == '2') {
    config.ibus_baudrate = 115200;
    Serial.println(F("Set to 115200 - RESTART REQUIRED!"));
  } else {
    Serial.println(F("Invalid - unchanged"));
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

void configureAdaptivePID() {
  Serial.println(F("\n--- Adaptive PID Settings ---"));

  Serial.println(F("Slow Speed (<50):"));
  config.kp_slow = getFloatInput(F("  KP"), config.kp_slow, 0, 100);
  config.kd_slow = getFloatInput(F("  KD"), config.kd_slow, 0, 10);

  Serial.println(F("Medium Speed (50-150):"));
  config.kp_medium = getFloatInput(F("  KP"), config.kp_medium, 0, 100);
  config.kd_medium = getFloatInput(F("  KD"), config.kd_medium, 0, 10);

  Serial.println(F("Fast Speed (>150):"));
  config.kp_fast = getFloatInput(F("  KP"), config.kp_fast, 0, 100);
  config.kd_fast = getFloatInput(F("  KD"), config.kd_fast, 0, 10);
}

void configureDynamics() {
  Serial.println(F("\n--- Driving Dynamics ---"));
  config.ramp_rate = getFloatInput(F("Ramp Rate (0.01-1.0)"), config.ramp_rate, 0.01, 1.0);
  config.max_acceleration = getFloatInput(F("Max Acceleration"), config.max_acceleration, 1, 100);
  config.max_lean_angle = getFloatInput(F("Max Lean Angle"), config.max_lean_angle, 0, 10);
  config.deadband = getIntInput(F("RC Deadband"), config.deadband, 0, 100);
  config.expo_factor = getFloatInput(F("Expo Factor (0-1)"), config.expo_factor, 0, 1);

  // Mixing Mode Configuration
  Serial.println(F("\n--- RC Mixing Mode ---"));
  Serial.print(F("Current: "));
  Serial.println(config.mixing_mode == 1 ? F("Arcade (CH1=Steer, CH2=Throttle)") : F("Tank (CH1=Motor1, CH2=Motor2)"));
  Serial.println(F("0 = Tank (two sticks, each controls one motor)"));
  Serial.println(F("1 = Arcade (one stick: up/down=drive, left/right=steer) [RECOMMENDED]"));
  Serial.print(F("Choice [0-1]: "));

  while (!Serial.available()) delay(10);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(choice);

  if (choice == '0') {
    config.mixing_mode = 0;
    Serial.println(F("Set to Tank Mode"));
  } else if (choice == '1') {
    config.mixing_mode = 1;
    Serial.println(F("Set to Arcade Mode"));
  } else {
    Serial.println(F("Invalid - unchanged"));
  }
}

void configureBattery() {
  Serial.println(F("\n--- Battery Settings ---"));
  config.battery_warning = getFloatInput(F("Warning V"), config.battery_warning, 6.0, 8.4);
  config.battery_critical = getFloatInput(F("Critical V"), config.battery_critical, 6.0, 8.4);
  config.voltage_divider = getFloatInput(F("Divider Factor"), config.voltage_divider, 1.0, 10.0);

  Serial.print(F("Current: "));
  Serial.print((analogRead(BATTERY_PIN) * 5.0 / 1024.0) * config.voltage_divider);
  Serial.println(F("V"));
}

void configureSound() {
  Serial.println(F("\n--- Sound Settings ---"));
  config.sound_volume = getIntInput(F("Volume (0-30)"), config.sound_volume, 0, 30);
  config.min_sound_interval = getIntInput(F("Min Sound Interval (ms)"), config.min_sound_interval, 100, 5000);
  config.idle_interval_min = getIntInput(F("Min Idle Interval (ms)"), config.idle_interval_min, 1000, 30000);
  config.idle_interval_max = getIntInput(F("Max Idle Interval (ms)"), config.idle_interval_max, config.idle_interval_min, 60000);
}

void configureFeatures() {
  Serial.println(F("\n--- Feature Toggles ---"));
  config.mainbar_correction = getBoolInput(F("Mainbar Auto-Correct"), config.mainbar_correction);
  config.ramping_enabled = getBoolInput(F("Motor Ramping"), config.ramping_enabled);
  config.adaptive_pid_enabled = getBoolInput(F("Adaptive PID"), config.adaptive_pid_enabled);
  config.dynamic_angle_enabled = getBoolInput(F("Dynamic Lean Angle"), config.dynamic_angle_enabled);
  config.idle_actions_enabled = getBoolInput(F("Idle Actions"), config.idle_actions_enabled);
  config.state_reactions_enabled = getBoolInput(F("State Reactions"), config.state_reactions_enabled);
  config.battery_monitor = getBoolInput(F("Battery Monitor"), config.battery_monitor);
  config.battery_recovery = getBoolInput(F("Battery Recovery"), config.battery_recovery);
  config.servos_enabled = getBoolInput(F("Servo Control"), config.servos_enabled);
  config.watchdog_enabled = getBoolInput(F("Watchdog Timer"), config.watchdog_enabled);
}

void showStatus() {
  Serial.println(F("\n--- Current Status ---"));
  Serial.print(F("Battery: "));
  Serial.print(battery_voltage);
  Serial.println(F("V"));
  Serial.print(F("Tilt Angle: "));
  Serial.println(total_angle[0]);
  Serial.print(F("PID Output: "));
  Serial.println(pid_output);
  Serial.print(F("RC Drive: "));
  Serial.print(rc_drive_1);
  Serial.print(F(" / "));
  Serial.println(rc_drive_2);
  Serial.print(F("Motors: "));
  Serial.print(motor_speed_1);
  Serial.print(F(" / "));
  Serial.println(motor_speed_2);
}

// Helper functions
float getFloatInput(const __FlashStringHelper* prompt, float current, float min, float max) {
  Serial.print(prompt);
  Serial.print(F(" ["));
  Serial.print(current);
  Serial.print(F("]: "));

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
  Serial.print(F("] (1/0): "));

  while (!Serial.available()) delay(10);
  char input = Serial.read();
  while (Serial.available()) Serial.read();

  if (input == '\n' || input == '\r') return current;

  bool value = (input == '1');
  Serial.println(value ? F("ON") : F("OFF"));
  return value;
}

// ============================================================================
// EEPROM PERSISTENCE
// ============================================================================

void loadConfiguration() {
  Configuration temp;
  EEPROM.get(0, temp);

  if (temp.magic == 0xD042) {
    config = temp;
    Serial.println(F("Configuration loaded"));
  } else {
    Serial.println(F("Using defaults"));
    saveConfiguration();
  }
}

void saveConfiguration() {
  EEPROM.put(0, config);
  Serial.println(F("Configuration saved"));
}

// ============================================================================
// End of D-O Universal Controller v3.3.0
// ============================================================================
