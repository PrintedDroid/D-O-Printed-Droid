/********************************************************************************
 * PROJECT: D-O Self-Balancing Droid - Universal Controller
 * VERSION: 3.4.0 (Review fixes on top of v3.3.6)
 * DATE:    April 2026
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
// FIRMWARE-LINE PORT FLAGS (cross-port from AIO32 v2)
// ============================================================================
//
// Flags that switch individual features between the old proven pathway and
// the newer implementation ported from the ESP32-S3 AIO32 sibling sketch.
// See MEGA_PORTING_PLAN.md in ../../D-O_AIO32/ for the porting history.
//
// All flags default to the safer behaviour so flashing this sketch without
// changing anything gives the same behaviour as v3.4.x.
//
// USE_PLAYMP3FOLDER:
//   0 = legacy myDFPlayer.play(N)       — FAT-order-dependent. The SD card
//                                          must be loaded file-by-file in
//                                          numerical order or DriveSort'd.
//   1 = myDFPlayer.playMp3Folder(N)     — filename-based, looks for
//                                          /mp3/NNNN.mp3 on the SD card.
//                                          FAT-order-independent. Cross-port
//                                          from AIO32 (Makuna playMp3FolderTrack).
//
//   Risk: very low. Both methods exist in the DFRobot library; the only
//   difference is the iBus command code sent to the DFPlayer chip (0x03 vs
//   0x12). File layout /mp3/NNNN.mp3 is already the convention the Mega
//   handbook documents, so existing SD cards should work unchanged.
//
//   Hardware-test on Mega v3.4.0+: flash, boot, listen for startup sound
//   (track 0001). If silent, revert to USE_PLAYMP3FOLDER 0 and report.
#define USE_PLAYMP3FOLDER   1

// USE_PID_CONTROLLER_CLASS:
//   0 = legacy calculatePID() inline (v3.4 proven) — no filter on D, integral
//       bounded via constrain() only. Shipped stable in hundreds of droids.
//   1 = ported class-style PID with D-term first-order low-pass filter and
//       back-calculation anti-windup (only integrates when the output is not
//       saturated in the same direction). Same gains, cleaner dynamics in
//       theory — but any change to the balance math must be hardware-verified
//       on a bench before enabling in production.
//
//   Risk: medium. The maths is equivalent at the fixed-point but transient
//   behaviour differs. Tuning values for kp/ki/kd that were dialled in with
//   the legacy loop may need a nudge with the class-style loop.
//
//   Hardware-test: on a stand, command balance, observe oscillation onset
//   vs the old path. Tune if needed; revert to 0 if the droid won't stabilise.
#define USE_PID_CONTROLLER_CLASS  0

// USE_MADGWICK_AHRS:
//   0 = legacy complementary filter (0.98 * gyro + 0.02 * accel) — proven,
//       runs comfortably at 100 Hz on ATmega2560.
//   1 = ported Madgwick 6-DoF quaternion AHRS. Better gyro-bias rejection,
//       better pitch accuracy in sustained accelerations, but more float
//       math (~20 muls + a few sqrts per update, ~300 us on Mega).
//
//   Risk: medium-high. Output angle units are the same (degrees pitch/roll)
//   and the same balance_angle feeds the PID, but the filter dynamics are
//   different. Adaptive PID gains tuned for the complementary filter may
//   behave differently with Madgwick.
//
//   Hardware-test: hold droid steady, compare pitch readout for drift
//   vs the old path. Then do a balance test — it should feel steadier
//   against sustained pushes. If unstable: back to 0 and report.
#define USE_MADGWICK_AHRS     0

// ============================================================================
// CONFIGURATION STRUCTURE
// ============================================================================

// RC protocol enum for Configuration.rc_protocol (only valid when setup_mode==1 = Serial).
// SBUS requires an **external hardware inverter** between receiver and Serial1-RX,
// because the AVR UART cannot invert the signal like the ESP32-S3 can.
#define RC_PROTO_IBUS 0
#define RC_PROTO_SBUS 1

struct Configuration {
  uint16_t magic = 0xD044;  // v3.4.1+ layout. Previous: 0xD043 (v3.4.0) and 0xD042 (v2.1/v3.x).
                            // Bumped in v3.4.1 when rc_protocol field was added and max_acceleration removed.

  // Setup Mode: 0=PWM Only, 1=Serial (iBus or SBUS chosen by rc_protocol)
  uint8_t setup_mode = 1;
  uint32_t ibus_baudrate = 115200;  // iBus baudrate (9600 or 115200). Unused when rc_protocol=SBUS (fixed 100000 8E2).

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
  bool battery_monitor = false;   // Default OFF: the voltage divider is NOT populated on the PCB ex-works, user must wire it to A15 first
  bool battery_recovery = false;
  bool servos_enabled = true;
  bool watchdog_enabled = true;

  // Battery (2x 2S in series = 4S, 16.8V full, 12.0V empty at 3.0V/cell)
  float battery_warning = 13.6;   // ~3.4V per cell
  float battery_critical = 12.8;  // ~3.2V per cell
  float voltage_divider = 4.03;   // Hardware divider 10k/3.3k for 4S (max ~20V at A15=5V)

  // Sound (all modes)
  uint8_t sound_volume = 25;
  uint16_t min_sound_interval = 500;
  uint16_t idle_interval_min = 5000;
  uint16_t idle_interval_max = 15000;

  // Driving Dynamics
  float ramp_rate = 0.15;
  // (max_acceleration removed in v3.4.1 — was dead-code EEPROM-only field since v3.4.0.
  //  EEPROM-Migration handled by magic bump 0xD043 -> 0xD044, see loadConfiguration().)
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

  // Motor Configuration
  bool motor_swap = false;      // Swap Motor 1 and Motor 2
  bool motor1_invert = false;   // Invert Motor 1 direction
  bool motor2_invert = false;   // Invert Motor 2 direction

  // IMU Configuration
  bool imu_invert = false;      // Invert balance axis (if tilting forward shows wrong direction)

  // IMU Calibration
  int16_t accel_x_offset = 0;
  int16_t accel_y_offset = 0;
  int16_t accel_z_offset = 0;
  int16_t gyro_x_offset = 0;
  int16_t gyro_y_offset = 0;
  int16_t gyro_z_offset = 0;

  // RC Protocol (v3.4.1+): RC_PROTO_IBUS=0 (default, FlySky) or RC_PROTO_SBUS=1 (Futaba/FrSky, NEEDS EXTERNAL INVERTER).
  // Menu option 'r' switches this at runtime. Also auto-clamped in loadConfiguration()
  // to default if EEPROM contains garbage (>1).
  uint8_t rc_protocol = RC_PROTO_IBUS;

  // Madgwick AHRS beta (v3.4.1+): only used when USE_MADGWICK_AHRS compile-flag
  // is enabled. Controls the gradient-descent gain that fuses accel into the
  // gyro-integrated quaternion. Range 0.02..0.3 typical; higher = more accel
  // weight (less drift, noisier). Menu option 'b' edits this at runtime.
  float madgwick_beta = 0.1f;
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

// ============================================================================
// SBUS parser state (v3.4.1 — Futaba/FrSky 25-byte frame at 100000 8E2).
// Requires an **external hardware inverter** between the receiver SBUS
// output and Serial1-RX on the Mega (AVR UART can't invert in hardware).
// When config.rc_protocol == RC_PROTO_SBUS we bypass the IBusBM library and
// feed Serial1 bytes into an inline state machine.
// ============================================================================
uint8_t  sbus_buf[25];
uint8_t  sbus_idx = 0;
uint32_t sbus_last_byte_us = 0;
uint16_t sbus_channels_us[16];      // Parsed channels in 1000..2000 µs convention
bool     sbus_failsafe_active = false;

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

#if USE_PID_CONTROLLER_CLASS
// ---- Class-style PID state (Port A from AIO32) ----
// Mirrors the AIO32 pid_controller.h structure, but compiled as struct for
// AVR RAM friendliness. D-term first-order low-pass + back-calculation
// anti-windup.
struct PIDClassState {
  float d_filtered;        // low-pass filtered derivative
  float prev_error;
  float integrator;
};
static PIDClassState pidc = { 0.0f, 0.0f, 0.0f };
// D-term low-pass cutoff. Alpha in [0..1]. 0.15 ≈ 30 Hz LPF at 100 Hz loop.
const float PIDC_D_ALPHA = 0.15f;
#endif

#if USE_MADGWICK_AHRS
// ---- Madgwick quaternion state (Port B from AIO32) ----
float mw_q0 = 1.0f, mw_q1 = 0.0f, mw_q2 = 0.0f, mw_q3 = 0.0f;
// Note: mw_beta is kept as a runtime global mirror of config.madgwick_beta.
// loadConfiguration() -> syncRuntimeFromConfig() copies it into mw_beta,
// and configureMadgwickBeta() writes both the config field AND this
// global (so the change takes effect immediately without a save+reload).
float mw_beta = 0.1f;
bool  mw_first_update = true;
#endif

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
uint16_t rc_drive_1_processed = 1500;
uint16_t rc_drive_2_processed = 1500;

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
  Serial.println(F("\n=== D-O Universal Controller v3.4.1 ==="));

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
  Serial.println(F("System ready! Press 'm' for menu, 'h' for help"));

  // Play ready sound
  if (sound_enabled) {
    delay(200);  // Short delay to ensure DFPlayer is ready
    playSound(SOUND_SYSTEM_READY);
  }
}

// ============================================================================
// HELP COMMAND
// ============================================================================

void showHelp() {
  Serial.println(F("\n=== CLI HELP (v3.4) ==="));
  Serial.println(F("\nQuick Commands (available anytime):"));
  Serial.println(F("  m - Open configuration menu"));
  Serial.println(F("  h - Show this help (also: ?)"));
  Serial.println(F("\nConfiguration Menu Options:"));
  Serial.println(F("  1 - Setup Mode (PWM/iBus)"));
  Serial.println(F("  2 - PID Configuration"));
  Serial.println(F("  3 - Adaptive PID Settings"));
  Serial.println(F("  4 - Driving Dynamics"));
  Serial.println(F("  5 - Battery Settings"));
  Serial.println(F("  6 - Sound Settings"));
  Serial.println(F("  7 - Feature Toggles"));
  Serial.println(F("  8 - IMU Calibration"));
  Serial.println(F("  9 - Show Current Status"));
  Serial.println(F("  m - Motor Test & Config"));
  Serial.println(F("  i - IMU Axis Test (live angles)"));
  Serial.println(F("  s - Save and Exit"));
  Serial.println(F("  0 - Exit without Saving"));
  Serial.println(F("\nStartup Commands (within 3 seconds):"));
  Serial.println(F("  m - Enter configuration menu"));
  Serial.println(F("  c - Run IMU calibration"));
  Serial.println();
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
    } else if (cmd == 'h' || cmd == 'H' || cmd == '?') {
      // Show help
      showHelp();
    }
  }

  // Update timing using micros() for precise PID control
  previous_time = current_time;
  current_time = micros();
  elapsed_time = (current_time - previous_time) / 1000000.0;  // Convert microseconds to seconds

  // Read RC inputs
  readRCInputs();
  updateProcessedRCInputs();

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

    case 1:  // Serial RC (iBus or SBUS, chosen by config.rc_protocol)
      initRCSerial();
      initializeDFPlayer();
      sound_enabled = true;
      break;

    default:
      Serial.println(F("Unknown mode - defaulting to iBus"));
      config.setup_mode = 1;
      config.rc_protocol = RC_PROTO_IBUS;
      initRCSerial();
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
// RC SERIAL (iBus / SBUS) — runtime-switchable via menu option 'r'
// ============================================================================
// SBUS electrical note: ATmega2560 UART cannot invert RX in HW, so an
// external inverter (NPN + 2 resistors, or 74HC14) MUST be wired between
// the receiver's SBUS output and Serial1-RX (D19 on the Mega) before
// selecting RC_PROTO_SBUS. Doing this without the inverter results in no
// frame sync — the CLI shows packets_received = 0.
// ============================================================================
void initRCSerial() {
  Serial1.end();  // safe even when Serial1 is not yet open
  sbus_idx = 0;
  sbus_failsafe_active = false;
  for (uint8_t i = 0; i < 16; i++) sbus_channels_us[i] = 1500;

  if (config.rc_protocol == RC_PROTO_SBUS) {
    Serial.println(F("Initializing SBUS (100000 8E2) — needs HW inverter!"));
    Serial1.begin(100000, SERIAL_8E2);
    // Do NOT call IBus.begin() — we bypass the library for SBUS
  } else {
    Serial.print(F("Initializing iBus @ "));
    Serial.print(config.ibus_baudrate);
    Serial.println(F(" baud"));
    Serial1.begin(config.ibus_baudrate);
    IBus.begin(Serial1, IBUSBM_NOTIMER);
  }
}

// ---------------------------------------------------------------------------
// SBUS state machine — call every loop iteration when rc_protocol==SBUS.
// Consumes available bytes from Serial1, assembles 25-byte frames, unpacks
// 16 × 11-bit channels on header match, stores 1000..2000 µs values.
// ---------------------------------------------------------------------------
void sbusLoop() {
  uint32_t now_us = micros();
  while (Serial1.available()) {
    // Inter-byte gap (>=2 ms) resyncs the parser to a fresh header
    if (sbus_idx > 0 && (now_us - sbus_last_byte_us) > 2000) {
      sbus_idx = 0;
    }
    sbus_last_byte_us = now_us;

    uint8_t b = Serial1.read();

    // Wait for 0x0F header at index 0
    if (sbus_idx == 0 && b != 0x0F) continue;

    sbus_buf[sbus_idx++] = b;

    if (sbus_idx == 25) {
      sbus_idx = 0;
      if (sbus_buf[0] != 0x0F) continue;  // re-verify header
      sbusParseFrame();
    }
  }
}

void sbusParseFrame() {
  // Bit-unpack 16 × 11-bit channels from bytes 1..22 (little-endian)
  uint16_t c[16];
  c[0]  = ((sbus_buf[1]     | sbus_buf[2] <<8))                        & 0x07FF;
  c[1]  = ((sbus_buf[2] >>3 | sbus_buf[3] <<5))                        & 0x07FF;
  c[2]  = ((sbus_buf[3] >>6 | sbus_buf[4] <<2 | sbus_buf[5] <<10))     & 0x07FF;
  c[3]  = ((sbus_buf[5] >>1 | sbus_buf[6] <<7))                        & 0x07FF;
  c[4]  = ((sbus_buf[6] >>4 | sbus_buf[7] <<4))                        & 0x07FF;
  c[5]  = ((sbus_buf[7] >>7 | sbus_buf[8] <<1 | sbus_buf[9] <<9))      & 0x07FF;
  c[6]  = ((sbus_buf[9] >>2 | sbus_buf[10]<<6))                        & 0x07FF;
  c[7]  = ((sbus_buf[10]>>5 | sbus_buf[11]<<3))                        & 0x07FF;
  c[8]  = ((sbus_buf[12]    | sbus_buf[13]<<8))                        & 0x07FF;
  c[9]  = ((sbus_buf[13]>>3 | sbus_buf[14]<<5))                        & 0x07FF;
  c[10] = ((sbus_buf[14]>>6 | sbus_buf[15]<<2 | sbus_buf[16]<<10))     & 0x07FF;
  c[11] = ((sbus_buf[16]>>1 | sbus_buf[17]<<7))                        & 0x07FF;
  c[12] = ((sbus_buf[17]>>4 | sbus_buf[18]<<4))                        & 0x07FF;
  c[13] = ((sbus_buf[18]>>7 | sbus_buf[19]<<1 | sbus_buf[20]<<9))      & 0x07FF;
  c[14] = ((sbus_buf[20]>>2 | sbus_buf[21]<<6))                        & 0x07FF;
  c[15] = ((sbus_buf[21]>>5 | sbus_buf[22]<<3))                        & 0x07FF;

  // Flags byte (24th byte, zero-indexed 23): bit 2 = frame-lost, bit 3 = failsafe
  uint8_t flags = sbus_buf[23];
  sbus_failsafe_active = (flags & 0x08) != 0;

  if (sbus_failsafe_active) {
    // Don't overwrite channels on failsafe — let the timeout path in loop()
    // trigger emergency_stop via the last_valid_signal mechanism.
    return;
  }

  // Convert 11-bit raw to 1000..2000 µs: (raw * 5 / 8) + 880
  for (uint8_t i = 0; i < 16; i++) {
    sbus_channels_us[i] = (uint16_t)((int)c[i] * 5 / 8 + 880);
  }
  // Touch last_valid_signal so the existing timeout logic knows we're live
  last_valid_signal = millis();
}

// ---------------------------------------------------------------------------
// Unified channel-read wrapper. All call-sites that previously used
// IBus.readChannel(ch) should use rcReadChannel(ch) so iBus↔SBUS is
// transparent above this line.
// ---------------------------------------------------------------------------
uint16_t rcReadChannel(uint8_t ch) {
  if (config.rc_protocol == RC_PROTO_SBUS) {
    return (ch < 16) ? sbus_channels_us[ch] : 1500;
  }
  return IBus.readChannel(ch);
}

// Drop-in replacement for IBus.loop() — branches to SBUS state machine when
// SBUS is active. Call it wherever IBus.loop() was called.
void rcProtocolLoop() {
  if (config.rc_protocol == RC_PROTO_SBUS) {
    sbusLoop();
  } else {
    IBus.loop();
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
      rcProtocolLoop();
      temp1 = rcReadChannel(CH_DRIVE1);
      temp2 = rcReadChannel(CH_DRIVE2);

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
        rcProtocolLoop();
        rc_drive_1 = rcReadChannel(CH_DRIVE1);
        rc_drive_2 = rcReadChannel(CH_DRIVE2);
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

#if USE_MADGWICK_AHRS
  // Port B: Madgwick 6-DoF AHRS. Raw accel is in LSB; normalise inside
  // the filter so unit doesn't matter (only direction counts for the
  // gradient-descent gravity-reference step).
  updateMadgwickIMU(gyro_angle[0], gyro_angle[1], (gyro_z / 131.0f),
                    (float)acc_x, (float)acc_y, (float)acc_z);
  // Extract pitch and roll from quaternion.
  // Same ZYX Tait-Bryan convention the complementary filter produced.
  {
    float sinp = 2.0f * (mw_q0 * mw_q2 - mw_q3 * mw_q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    total_angle[0] = asin(sinp) * 180.0f / PI;     // pitch around Y (our balance axis)
    total_angle[1] = atan2(2.0f * (mw_q0 * mw_q1 + mw_q2 * mw_q3),
                           1.0f - 2.0f * (mw_q1 * mw_q1 + mw_q2 * mw_q2)) * 180.0f / PI; // roll
  }
#else
  // Legacy complementary filter
  total_angle[0] = 0.98 * (total_angle[0] + gyro_angle[0] * elapsed_time) + 0.02 * accel_angle[0];
  total_angle[1] = 0.98 * (total_angle[1] + gyro_angle[1] * elapsed_time) + 0.02 * accel_angle[1];
#endif

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

#if USE_MADGWICK_AHRS
// ---------------------------------------------------------------------------
// Madgwick 6-DoF AHRS — ported from AIO32 madgwick_ahrs.cpp.
// Original algorithm: S. Madgwick 2010 (x-io.co.uk). BBDroids cross-port
// Apache 2.0.
// Inputs: gyro in deg/s, accel in raw LSB (any unit — normalised inside).
// Uses `elapsed_time` (seconds) from the main loop for integration dt.
// ---------------------------------------------------------------------------
void updateMadgwickIMU(float gx_deg, float gy_deg, float gz_deg,
                       float ax, float ay, float az) {
  const float DEG2RAD = 0.01745329252f;
  float dt = elapsed_time;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;  // safety clamp

  float gx = gx_deg * DEG2RAD;
  float gy = gy_deg * DEG2RAD;
  float gz = gz_deg * DEG2RAD;

  // Rate of change of quaternion from gyro integration
  float qDot1 = 0.5f * (-mw_q1 * gx - mw_q2 * gy - mw_q3 * gz);
  float qDot2 = 0.5f * ( mw_q0 * gx + mw_q2 * gz - mw_q3 * gy);
  float qDot3 = 0.5f * ( mw_q0 * gy - mw_q1 * gz + mw_q3 * gx);
  float qDot4 = 0.5f * ( mw_q0 * gz + mw_q1 * gy - mw_q2 * gx);

  float accNormSq = ax * ax + ay * ay + az * az;
  if (accNormSq > 0.0f) {
    float recipNorm = 1.0f / sqrt(accNormSq);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float _2q0 = 2.0f * mw_q0, _2q1 = 2.0f * mw_q1;
    float _2q2 = 2.0f * mw_q2, _2q3 = 2.0f * mw_q3;
    float _4q0 = 4.0f * mw_q0, _4q1 = 4.0f * mw_q1, _4q2 = 4.0f * mw_q2;
    float _8q1 = 8.0f * mw_q1, _8q2 = 8.0f * mw_q2;
    float q0q0 = mw_q0 * mw_q0, q1q1 = mw_q1 * mw_q1;
    float q2q2 = mw_q2 * mw_q2, q3q3 = mw_q3 * mw_q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * mw_q1 - _2q0 * ay
               - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * mw_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
               - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * mw_q3 - _2q1 * ax + 4.0f * q2q2 * mw_q3 - _2q2 * ay;

    float sNormSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (sNormSq > 0.0f) {
      recipNorm = 1.0f / sqrt(sNormSq);
      s0 *= recipNorm; s1 *= recipNorm;
      s2 *= recipNorm; s3 *= recipNorm;
      qDot1 -= mw_beta * s0;
      qDot2 -= mw_beta * s1;
      qDot3 -= mw_beta * s2;
      qDot4 -= mw_beta * s3;
    }
  }

  mw_q0 += qDot1 * dt;
  mw_q1 += qDot2 * dt;
  mw_q2 += qDot3 * dt;
  mw_q3 += qDot4 * dt;

  float qNormSq = mw_q0 * mw_q0 + mw_q1 * mw_q1 + mw_q2 * mw_q2 + mw_q3 * mw_q3;
  if (qNormSq > 0.0f) {
    float recipNorm = 1.0f / sqrt(qNormSq);
    mw_q0 *= recipNorm; mw_q1 *= recipNorm;
    mw_q2 *= recipNorm; mw_q3 *= recipNorm;
  }
  mw_first_update = false;
}
#endif  // USE_MADGWICK_AHRS

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

int processRCInput(int raw_input) {
  // Clamp to nominal range so the shaped output stays in [RC_MIN, RC_MAX].
  raw_input = constrain(raw_input, RC_MIN, RC_MAX);

  int centered = raw_input - RC_CENTER;

  if (abs(centered) < config.deadband) return RC_CENTER;

  float normalized = centered / 500.0f;
  float curved = normalized * (abs(normalized) * config.expo_factor + (1.0f - config.expo_factor));

  return RC_CENTER + (int)(curved * 500.0f);
}

void updateProcessedRCInputs() {
  rc_drive_1_processed = processRCInput(rc_drive_1);
  rc_drive_2_processed = processRCInput(rc_drive_2);
}

void calculatePID() {
  // Calculate target angle
  float target = config.target_angle;

  if (config.dynamic_angle_enabled) {
    // Keep lean demand aligned with the same deadband/expo shaping as motor mixing.
    float drive_input = ((rc_drive_1_processed + rc_drive_2_processed) / 2.0f - RC_CENTER) / 500.0f;
    target += (drive_input * config.max_lean_angle);
  }

  // Calculate error (apply IMU invert if configured)
  float balance_angle = config.imu_invert ? -total_angle[0] : total_angle[0];
  pid_error = balance_angle - target;

#if USE_PID_CONTROLLER_CLASS
  // ------ Port A: class-style PID with filtered D + anti-windup ------
  // P term
  pid_p = current_kp * pid_error;

  // D term — raw derivative then first-order low-pass
  float d_raw = (pid_error - pidc.prev_error) / elapsed_time;
  pidc.d_filtered = pidc.d_filtered + PIDC_D_ALPHA * (d_raw - pidc.d_filtered);
  pid_d = current_kd * pidc.d_filtered;

  // Unsaturated output (pre-integrator) to detect saturation direction
  float pre_i_output = pid_p + pidc.integrator + pid_d;

  // Back-calculation anti-windup: only integrate when NOT driving the
  // output further into saturation in the same direction as the error.
  bool saturating_high = (pre_i_output >=  1000.0f && pid_error > 0.0f);
  bool saturating_low  = (pre_i_output <= -1000.0f && pid_error < 0.0f);
  if (!saturating_high && !saturating_low) {
    pidc.integrator += current_ki * pid_error * elapsed_time;
    pidc.integrator = constrain(pidc.integrator,
                                -config.max_integral, config.max_integral);
  }
  pid_i = pidc.integrator;

  pid_output = pid_p + pid_i + pid_d;
  pid_output = constrain(pid_output, -1000, 1000);

  pidc.prev_error = pid_error;
#else
  // ------ Legacy v3.4 inline PID (proven) ------
  pid_p = current_kp * pid_error;
  pid_i = pid_i + (current_ki * pid_error * elapsed_time);
  pid_i = constrain(pid_i, -config.max_integral, config.max_integral);
  pid_d = current_kd * ((pid_error - pid_previous_error) / elapsed_time);

  pid_output = pid_p + pid_i + pid_d;
  pid_output = constrain(pid_output, -1000, 1000);
#endif

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
  // Calculate base speeds based on mixing mode
  float base_speed_1, base_speed_2;

  if (config.mixing_mode == 1) {
    // Arcade Mode: CH1=Steering, CH2=Throttle (recommended for standard FlySky config)
    int throttle = map(rc_drive_2_processed, RC_MIN, RC_MAX, -255, 255);  // CH2 = forward/back
    int steering = map(rc_drive_1_processed, RC_MIN, RC_MAX, -255, 255);  // CH1 = left/right
    base_speed_1 = throttle + steering;  // Left motor
    base_speed_2 = throttle - steering;  // Right motor
  } else {
    // Tank Mode: CH1=Motor1, CH2=Motor2 (original behavior)
    base_speed_1 = map(rc_drive_1_processed, RC_MIN, RC_MAX, -255, 255);
    base_speed_2 = map(rc_drive_2_processed, RC_MIN, RC_MAX, -255, 255);
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
  float tilt_angle = config.imu_invert ? -total_angle[0] : total_angle[0];
  float current_tilt = abs(tilt_angle - config.target_angle);
  if (current_tilt > 45.0) {
    motor_speed_1 = 0;
    motor_speed_2 = 0;
    if (!emergency_stop) {
      Serial.println(F("TILT CUTOFF: Motors stopped (>45 degrees)"));
      emergency_stop = true;
    }
  }

  // Apply motor configuration (swap/invert)
  int out_speed_1 = motor_speed_1;
  int out_speed_2 = motor_speed_2;
  bool out_dir_1 = dir1;
  bool out_dir_2 = dir2;

  // Swap motors if configured
  if (config.motor_swap) {
    out_speed_1 = motor_speed_2;
    out_speed_2 = motor_speed_1;
    out_dir_1 = dir2;
    out_dir_2 = dir1;
  }

  // Invert directions if configured
  if (config.motor1_invert) out_dir_1 = !out_dir_1;
  if (config.motor2_invert) out_dir_2 = !out_dir_2;

  // Output to motors
  digitalWrite(DIR1_PIN, out_dir_1 ? HIGH : LOW);
  analogWrite(PWM1_PIN, out_speed_1);
  digitalWrite(DIR2_PIN, out_dir_2 ? HIGH : LOW);
  analogWrite(PWM2_PIN, out_speed_2);
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
    mainbar_pos = rcReadChannel(CH_MAINBAR);
    head1_pos = rcReadChannel(CH_HEAD1);
    head2_pos = rcReadChannel(CH_HEAD2);
    head3_pos = rcReadChannel(CH_HEAD3);
  }
  // Mode 0 (PWM): Would need additional PWM inputs for servos (not implemented)

  // Apply mainbar correction
  if (config.mainbar_correction) {
    float corrected_angle = config.imu_invert ? -total_angle[0] : total_angle[0];
    int actual_angle = corrected_angle - config.target_angle;
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
  bool signal_ok = (millis() - last_valid_signal <= SIGNAL_TIMEOUT);
  bool ever_received = (last_valid_signal != 0);
  is_idle = ever_received && signal_ok && (millis() - last_rc_activity > 3000);
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
  // Tilt warning (apply IMU invert if configured)
  float angle_for_tilt = config.imu_invert ? -total_angle[0] : total_angle[0];
  float tilt = abs(angle_for_tilt - config.target_angle);

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
#if USE_PLAYMP3FOLDER
  // File-by-name: /mp3/0001.mp3 (FAT-order-independent)
  myDFPlayer.playMp3Folder(SOUND_STARTUP);
#else
  // Legacy: FAT-order-dependent; SD card must be written in numerical order
  myDFPlayer.play(SOUND_STARTUP);
#endif
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
    sw_mute = rcReadChannel(CH_SOUND_MUTE);
    sw_mode = rcReadChannel(CH_SOUND_MODE);
    sw_mood = rcReadChannel(CH_SOUND_MOOD);
    sw_squeak = rcReadChannel(CH_SOUND_SQUEAK);
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
#if USE_PLAYMP3FOLDER
  // File-by-name addressing in /mp3/NNNN.mp3 — FAT-order-independent.
  // Same DFRobot library, different iBus command byte (0x12 vs 0x03).
  // SD card layout documented in the user handbook stays unchanged.
  myDFPlayer.playMp3Folder(track);
#else
  // Legacy: needs files copied in numerical order (or DriveSort'd)
  myDFPlayer.play(track);
#endif
  soundState.last_sound_time = millis();
}

// ============================================================================
// CONFIGURATION MENU
// ============================================================================

void configurationMenu() {
  Serial.println(F("\n=== CONFIGURATION MENU ==="));

  while (true) {
    Serial.println(F("\n1. Setup Mode (PWM/Serial)"));
    Serial.println(F("2. PID Configuration"));
    Serial.println(F("3. Adaptive PID Settings"));
    Serial.println(F("4. Driving Dynamics"));
    Serial.println(F("5. Battery Settings"));
    Serial.println(F("6. Sound Settings"));
    Serial.println(F("7. Feature Toggles"));
    Serial.println(F("8. IMU Calibration"));
    Serial.println(F("9. Show Current Status"));
    Serial.println(F("r. RC Protocol (iBus / SBUS)"));
#if USE_MADGWICK_AHRS
    Serial.println(F("b. Madgwick Beta (AHRS tuning)"));
#endif
    Serial.println(F("m. Motor Test & Config"));
    Serial.println(F("i. IMU Axis Test (live angles)"));
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
      case 'r':
      case 'R': configureRCProtocol(); break;
      case 'b':
      case 'B': configureMadgwickBeta(); break;
      case 'm':
      case 'M': motorTestMenu(); break;
      case 'i':
      case 'I': imuTestMenu(); break;
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
  Serial.println(F("1 = Serial RC (iBus or SBUS — choose via menu option 'r')"));
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

// ---------------------------------------------------------------------------
// RC Protocol selection (iBus vs SBUS). Runtime-switchable — re-inits
// Serial1 immediately so the user can test without reboot.
// ---------------------------------------------------------------------------
void configureRCProtocol() {
  Serial.println(F("\n--- RC Protocol ---"));
  Serial.print(F("Current: "));
  Serial.println(config.rc_protocol == RC_PROTO_SBUS ? F("SBUS") : F("iBus"));
  Serial.println();
  Serial.println(F("0 = iBus (FlySky) — default, no inverter needed"));
  Serial.println(F("1 = SBUS (Futaba/FrSky) — REQUIRES EXTERNAL INVERTER"));
  Serial.println();
  Serial.println(F("SBUS wiring: RX-SBUS-out -> inverter -> Mega D19 (Serial1-RX)."));
  Serial.println(F("Without inverter the Mega will not sync (packets=0)."));
  Serial.println();
  Serial.print(F("Select (0/1, anything else = cancel): "));

  while (!Serial.available()) delay(10);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();
  Serial.println(choice);

  uint8_t new_proto = config.rc_protocol;
  if (choice == '0') {
    new_proto = RC_PROTO_IBUS;
  } else if (choice == '1') {
    new_proto = RC_PROTO_SBUS;
  } else {
    Serial.println(F("Cancelled - unchanged"));
    return;
  }

  if (new_proto == config.rc_protocol) {
    Serial.println(F("No change."));
    return;
  }

  config.rc_protocol = new_proto;
  Serial.print(F("Switching to: "));
  Serial.println(new_proto == RC_PROTO_SBUS ? F("SBUS") : F("iBus"));
  Serial.println(F("Re-initialising Serial1..."));
  initRCSerial();
  Serial.println(F("Done. Use menu option 's' to persist, or '0' to discard."));
  Serial.println(F("Check RX traffic with option '9' (Show Status) after saving."));
}

// ---------------------------------------------------------------------------
// Madgwick beta tuning (only relevant when USE_MADGWICK_AHRS compile-flag=1).
// Higher beta = more accel weight = less drift but noisier orientation.
// Lower beta  = more gyro trust = smoother but drifts if gyro bias is off.
// Typical range for D-O on a stand: 0.05..0.15. Editable at runtime; save+exit
// persists. If USE_MADGWICK_AHRS=0 the value is stored but unused.
// ---------------------------------------------------------------------------
void configureMadgwickBeta() {
  Serial.println(F("\n--- Madgwick Beta (AHRS) ---"));
#if USE_MADGWICK_AHRS
  Serial.println(F("Madgwick filter is ACTIVE (USE_MADGWICK_AHRS=1)"));
#else
  Serial.println(F("NOTE: Madgwick is compiled OUT (USE_MADGWICK_AHRS=0)."));
  Serial.println(F("Value is stored but ignored until you recompile with flag=1."));
#endif
  Serial.print(F("Current beta: "));
  Serial.println(config.madgwick_beta, 3);
  Serial.println();
  Serial.println(F("Typical range: 0.02 (smooth/trust gyro) .. 0.30 (snappy/trust accel)"));
  Serial.println(F("D-O recommendation: 0.08 .. 0.15 on a stand, tune from there"));
  config.madgwick_beta = getFloatInput(F("New beta"), config.madgwick_beta, 0.0, 1.0);
#if USE_MADGWICK_AHRS
  mw_beta = config.madgwick_beta;  // take effect immediately
  Serial.println(F("Applied live. Use 's' to persist across reboot."));
#else
  Serial.println(F("Stored. Recompile with USE_MADGWICK_AHRS=1 to use it."));
#endif
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
  config.battery_warning = getFloatInput(F("Warning V (4S, e.g. 13.6)"), config.battery_warning, 10.0, 17.0);
  config.battery_critical = getFloatInput(F("Critical V (4S, e.g. 12.8)"), config.battery_critical, 10.0, 17.0);
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
  Serial.print(F("RC Protocol: "));
  if (config.rc_protocol == RC_PROTO_SBUS) {
    Serial.print(F("SBUS (100000 8E2, needs HW inverter)"));
    if (sbus_failsafe_active) Serial.print(F("  [FAILSAFE]"));
    Serial.println();
  } else {
    Serial.print(F("iBus @ "));
    Serial.print(config.ibus_baudrate);
    Serial.println(F(" baud"));
  }
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

void motorTestMenu() {
  Serial.println(F("\n=== MOTOR TEST & CONFIGURATION ==="));
  Serial.println(F("WARNING: D-O should be on a stand or held safely!"));
  Serial.println(F("Motors will spin during testing!\n"));

  while (true) {
    Serial.println(F("\nCurrent Motor Configuration:"));
    Serial.print(F("  Motor Swap (L<->R): ")); Serial.println(config.motor_swap ? F("YES") : F("NO"));
    Serial.print(F("  Motor 1 Invert:     ")); Serial.println(config.motor1_invert ? F("YES") : F("NO"));
    Serial.print(F("  Motor 2 Invert:     ")); Serial.println(config.motor2_invert ? F("YES") : F("NO"));

    Serial.println(F("\nOptions:"));
    Serial.println(F("1. Test Motor 1 (Left) - Forward"));
    Serial.println(F("2. Test Motor 2 (Right) - Forward"));
    Serial.println(F("3. Test Both Motors - Forward"));
    Serial.println(F("4. Test Both Motors - Backward"));
    Serial.println(F("S. Toggle Motor Swap (Left<->Right)"));
    Serial.println(F("A. Toggle Motor 1 Invert"));
    Serial.println(F("B. Toggle Motor 2 Invert"));
    Serial.println(F("X. Exit Motor Test"));
    Serial.print(F("\nSelect: "));

    while (!Serial.available()) delay(10);
    char choice = Serial.read();
    while (Serial.available()) Serial.read();
    Serial.println(choice);

    // Stop motors before each action
    analogWrite(PWM1_PIN, 0);
    analogWrite(PWM2_PIN, 0);

    switch (choice) {
      case '1': // Test Motor 1
        Serial.println(F("Motor 1 (Left) running FORWARD for 2 seconds..."));
        digitalWrite(DIR1_PIN, config.motor1_invert ? LOW : HIGH);
        analogWrite(PWM1_PIN, 150);
        delay(2000);
        analogWrite(PWM1_PIN, 0);
        Serial.println(F("Done. Did the LEFT wheel spin FORWARD?"));
        Serial.println(F("If not: Try 'S' to swap or 'A' to invert"));
        break;

      case '2': // Test Motor 2
        Serial.println(F("Motor 2 (Right) running FORWARD for 2 seconds..."));
        digitalWrite(DIR2_PIN, config.motor2_invert ? LOW : HIGH);
        analogWrite(PWM2_PIN, 150);
        delay(2000);
        analogWrite(PWM2_PIN, 0);
        Serial.println(F("Done. Did the RIGHT wheel spin FORWARD?"));
        Serial.println(F("If not: Try 'S' to swap or 'B' to invert"));
        break;

      case '3': // Both Forward
        Serial.println(F("Both motors running FORWARD for 2 seconds..."));
        digitalWrite(DIR1_PIN, config.motor1_invert ? LOW : HIGH);
        digitalWrite(DIR2_PIN, config.motor2_invert ? LOW : HIGH);
        analogWrite(PWM1_PIN, 150);
        analogWrite(PWM2_PIN, 150);
        delay(2000);
        analogWrite(PWM1_PIN, 0);
        analogWrite(PWM2_PIN, 0);
        Serial.println(F("Done. Both wheels should have spun FORWARD."));
        break;

      case '4': // Both Backward
        Serial.println(F("Both motors running BACKWARD for 2 seconds..."));
        digitalWrite(DIR1_PIN, config.motor1_invert ? HIGH : LOW);
        digitalWrite(DIR2_PIN, config.motor2_invert ? HIGH : LOW);
        analogWrite(PWM1_PIN, 150);
        analogWrite(PWM2_PIN, 150);
        delay(2000);
        analogWrite(PWM1_PIN, 0);
        analogWrite(PWM2_PIN, 0);
        Serial.println(F("Done. Both wheels should have spun BACKWARD."));
        break;

      case 'S':
      case 's':
        config.motor_swap = !config.motor_swap;
        Serial.print(F("Motor Swap: ")); Serial.println(config.motor_swap ? F("ENABLED") : F("DISABLED"));
        break;

      case 'A':
      case 'a':
        config.motor1_invert = !config.motor1_invert;
        Serial.print(F("Motor 1 Invert: ")); Serial.println(config.motor1_invert ? F("ENABLED") : F("DISABLED"));
        break;

      case 'B':
      case 'b':
        config.motor2_invert = !config.motor2_invert;
        Serial.print(F("Motor 2 Invert: ")); Serial.println(config.motor2_invert ? F("ENABLED") : F("DISABLED"));
        break;

      case 'X':
      case 'x':
        Serial.println(F("Exiting Motor Test..."));
        Serial.println(F("Remember to SAVE configuration in main menu!"));
        return;

      default:
        Serial.println(F("Invalid option"));
    }
  }
}

void imuTestMenu() {
  Serial.println(F("\n=== IMU AXIS TEST ==="));
  Serial.println(F("Angle[0] is used for balancing (front/back tilt)."));
  Serial.println(F("\nExpected behavior when tilting D-O:"));
  Serial.println(F("  FORWARD (nose down) -> Angle[0] should INCREASE"));
  Serial.println(F("  BACKWARD (nose up)  -> Angle[0] should DECREASE"));
  Serial.println(F("\nIf it's reversed: Press 'F' to flip/invert the axis"));

  while (true) {
    Serial.println(F("\n----------------------------------------"));
    Serial.print(F("IMU Axis Invert: "));
    Serial.println(config.imu_invert ? F("ON (front/back FLIPPED)") : F("OFF (normal)"));
    Serial.println(F("----------------------------------------"));
    Serial.println(F("\nOptions:"));
    Serial.println(F("T. Start live angle Test"));
    Serial.println(F("F. Flip/Toggle IMU axis invert"));
    Serial.println(F("X. Exit IMU Test"));
    Serial.print(F("\nSelect: "));

    while (!Serial.available()) delay(10);
    char choice = Serial.read();
    while (Serial.available()) Serial.read();
    Serial.println(choice);

    switch (choice) {
      case 'T':
      case 't':
        Serial.println(F("\nLive IMU data - press any key to stop...\n"));
        delay(300);
        while (Serial.available()) Serial.read();
        {
          unsigned long last_print = 0;
          while (!Serial.available()) {
            updateIMUReadings();
            if (millis() - last_print > 200) {
              float displayed_angle = config.imu_invert ? -total_angle[0] : total_angle[0];
              Serial.print(F("Balance Angle: "));
              if (displayed_angle >= 0) Serial.print(F(" "));
              Serial.print(displayed_angle, 1);
              Serial.print(F("°"));
              if (config.imu_invert) Serial.print(F(" [INV]"));
              Serial.print(F("   Raw: "));
              Serial.print(total_angle[0], 1);
              Serial.print(F("°   Accel: X="));
              Serial.print(acc_x);
              Serial.print(F(" Y="));
              Serial.print(acc_y);
              Serial.print(F(" Z="));
              Serial.println(acc_z);
              last_print = millis();
            }
          }
          while (Serial.available()) Serial.read();
        }
        Serial.println(F("\nTest stopped."));
        break;

      case 'F':
      case 'f':
        config.imu_invert = !config.imu_invert;
        Serial.print(F("IMU Axis Invert: "));
        Serial.println(config.imu_invert ? F("ON - Front/back FLIPPED") : F("OFF - Normal"));
        Serial.println(F("Remember to SAVE in main menu!"));
        break;

      case 'X':
      case 'x':
        Serial.println(F("Exiting IMU Test..."));
        return;

      default:
        Serial.println(F("Invalid option"));
    }
  }
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

  if (temp.magic == 0xD044) {
    config = temp;
    // Defensive clamp: rc_protocol must be 0 or 1. Anything else means
    // EEPROM corruption or cross-version ghost bytes. Silently reset to iBus.
    if (config.rc_protocol > 1) {
      config.rc_protocol = RC_PROTO_IBUS;
    }
    // Clamp Madgwick beta into a sane range (defensive against NaN/corruption)
    if (!(config.madgwick_beta >= 0.0f && config.madgwick_beta <= 1.0f)) {
      config.madgwick_beta = 0.1f;
    }
#if USE_MADGWICK_AHRS
    mw_beta = config.madgwick_beta;  // runtime mirror
#endif
    Serial.println(F("Configuration loaded (v3.4.1 layout)"));
  } else {
    // v3.4.0 (0xD043) and older are NOT field-layout compatible with v3.4.1:
    // `max_acceleration` was dropped from the struct and `rc_protocol`
    // appended. Rather than risk reading shifted bytes into the wrong
    // fields, force a clean defaults reset and persist the new magic.
    // One-shot config-loss on upgrade — documented in CHANGELOG v3.4.1.
    Serial.print(F("EEPROM magic 0x"));
    Serial.print(temp.magic, HEX);
    Serial.println(F(" — not v3.4.1, resetting to defaults"));
    saveConfiguration();
  }
}

void saveConfiguration() {
  EEPROM.put(0, config);
  Serial.println(F("Configuration saved"));
}

// ============================================================================
// End of D-O Universal Controller v3.4.1
// ============================================================================
