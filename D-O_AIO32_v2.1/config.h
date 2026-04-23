/**
 * D-O AIO32 v2.1 Configuration
 * Controller board: D-O AIO32 v2.1 (Printed-Droid.com)
 * MCU module:       TENSTAR TS-ESP32-S3 (plugs into AIO32 socket)
 *                   - ESP32-S3FH4R2 (4 MB flash, 2 MB PSRAM)
 *                   - 1.14" ST7789 TFT onboard
 *                   - QMI8658C IMU onboard (I2C 0x6B)
 *                   - BMP280 pressure sensor onboard (I2C 0x77)
 *                   - WS2812 NeoPixel onboard
 * AIO32-side:       GY-LSM6DS3 IMU fixed to board (I2C 0x6A, SA0 low)
 *                   Cytron MDD10A motor driver, DFPlayer Mini, 4 servos,
 *                   6-button ADC ladder, 4S battery monitor
 *
 * Version: 2.1 (AIO32 baseline)
 * Date:    2026-04
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

#define DEBUG_MODE 0       // 0 = production, 1 = verbose CHECKPOINT logging
#define SERIAL_BAUD 115200

// ============================================================================
// IMU CONFIGURATION
// ============================================================================

// Two IMUs are present on the AIO32 v2.1:
//   - QMI8658C on TENSTAR module  (I2C 0x6B, mechanically tied to the plugged module)
//   - LSM6DS3  on AIO32 PCB        (I2C 0x6A, fixed to droid frame — preferred for balance)
//
// Current sketch still reads QMI8658C as primary. LSM6DS3 integration (library + fusion or
// switch-over) is a separate task — see docs/ROADMAP.md.

#define USE_QMI8658C       true     // Primary IMU today (TENSTAR onboard)
#define USE_LSM6DS3        true     // Register-driver in imu_handler.cpp, uses LSM6DS3_ADDRESS
#define IMU_FUSION_MODE    false    // TODO: true when both IMUs should be fused

// Madgwick AHRS orientation filter (ported from BBDroids / PaulStoffregen Arduino-Madgwick).
// Quaternion-based sensor fusion with gyro-bias correction — much better pitch/roll
// stability than the simple complementary + Kalman chain we used before. When
// enabled, IMUHandler drives its orientation output from Madgwick instead of the
// per-sensor complementary filter. Requires the "Madgwick" library from the
// Arduino Library Manager (installable via Tools -> Manage Libraries).
#define IMU_USE_MADGWICK   true

// Madgwick gyro-bias correction gain. Our own madgwick_ahrs.h/.cpp exposes
// this at runtime via setBeta() (and via the CLI "imu beta <val>" command);
// this value is only the compile-time seed. 0.1 is the original Madgwick
// paper's default — smooth and PID-friendly. For quick static display
// response (e.g. kippen + pitch-anzeige soll sofort 90° zeigen) set 0.3–0.5
// via CLI temporarily.
#define IMU_MADGWICK_BETA    0.10f

// --- IMU mounting / axis alignment ---------------------------------------
// Each IMU on the board may be mounted in a different orientation relative
// to the droid's logical "up / forward / left" axes. The IMUMount enum
// below captures the common discrete cases — pick per IMU what matches
// your physical installation.
//
// Convention (droid frame, right-handed):
//   X_droid = forward (direction the droid rolls when throttle is pushed)
//   Y_droid = left
//   Z_droid = up
//
// To calibrate on a new board: run the 3-position "debug imu" test
// (flat, pitched 90° forward, rolled 90° right) and pick the enum entry
// whose documented mapping matches your measured accel signs.
enum IMUMount {
    IMU_MOUNT_NORMAL        = 0,  // chip axes align with droid axes
    IMU_MOUNT_ROTATE_Y_180  = 1,  // chip rotated 180° around Y: x,z flipped
    IMU_MOUNT_ROTATE_X_180  = 2,  // chip rotated 180° around X: y,z flipped
    IMU_MOUNT_ROTATE_Z_180  = 3,  // chip rotated 180° around Z: x,y flipped
    IMU_MOUNT_YAW_90_CW     = 4,  // chip yaw-rotated 90° clockwise: x<-y, y<-(-x)
    IMU_MOUNT_YAW_90_CCW    = 5,  // chip yaw-rotated 90° CCW: x<-(-y), y<-x
    // Extend here if ever needed (flip-on-side orientations etc.)
};

// Board-specific mount assignments for AIO32 v2.1. Verified 2026-04-23
// with the 3-position orientation test:
//   QMI: x_droid = -x_chip, y_droid = +y_chip, z_droid = -z_chip   (ROTATE_Y_180)
//   LSM: chip axes align with droid axes                           (NORMAL)
#define IMU_QMI_MOUNT   IMU_MOUNT_ROTATE_Y_180
#define IMU_LSM_MOUNT   IMU_MOUNT_NORMAL

// SensorQMI8658 library returns accelerometer data in units of g (gravity),
// not m/s². The imu_handler.cpp comment used to claim m/s² but that was
// wrong — raw values confirm g. We multiply by G after read so fusion and
// Madgwick see the same SI units as the LSM6DS3 pipeline.
#define QMI_ACCEL_SCALE_G_TO_MS2   true

// I2C addresses
#define QMI8658C_ADDRESS   0x6B     // TENSTAR onboard IMU
#define LSM6DS3_ADDRESS    0x6A     // AIO32 PCB IMU (SA0 low). 0x6B would conflict with QMI.
#define BMP280_ADDRESS     0x77     // TENSTAR onboard pressure sensor (optional)

#define IMU_UPDATE_RATE    100      // Hz

// ============================================================================
// PIN DEFINITIONS - AIO32 v2.1 (external connectors)
// ============================================================================

// --- Motor driver (Cytron MDD10A) ---
#define MOTOR1_DIR_PIN     13
#define MOTOR1_PWM_PIN     11
#define MOTOR2_DIR_PIN     10
#define MOTOR2_PWM_PIN     9

// --- DFPlayer Mini (HardwareSerial on Serial2) ---
#define DFPLAYER_TX_PIN    18       // ESP TX -> DFPlayer RX (via 1 kOhm)
#define DFPLAYER_RX_PIN    17       // ESP RX <- DFPlayer TX

// Gate the sound subsystem. When no DFPlayer is wired up, calling
// soundController.begin() hangs/crashes inside the DFRobotDFPlayerMini
// library's handshake (readFileCounts() blocks before setTimeOut() is
// applied, and the 5x retry loop in initializeDFPlayer() keeps UART2 in
// a half-open state). Keep this `false` during bare-bringup and flip it
// to `true` once the DFPlayer + SD card are physically connected.
#define SOUND_CONTROLLER_ENABLED   true

// --- RC receiver (iBus only) ---
#define IBUS_PIN           2        // Serial input from FlySky iBus-capable receiver

// --- Servos (board labels = FlySky channel numbers 3..6) ---
#define SERVO_MAINBAR_PIN  6        // Board label "Servo 3"  -> iBus CH3 (Mainbar)
#define SERVO_HEAD1_PIN    15       // Board label "Servo 4"  -> iBus CH4 (Head pitch)
#define SERVO_HEAD2_PIN    14       // Board label "Servo 5"  -> iBus CH5 (Head yaw)
#define SERVO_HEAD3_PIN    8        // Board label "Servo 6"  -> iBus CH6 (Head roll)

// --- Button ADC ladders (6 buttons on 2 pins) ---
// Pull-up 10 kOhm to 3.3 V, each button shorts through a resistor to GND:
//   Btn 1/4: 0 Ohm       -> ADC ~0
//   Btn 2/5: 4.7 kOhm    -> ADC ~1300 (12-bit, 11 dB attenuation)
//   Btn 3/6: 15  kOhm    -> ADC ~2450
//   released: pulled high -> ADC ~4095
#define BUTTON_LADDER1_PIN 5        // Buttons 1, 2, 3 (ADC1_CH4)
#define BUTTON_LADDER2_PIN 12       // Buttons 4, 5, 6 (ADC2_CH1)

// ADC thresholds (generous windows, can be auto-calibrated later)
#define BTN_ADC_RELEASED_MIN  3500
#define BTN_ADC_B1_MAX         400
#define BTN_ADC_B2_MIN         900
#define BTN_ADC_B2_MAX        1700
#define BTN_ADC_B3_MIN        2000
#define BTN_ADC_B3_MAX        2900

// --- Battery voltage monitoring ---
// Divider on AIO32:    Battery (+) -- 100 kOhm -- sense node -- 22 kOhm -- GND
// Divider factor:      (100 + 22) / 22 = 5.545
// At 4S full  (16.8 V) -> sense = 3.03 V
// At 4S empty (12.0 V) -> sense = 2.16 V
//
// HARDWARE NOTE:
//   AIO32 v2.2 (current):  divider is routed natively to GPIO 16 (ADC2_CH5).
//                           No jumper needed — plug and play.
//   AIO32 v2.1 (legacy):   silk screen labels the sense node "IO36", but
//                           GPIO 36 on ESP32-S3 is NOT an ADC input. A
//                           solder-jumper from the IO36 pad to GPIO 16 is
//                           required. See BUGFIXES.md #1 for details.
// The firmware works on both revisions without code change since both end
// up sampling GPIO 16.
#define BATTERY_PIN            16       // native on v2.2, solder-jumper on v2.1
#define BATTERY_DIVIDER_FACTOR 5.545f
#define BATTERY_MONITOR_ENABLED false   // enable via menu once hardware is jumpered

// 4S LiPo thresholds (2 x 2S in series)
#define BATTERY_FULL_V         16.8f    // 4.2 V / cell
#define BATTERY_NOMINAL_V      14.8f    // 3.7 V / cell
#define BATTERY_WARN_V         13.6f    // 3.4 V / cell
#define BATTERY_CRITICAL_V     12.8f    // 3.2 V / cell
#define BATTERY_EMPTY_V        12.0f    // 3.0 V / cell

// ============================================================================
// PIN DEFINITIONS - TENSTAR module internal (do not change unless module changes)
// ============================================================================

// I2C bus (shared between TENSTAR onboard QMI8658C/BMP280 and AIO32 LSM6DS3)
#define I2C_SDA_PIN        42
#define I2C_SCL_PIN        41
#define TFT_I2C_POWER_PIN  21       // MUST be HIGH for I2C on TENSTAR

// Display (ST7789, on TENSTAR module)
#define TFT_BL_PIN         45
#define TFT_MISO_PIN       37
#define TFT_MOSI_PIN       35
#define TFT_SCLK_PIN       36
#define TFT_CS_PIN         7
#define TFT_DC_PIN         39
#define TFT_RST_PIN        40

// Status LED (WS2812, on TENSTAR module)
#define NEOPIXEL_PIN       33
#define NEOPIXEL_COUNT     1

// ============================================================================
// iBus CHANNEL MAPPING  —  identical to D-O_ibus_v3.4 (Mega line)
// ============================================================================
//
// Same transmitter / receiver / SD card sound layout works on both Mega- and
// ESP32-S3-based D-O controllers. Do not change without matching the Mega line.

#define CH_DRIVE1          0        // CH1 - steering (arcade) or motor 1 (tank)
#define CH_DRIVE2          1        // CH2 - throttle (arcade) or motor 2 (tank)
#define CH_MAINBAR         2        // CH3 - mainbar servo (must be assigned on FlySky!)
#define CH_HEAD1           3        // CH4 - head pitch
#define CH_HEAD2           4        // CH5 - head yaw
#define CH_HEAD3           5        // CH6 - head roll
#define CH_SOUND_MUTE      6        // CH7 - 2-pos switch
#define CH_SOUND_MODE      7        // CH8 - 2-pos switch
#define CH_SOUND_MOOD      8        // CH9 - 3-pos switch
#define CH_SOUND_SQUEAK    9        // CH10 - 2-pos switch

#define RC_CHANNELS        10
#define RC_MIN_VALUE       1000
#define RC_MID_VALUE       1500
#define RC_MAX_VALUE       2000

// Range-validation gates for raw iBus values. Anything outside this
// window is treated as corrupt / unassigned-channel / library hiccup and
// replaced with RC_MID_VALUE in RCReceiver::update() before it reaches
// the normalizeChannels() step. Mirror of the defensive check the Mega
// v3.4 sketch does inline (see CHANGELOG "RC-Safety-Fixes").
#define RC_MIN_VALID        900
#define RC_MAX_VALID       2100
#define RC_DEADBAND_US     30       // microseconds around center

// ============================================================================
// SOUND CONFIGURATION
// ============================================================================

#define DEFAULT_VOLUME       25     // 0..30

// Track numbers on the SD card (/mp3/NNNN.mp3), same layout as Mega line.
#define SOUND_STARTUP         1
#define SOUND_DEFAULT         2
#define SOUND_GREET_START     3
#define SOUND_GREET_END       5
#define SOUND_NEG_START       6
#define SOUND_NEG_END         9
#define SOUND_POS_START      10
#define SOUND_POS_END        14
#define SOUND_SQUEAK_START   15
#define SOUND_SQUEAK_END     20
#define SOUND_TILT_WARNING   21
#define SOUND_RECOVERY       22
#define SOUND_LOW_BATTERY    23
#define SOUND_IDLE_START     24
#define SOUND_IDLE_END       30
#define SOUND_SYSTEM_READY   31
#define SOUND_SIGNAL_LOST    32

#define MIN_SOUND_INTERVAL   400    // ms between triggers
#define STARTUP_DELAY       2000    // ms before user inputs are accepted
#define DEBOUNCE_DELAY        50    // button debounce (ms)

// --- State-reaction sounds (Mega v3.4 feature-parity) ---------------------
// Tilt-Warning: if the droid's absolute pitch exceeds TILT_WARN_ANGLE,
// play track SOUND_TILT_WARNING once, then cool down for TILT_COOLDOWN_MS
// before another warn can fire. Once pitch recovers below TILT_RECOVER_ANGLE,
// play SOUND_RECOVERY and re-arm the warning state machine.
#define TILT_WARN_ANGLE         15.0f    // deg |pitch| trigger threshold
#define TILT_RECOVER_ANGLE       5.0f    // deg |pitch| arm-recovery threshold
#define TILT_COOLDOWN_MS        5000     // anti-spam between warns
#define STATE_REACTIONS_ENABLED  true    // master flag (runtime-persistent)

// Idle-sounds: if the RC receiver is connected but user input has been idle
// (no stick movement, no mute toggle) for IDLE_DETECT_MS, roll a dice every
// random interval in [IDLE_INTERVAL_MIN, IDLE_INTERVAL_MAX] ms — if the
// probability check passes, play a random track from SOUND_IDLE_START..END.
// When the RC receiver is NOT connected, idle sounds are suppressed so the
// droid doesn't chatter on its own when the transmitter is off.
#define IDLE_DETECT_MS          3000     // how long stick must be still
#define IDLE_INTERVAL_MIN_MS    5000     // min ms between idle dice-rolls
#define IDLE_INTERVAL_MAX_MS   15000     // max ms between idle dice-rolls
#define IDLE_PROBABILITY_PCT      70     // % chance to play when dice rolls
#define IDLE_STICK_DEADZONE      0.05f   // normalized stick below this = "still"
#define IDLE_ACTIONS_ENABLED     true    // master flag (runtime-persistent)

// --- Idle-Servo-Animationen (Mega v3.4-Stil, head-servo Leben bei Stillstand) ---
// Gates identisch zu den idle-Sounds (RC connected + sticks still + kein tilt),
// aber die Dice-Rolls sind unabhängig — Sound und Animation dürfen auch
// gleichzeitig feuern, das wirkt natürlich.
#define IDLE_ANIMATIONS_ENABLED   true       // master flag (runtime-persistent)
#define IDLE_ANIM_INTERVAL_MIN_MS   8000     // min ms between anim dice-rolls
#define IDLE_ANIM_INTERVAL_MAX_MS  20000     // max ms between anim dice-rolls
#define IDLE_ANIM_PROBABILITY_PCT    60      // % chance to play when dice rolls

// Per-anim amplitudes in servo-degrees off the 90° neutral. Subtle — the
// droid should feel alive, not manic. Tune up if you want more drama.
#define IDLE_ANIM_NOD_DEG        12
#define IDLE_ANIM_LOOK_DEG       20
#define IDLE_ANIM_SHAKE_DEG       8
#define IDLE_ANIM_TILT_DEG       12

// Per-anim durations (ms). Kept short so the droid never looks "stuck in
// a move" and the regular RC-driven head control can resume cleanly.
#define IDLE_ANIM_NOD_MS        2000
#define IDLE_ANIM_LOOK_MS       3000
#define IDLE_ANIM_SHAKE_MS      1500
#define IDLE_ANIM_TILT_MS       2500

// ============================================================================
// BALANCE CONTROL (PID)
// ============================================================================

// Defaults tuned for the ESP32-S3 + QMI8658C path. Adjust in menu.
// These are the BASE gains — used as-is when ADAPTIVE_PID_ENABLED is false,
// or as a fallback when the adaptive band values have not been saved yet.
#define DEFAULT_KP         12.0f
#define DEFAULT_KI          0.8f
#define DEFAULT_KD          0.5f

#define PID_OUTPUT_LIMIT   255
#define ANGLE_LIMIT         45.0f   // emergency stop above this tilt (deg)

// --- Driving Dynamics (Mega v3.4 Feature-Parität) ------------------------
// RC-Shaping (Deadband + Expo), Mixing-Mode (Arcade/Tank), Motor-Ramping
// und Dynamic Lean Angle.

// Mixing-Mode: Arcade = CH1 steering + CH2 throttle gemischt auf beide
// Motoren. Tank = CH1 und CH2 direkt auf Motor 1 und Motor 2. Default
// Arcade (was der Mega v3.4 auch als default hat).
#define DRIVE_MODE_ARCADE        0
#define DRIVE_MODE_TANK          1
#define DEFAULT_DRIVE_MODE       DRIVE_MODE_ARCADE

// RC-Shaping: Deadband nullt kleine Stick-Bewegungen um die Mitte, Expo
// gibt eine nichtlineare Kurve (y = (1-e)*x + e*x³) für feinere Kontrolle
// in der Stick-Mitte und aggressivere Reaktion am Anschlag.
#define RC_SHAPING_ENABLED       true
#define RC_DEADBAND              0.05f   // 5 % Deadzone (normalisiert)
#define RC_EXPO                  0.30f   // 30 % expo curve (0=linear, 1=voll-kubisch)

// Motor-Ramping: Low-Pass-Filter auf Motor-Output verhindert ruckartige
// Beschleunigung/Bremsung. Filter-Koeffizient 0..1 — höher = schneller,
// niedriger = sanfter. 0.15 ist Mega v3.4 Default.
#define MOTOR_RAMPING_ENABLED    true
#define MOTOR_RAMP_RATE          0.15f

// Dynamic Lean Angle: Droid lehnt sich beim Fahren nach vorn/hinten
// proportional zum Throttle-Stick. Das entlastet den PID der sonst gegen
// den Droid-Schwerpunkt bei hoher Beschleunigung ankämpfen müsste.
#define DYNAMIC_ANGLE_ENABLED    true
#define MAX_LEAN_ANGLE           3.0f    // degrees at full throttle

// --- Adaptive PID (3-band) — ported from Mega v3.4 ------------------------
// Balance-PID-Verstärkungen ändern sich mit der Fahrtgeschwindigkeit:
//   slow   -> hohe P/D für präzise Stehposition
//   medium -> ausgeglichen fürs mäßige Fahren
//   fast   -> niedrigere P/D gegen Oszillation bei schnellerer Fahrt
// Zwischen den Bändern wird linear interpoliert, damit es keine Sprünge gibt.
// Speed-Proxy ist |rcControl.throttle| (normalisiert 0..1).
#define ADAPTIVE_PID_ENABLED      true

// 3 Sätze Gains (Mega v3.4 Defaults, angepasst an unseren ESP32-Skalierungs-Level)
#define KP_SLOW    15.0f
#define KD_SLOW     0.7f
#define KP_MEDIUM  12.0f
#define KD_MEDIUM   0.5f
#define KP_FAST     9.0f
#define KD_FAST     0.3f

// Ki bleibt konstant über alle Bänder (adaptives Ki wird in der Praxis selten
// sauber — dafür gibt es den einheitlichen DEFAULT_KI).

// Speed-Thresholds auf der normalisierten |throttle|-Skala 0..1
#define ADAPTIVE_THRESH_SLOW   0.20f   // unter diesem Wert: rein slow-Band
#define ADAPTIVE_THRESH_MED    0.50f   // um diesen Wert herum: medium-Band
#define ADAPTIVE_THRESH_FAST   0.80f   // ab diesem Wert: rein fast-Band

// PID-Integral anti-windup (Absolutbetrag). Begrenzt den I-Akku so dass
// der PIDController ihn nicht ewig hochintegriert wenn Ki*dt*err groß wird.
#define PID_I_BOUND       200.0f

// D-Term Low-Pass cutoff für die Butterworth-Glättung im PIDController.
// 20 Hz ist hart genug um Highfrequency-Noise rauszufiltern, schnell genug
// dass die D-Komponente noch auf echte Kipp-Bewegungen reagieren kann.
#define PID_D_CUTOFF_HZ    20.0f

// Complementary filter
#define COMP_FILTER_ALPHA   0.98f

// Kalman filter (if fusion is enabled)
#define KALMAN_Q_ANGLE      0.001f
#define KALMAN_Q_GYRO       0.003f
#define KALMAN_R_ANGLE      0.03f

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================

#define MOTOR_PWM_FREQ     1000     // Hz (LEDC)
#define MOTOR_PWM_RESOLUTION  8     // 8-bit (0..255)
#define MOTOR_DEADZONE       10

// ============================================================================
// DISPLAY
// ============================================================================

// ST7789 native panel: 240 x 135 (rows x columns at rotation 0). The
// Adafruit library init call below uses these hardware dimensions; the
// actual screen width/height shown to the rest of the code is read back
// after setRotation() via tft.width() / tft.height(), so changing
// TFT_ROTATION below is sufficient — no need to swap TFT_WIDTH/HEIGHT.
#define TFT_WIDTH          240
#define TFT_HEIGHT         135

// Display orientation. Pick one of:
//   0  portrait        135 x 240, USB port down
//   1  landscape       240 x 135, USB port right (DEFAULT-alt)
//   2  portrait flipped 135 x 240, USB port up
//   3  landscape flipped 240 x 135, USB port left  (DEFAULT)
// For "90°" rotated from the factory landscape, use 0 or 2 (portrait).
#define TFT_ROTATION         3

#define DISPLAY_UPDATE_INTERVAL    100    // ms, 10 Hz
#define TELEMETRY_UPDATE_INTERVAL  250    // ms, 4 Hz
#define GRAPH_UPDATE_INTERVAL      500    // ms, 2 Hz

#define STATUS_MESSAGE_DURATION   2000    // ms default toast
#define DISPLAY_CHUNK_LINES          5    // chunked redraws

#define HEADER_HEIGHT       20
#define FOOTER_HEIGHT       16
#define MARGIN               4
#define LINE_HEIGHT         12
#define GRAPH_HEIGHT        40

// TFT colors (RGB565)
#define TFT_BLACK   0x0000
#define TFT_WHITE   0xFFFF
#define TFT_RED     0xF800
#define TFT_GREEN   0x07E0
#define TFT_BLUE    0x001F
#define TFT_CYAN    0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW  0xFFE0
#define TFT_ORANGE  0xFDA0

// ============================================================================
// SYSTEM TIMING
// ============================================================================

#define MAIN_LOOP_INTERVAL   10     // ms (100 Hz)
#define TELEMETRY_INTERVAL  100     // ms (10 Hz)
#define STATUS_LED_INTERVAL 500     // ms

// ============================================================================
// BUTTON TIMING
// ============================================================================

#define BUTTON_DEBOUNCE_DELAY     50
#define BUTTON_LONG_PRESS_TIME  1000
#define BUTTON_DOUBLE_CLICK_TIME 300
#define BUTTON_HOLD_REPEAT_TIME  500

// Combos (B1+B2 etc.)
#define BUTTON_COMBO_TIME         500
#define BUTTON_COMBO_TIMEOUT      100
#define BUTTON_COMBO_RESET_TIME  2000
#define BUTTON_COMBO_FACTORY_TIME 3000
#define BUTTON_COMBO_SOUND_TIME   100

// ============================================================================
// BUFFERS
// ============================================================================

#define STATUS_TEXT_BUFFER_SIZE  64
#define DIAGNOSTICS_BUFFER_SIZE 256
#define DEBUG_BUFFER_SIZE       128
#define FORMAT_BUFFER_SIZE       32

// ============================================================================
// DEBUG MACROS
// ============================================================================

#if DEBUG_MODE
  #define DEBUG_BEGIN(x)        Serial.begin(x)
  #define DEBUG_PRINT(x)        Serial.print(x)
  #define DEBUG_PRINTLN(x)      Serial.println(x)
  #define DEBUG_PRINTF(x, ...)  Serial.printf(x, ##__VA_ARGS__)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, ...)
#endif

#endif // CONFIG_H
