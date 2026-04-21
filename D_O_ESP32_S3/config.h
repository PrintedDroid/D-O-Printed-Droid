/**
 * D-O Droid ESP32-S3 Configuration
 * Hardware: TENSTAR TS-ESP32-S3 (Adafruit Feather ESP32-S3 TFT Clone)
 * Version: 1.0
 * 
 * This file contains all pin definitions, configuration parameters,
 * and hardware settings for the D-O droid controller.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

// Debug mode
#define DEBUG_MODE 1
#define SERIAL_BAUD 115200

// IMU Configuration
// Only QMI8658C (onboard) is used - sufficient for self-balancing
#define USE_QMI8658C true      // Internal IMU on TENSTAR ESP32-S3 board

// External IMUs - DISABLED (not needed for D-O)
// #define USE_MPU6050 false   // MPU6050 - not connected
// #define USE_BNO055 false    // BNO055 - has clock stretching issues with ESP32
#define USE_MPU6050 false      // Keep false - not used
#define IMU_FUSION_MODE false  // Single IMU mode - no fusion needed

// IMU I2C Addresses
#define QMI8658C_ADDRESS 0x6B
#define QMI8658_L_SLAVE_ADDRESS 0x6B  // For SensorLib compatibility
// #define MPU6050_ADDRESS 0x68       // Not used
// #define BNO055_ADDRESS 0x28        // Not used (has ESP32 I2C issues)
#define MPU6050_ADDRESS 0x68          // Keep for compile compatibility

// Pressure sensor (optional)
#define BMP280_ADDRESS 0x77

// IMU Update rates (Hz)
#define IMU_UPDATE_RATE 100    // 100Hz = 10ms for QMI8658C

// ============================================================================
// PIN DEFINITIONS - ESP32-S3
// ============================================================================

// Motor Controller (Cytron)
#define MOTOR1_DIR_PIN 13
#define MOTOR1_PWM_PIN 11
#define MOTOR2_DIR_PIN 10
#define MOTOR2_PWM_PIN 9

// DFPlayer Communication
#define DFPLAYER_TX_PIN 18  // ESP32 TX to DFPlayer RX
#define DFPLAYER_RX_PIN 17  // ESP32 RX to DFPlayer TX

// RC Receiver
#define IBUS_PIN 2

// Servo Pins
#define SERVO_MAINBAR_PIN 6
#define SERVO_HEAD1_PIN 15
#define SERVO_HEAD2_PIN 14
#define SERVO_HEAD3_PIN 8

// Input Buttons
#define BUTTON1_PIN 5      // Start/Stop
#define BUTTON2_PIN 12     // Calibrate/Setup
#define BUTTON3_PIN 1      // Display Control (TX0 - Serial still works via USB)
#define BUTTON4_PIN 0      // Debug/Select/Info (Boot pin - safe after boot)

// Button functions
#define BUTTON_LONG_PRESS_TIME 1000  // 1 second for long press
#define BUTTON_COMBO_TIME 500        // Time window for button combinations

// MPU6050 Interrupt
#define MPU6050_INT_PIN 16

// I2C Bus
#define I2C_SDA_PIN 42
#define I2C_SCL_PIN 41
#define TFT_I2C_POWER_PIN 21  // CRITICAL! Must be HIGH for I2C

// Display (ST7789)
#define TFT_BL_PIN 45         // Backlight
#define TFT_MISO_PIN 37
#define TFT_MOSI_PIN 35
#define TFT_SCLK_PIN 36
#define TFT_CS_PIN 7
#define TFT_DC_PIN 39
#define TFT_RST_PIN 40

// Status LED (WS2812)
#define NEOPIXEL_PIN 33
#define NEOPIXEL_COUNT 1

// ============================================================================
// SOUND CONFIGURATION
// ============================================================================

// Volume setting (0-30)
#define DEFAULT_VOLUME 25

// Sound file ranges (track numbers on SD card)
#define SOUND_STARTUP 1
#define SOUND_DEFAULT 2
#define SOUND_GREET_START 3
#define SOUND_GREET_END 5
#define SOUND_NEG_START 6
#define SOUND_NEG_END 9
#define SOUND_POS_START 10
#define SOUND_POS_END 14
#define SOUND_SQUEAK_START 15
#define SOUND_SQUEAK_END 20

// Timing constants
#define MIN_SOUND_INTERVAL 400
#define STARTUP_DELAY 2000
#define DEBOUNCE_DELAY 50      // Button debounce time in ms

// ============================================================================
// RC RECEIVER CONFIGURATION
// ============================================================================

// PWM thresholds (microseconds)
#define PWM_THRESHOLD_LOW 1300
#define PWM_THRESHOLD_MID 1500
#define PWM_THRESHOLD_HIGH 1700
#define PWM_HYSTERESIS 50

// PWM reading timeout
#define PWM_TIMEOUT 5000

// Channel assignments
#define CH_MUTE 1
#define CH_GREET 2
#define CH_MOOD 3
#define CH_SQUEAK 4

// ============================================================================
// BALANCE CONTROL (PID)
// ============================================================================

// PID Default values
#define DEFAULT_KP 12.0f
#define DEFAULT_KI 0.8f
#define DEFAULT_KD 0.5f

// PID Limits
#define PID_OUTPUT_LIMIT 255
#define ANGLE_LIMIT 45.0f    // Maximum tilt before shutdown

// Complementary filter coefficient (0.0 - 1.0)
// Higher = trust gyro more, Lower = trust accelerometer more
#define COMP_FILTER_ALPHA 0.98f

// Kalman filter (if using sensor fusion)
#define KALMAN_Q_ANGLE 0.001f
#define KALMAN_Q_GYRO 0.003f
#define KALMAN_R_ANGLE 0.03f

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================

// Motor PWM Configuration
#define MOTOR_PWM_FREQ 1000   // 1kHz
#define MOTOR_PWM_RESOLUTION 8 // 8-bit (0-255)

// Motor dead zone
#define MOTOR_DEADZONE 10

// ============================================================================
// DISPLAY CONFIGURATION
// ============================================================================

#define TFT_WIDTH 240
#define TFT_HEIGHT 135
#define TFT_ROTATION 3  // Landscape

// Display update rates (ms)
#define DISPLAY_UPDATE_INTERVAL 100  // 10Hz for smooth updates with canvas
#define TELEMETRY_UPDATE_INTERVAL 250 // 4Hz
#define GRAPH_UPDATE_INTERVAL 500    // 2Hz for graphs

// TFT Color definitions (RGB565)
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

#define MAIN_LOOP_INTERVAL 10    // 10ms = 100Hz
#define TELEMETRY_INTERVAL 100   // 100ms = 10Hz
#define STATUS_LED_INTERVAL 500  // 500ms = 2Hz

// ============================================================================
// BUTTON TIMING CONFIGURATION
// ============================================================================

// Button debounce and timing
#define BUTTON_DEBOUNCE_DELAY 50      // Debounce time in ms
#define BUTTON_LONG_PRESS_TIME 1000   // Long press threshold
#define BUTTON_DOUBLE_CLICK_TIME 300  // Double-click window
#define BUTTON_HOLD_REPEAT_TIME 500   // Hold repeat interval
#define BUTTON_COMBO_TIMEOUT 100      // Combo clear timeout

// Button combination timings
#define BUTTON_COMBO_RESET_TIME 2000  // B1+B2 for system reset
#define BUTTON_COMBO_FACTORY_TIME 3000 // B3+B4 for factory reset
#define BUTTON_COMBO_SOUND_TIME 100   // B2+B3 for sound toggle

// ============================================================================
// DISPLAY CONFIGURATION  
// ============================================================================

// Display update parameters
#define DISPLAY_UPDATE_INTERVAL 100    // Normal update rate (ms)
#define DISPLAY_CHUNK_LINES 5          // Lines per update chunk
#define STATUS_MESSAGE_DURATION 2000   // Default status message time
#define TELEMETRY_UPDATE_INTERVAL 250  // Telemetry refresh rate

// Display layout
#define HEADER_HEIGHT 20
#define FOOTER_HEIGHT 16
#define MARGIN 4
#define LINE_HEIGHT 12
#define GRAPH_HEIGHT 40

// ============================================================================
// MEMORY OPTIMIZATION
// ============================================================================

// Buffer sizes
#define STATUS_TEXT_BUFFER_SIZE 64
#define DIAGNOSTICS_BUFFER_SIZE 256
#define DEBUG_BUFFER_SIZE 128
#define FORMAT_BUFFER_SIZE 32

// ============================================================================
// DEBUG MACROS
// ============================================================================

#if DEBUG_MODE
  #define DEBUG_BEGIN(x) Serial.begin(x)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, ...)
#endif

#endif // CONFIG_H