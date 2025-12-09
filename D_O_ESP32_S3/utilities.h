/**
 * Utilities for D-O Droid
 * Helper functions, LED patterns, system monitoring
 */

#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>
#include "config.h"
#include <Adafruit_NeoPixel.h>

// System state enum (forward declaration for utilities)
enum SystemState {
    STATE_INIT,
    STATE_CALIBRATING,
    STATE_READY,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_EMERGENCY_STOP
};

// LED patterns
enum LEDPattern {
    LED_PATTERN_SOLID,
    LED_PATTERN_BLINK,
    LED_PATTERN_PULSE,
    LED_PATTERN_RAINBOW,
    LED_PATTERN_CHASE,
    LED_PATTERN_FLASH,
    LED_PATTERN_CUSTOM
};

// LED colors
struct LEDColor {
    uint8_t r, g, b;
    
    LEDColor(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0) : 
        r(red), g(green), b(blue) {}
};

// Predefined colors
const LEDColor LED_OFF(0, 0, 0);
const LEDColor LED_RED(255, 0, 0);
const LEDColor LED_GREEN(0, 255, 0);
const LEDColor LED_BLUE(0, 0, 255);
const LEDColor LED_YELLOW(255, 255, 0);
const LEDColor LED_CYAN(0, 255, 255);
const LEDColor LED_MAGENTA(255, 0, 255);
const LEDColor LED_WHITE(255, 255, 255);
const LEDColor LED_ORANGE(255, 128, 0);
const LEDColor LED_PURPLE(128, 0, 255);

// System info structure
struct SystemInfo {
    float cpuTemp;
    uint32_t freeHeap;
    uint32_t totalHeap;
    uint32_t uptime;
    float cpuUsage;
    uint8_t wifiStrength;
    float batteryVoltage;
    bool sdCardPresent;
};

// Battery monitoring
struct BatteryInfo {
    float voltage;
    float current;
    float percentage;
    bool charging;
    bool lowBattery;
    bool criticalBattery;
};

class Utilities {
private:
    // NeoPixel LED
    Adafruit_NeoPixel* statusLED;
    
    // LED state
    LEDPattern currentPattern;
    LEDColor currentColor;
    uint8_t brightness;
    uint32_t lastLEDUpdate;
    uint8_t animationFrame;
    
    // System monitoring
    uint32_t lastSystemCheck;
    SystemInfo systemInfo;
    BatteryInfo batteryInfo;
    
    // Battery monitoring pins (if available)
    int batteryPin;
    float batteryScale;
    
    // Private methods
    void updateLEDAnimation();
    void updateSystemInfo();
    void updateBatteryInfo();
    float readBatteryVoltage();
    
public:
    Utilities();
    ~Utilities();
    
    // Initialization
    bool begin();
    void end();
    
    // LED control
    void setLED(const LEDColor& color);
    void setLED(uint8_t r, uint8_t g, uint8_t b);
    void setLEDPattern(LEDPattern pattern, const LEDColor& color);
    void setLEDBrightness(uint8_t brightness);
    void turnOffLED();
    
    // LED patterns
    void blinkLED(const LEDColor& color, uint32_t interval = 500);
    void pulseLED(const LEDColor& color, uint32_t period = 2000);
    void rainbowLED(uint32_t speed = 20);
    void chaseLED(const LEDColor& color, uint32_t speed = 100);
    void flashLED(const LEDColor& color, uint8_t count = 3);
    
    // Status indicators
    void showStatus(SystemState state);
    void showError(uint8_t errorCode);
    void showBatteryLevel();
    void showActivity(float intensity); // 0.0 to 1.0
    
    // System monitoring
    SystemInfo getSystemInfo();
    float getCPUTemperature();
    uint32_t getFreeHeap();
    uint32_t getUptime();
    float getCPUUsage();
    
    // Battery monitoring
    void setBatteryPin(int pin, float scale = 1.0f);
    BatteryInfo getBatteryInfo();
    float getBatteryVoltage();
    float getBatteryPercentage();
    bool isLowBattery();
    bool isCriticalBattery();
    
    // Update function - call in main loop
    void update();
    
    // Utility functions
    static void formatTime(uint32_t milliseconds, char* buffer, size_t bufferSize);
    static void formatBytes(size_t bytes, char* buffer, size_t bufferSize);
    static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
    static float constrainFloat(float x, float min, float max);
    static float exponentialFilter(float newValue, float oldValue, float alpha);
    static bool isInRange(float value, float center, float tolerance);
    
    // Debug helpers
    void printSystemInfo();
    void printMemoryInfo();
    void dumpI2CDevices();
    
    // Timing helpers
    class Timer {
    private:
        uint32_t interval;
        uint32_t lastTrigger;
        bool enabled;
        
    public:
        Timer(uint32_t interval_ms = 1000) : 
            interval(interval_ms), lastTrigger(0), enabled(true) {}
            
        bool ready() {
            if (!enabled) return false;
            uint32_t now = millis();
            if (now - lastTrigger >= interval) {
                lastTrigger = now;
                return true;
            }
            return false;
        }
        
        void reset() { lastTrigger = millis(); }
        void setInterval(uint32_t ms) { interval = ms; }
        void enable(bool en = true) { enabled = en; }
    };
    
    // Rate limiter
    class RateLimiter {
    private:
        uint32_t minInterval;
        uint32_t lastCall;
        
    public:
        RateLimiter(uint32_t min_interval_ms = 100) : 
            minInterval(min_interval_ms), lastCall(0) {}
            
        bool canExecute() {
            uint32_t now = millis();
            if (now - lastCall >= minInterval) {
                lastCall = now;
                return true;
            }
            return false;
        }
        
        void setMinInterval(uint32_t ms) { minInterval = ms; }
    };
};

// Global instance
extern Utilities utilities;

#endif // UTILITIES_H