/**
 * Utilities Implementation - CORRECTED VERSION
 * Helper functions and system monitoring
 * 
 * Fixed issues:
 * - Removed all String objects
 * - Optimized LED update frequency
 * - Added null pointer checks
 * - Reduced system monitoring overhead
 */

#include "utilities.h"
#include <Wire.h>

// Helper function for rainbow effect
uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170) {
        WheelPos -= 85;
        return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Global instance
Utilities utilities;

// Constructor
Utilities::Utilities() :
    statusLED(nullptr),
    currentPattern(LED_PATTERN_SOLID),
    currentColor(0, 0, 0),
    brightness(255),
    lastLEDUpdate(0),
    animationFrame(0),
    lastSystemCheck(0),
    batteryPin(-1),
    batteryScale(1.0f) {
    
    memset(&systemInfo, 0, sizeof(SystemInfo));
    memset(&batteryInfo, 0, sizeof(BatteryInfo));
}

// Destructor
Utilities::~Utilities() {
    end();
}

// Initialize utilities
bool Utilities::begin() {
    DEBUG_PRINTLN("Utilities: Initializing...");
    
    // Initialize NeoPixel with null check
    try {
        statusLED = new Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
        
        if (statusLED) {
            statusLED->begin();
            statusLED->setBrightness(brightness);
            statusLED->clear();
            statusLED->show();
            
            DEBUG_PRINTLN("Utilities: NeoPixel initialized");
        } else {
            DEBUG_PRINTLN("Utilities: Failed to create NeoPixel object");
            return false;
        }
    } catch (...) {
        DEBUG_PRINTLN("Utilities: Exception during NeoPixel init");
        statusLED = nullptr;
        return false;
    }
    
    // Initial system check
    updateSystemInfo();
    
    DEBUG_PRINTLN("Utilities: Initialized successfully");
    return true;
}

// Clean up
void Utilities::end() {
    if (statusLED) {
        statusLED->clear();
        statusLED->show();
        delete statusLED;
        statusLED = nullptr;
    }
}

// Main update function
void Utilities::update() {
    uint32_t now = millis();
    
    // Update LED animation
    updateLEDAnimation();
    
    // Update system info less frequently (every 5 seconds instead of 1)
    if (now - lastSystemCheck >= 5000) {
        lastSystemCheck = now;
        updateSystemInfo();
        updateBatteryInfo();
    }
}

// Set LED color
void Utilities::setLED(const LEDColor& color) {
    currentColor = color;
    currentPattern = LED_PATTERN_SOLID;
    
    if (statusLED) {
        statusLED->setPixelColor(0, statusLED->Color(color.r, color.g, color.b));
        statusLED->show();
    }
}

void Utilities::setLED(uint8_t r, uint8_t g, uint8_t b) {
    setLED(LEDColor(r, g, b));
}

// Set LED pattern
void Utilities::setLEDPattern(LEDPattern pattern, const LEDColor& color) {
    currentPattern = pattern;
    currentColor = color;
    animationFrame = 0;
    lastLEDUpdate = millis(); // Reset timing
}

// Set LED brightness
void Utilities::setLEDBrightness(uint8_t bright) {
    brightness = bright;
    if (statusLED) {
        statusLED->setBrightness(brightness);
        statusLED->show();
    }
}

// Turn off LED
void Utilities::turnOffLED() {
    setLED(LED_OFF);
}

// Update LED animation - Optimized
void Utilities::updateLEDAnimation() {
    if (!statusLED) return;
    
    uint32_t now = millis();
    uint32_t updateInterval = 20; // Default 50Hz update
    
    switch (currentPattern) {
        case LED_PATTERN_SOLID:
            // Nothing to animate
            return;
            
        case LED_PATTERN_BLINK:
            updateInterval = 500; // 2Hz for blink
            if (now - lastLEDUpdate >= updateInterval) {
                lastLEDUpdate = now;
                animationFrame = !animationFrame;
                
                if (animationFrame) {
                    statusLED->setPixelColor(0, statusLED->Color(currentColor.r, currentColor.g, currentColor.b));
                } else {
                    statusLED->setPixelColor(0, 0);
                }
                statusLED->show();
            }
            break;
            
        case LED_PATTERN_PULSE:
            updateInterval = 50; // 20Hz for smooth pulse
            if (now - lastLEDUpdate >= updateInterval) {
                lastLEDUpdate = now;
                animationFrame = (animationFrame + 2) % 100; // Slower increment
                
                // Sine wave brightness
                float brightness = (sin(animationFrame * 0.0628) + 1.0) / 2.0;
                uint8_t r = currentColor.r * brightness;
                uint8_t g = currentColor.g * brightness;
                uint8_t b = currentColor.b * brightness;
                
                statusLED->setPixelColor(0, statusLED->Color(r, g, b));
                statusLED->show();
            }
            break;
            
        case LED_PATTERN_RAINBOW:
            updateInterval = 50; // 20Hz for rainbow
            if (now - lastLEDUpdate >= updateInterval) {
                lastLEDUpdate = now;
                animationFrame = (animationFrame + 2) % 256; // Slower rotation
                
                // Rainbow cycle
                uint32_t color = Wheel(animationFrame);
                statusLED->setPixelColor(0, color);
                statusLED->show();
            }
            break;
            
        case LED_PATTERN_CHASE:
            updateInterval = 100; // 10Hz for chase
            if (now - lastLEDUpdate >= updateInterval) {
                lastLEDUpdate = now;
                animationFrame = (animationFrame + 1) % 10;
                
                // Flash on every 3rd frame
                if (animationFrame % 3 == 0) {
                    statusLED->setPixelColor(0, statusLED->Color(currentColor.r, currentColor.g, currentColor.b));
                } else {
                    statusLED->setPixelColor(0, 0);
                }
                statusLED->show();
            }
            break;
    }
}

// LED pattern functions
void Utilities::blinkLED(const LEDColor& color, uint32_t interval) {
    setLEDPattern(LED_PATTERN_BLINK, color);
}

void Utilities::pulseLED(const LEDColor& color, uint32_t period) {
    setLEDPattern(LED_PATTERN_PULSE, color);
}

void Utilities::rainbowLED(uint32_t speed) {
    setLEDPattern(LED_PATTERN_RAINBOW, LED_WHITE);
}

void Utilities::chaseLED(const LEDColor& color, uint32_t speed) {
    setLEDPattern(LED_PATTERN_CHASE, color);
}

void Utilities::flashLED(const LEDColor& color, uint8_t count) {
    // Quick flash sequence
    for (uint8_t i = 0; i < count; i++) {
        setLED(color);
        delay(100);
        turnOffLED();
        delay(100);
    }
}

// Show status with LED
void Utilities::showStatus(SystemState state) {
    switch (state) {
        case STATE_INIT:
            setLED(LED_YELLOW);
            break;
        case STATE_READY:
            setLED(LED_GREEN);
            break;
        case STATE_RUNNING:
            pulseLED(LED_BLUE, 2000);
            break;
        case STATE_CALIBRATING:
            blinkLED(LED_YELLOW, 200);
            break;
        case STATE_ERROR:
            blinkLED(LED_RED, 500);
            break;
        case STATE_EMERGENCY_STOP:
            setLED(LED_RED);
            break;
    }
}

// Show error code
void Utilities::showError(uint8_t errorCode) {
    // Flash red LED with error code
    flashLED(LED_RED, errorCode);
}

// Show battery level
void Utilities::showBatteryLevel() {
    if (batteryInfo.criticalBattery) {
        blinkLED(LED_RED, 200);
    } else if (batteryInfo.lowBattery) {
        setLED(LED_ORANGE);
    } else if (batteryInfo.percentage > 75) {
        setLED(LED_GREEN);
    } else if (batteryInfo.percentage > 50) {
        setLED(LED_YELLOW);
    } else {
        setLED(LED_ORANGE);
    }
}

// Show activity
void Utilities::showActivity(float intensity) {
    intensity = constrain(intensity, 0.0f, 1.0f);
    uint8_t brightness = 50 + (intensity * 205);
    setLED(0, 0, brightness);
}

// Update system info - Optimized
void Utilities::updateSystemInfo() {
    systemInfo.uptime = millis();
    systemInfo.freeHeap = ESP.getFreeHeap();
    systemInfo.totalHeap = ESP.getHeapSize();
    
    // CPU temperature - ESP32-S3 doesn't have internal temp sensor
    systemInfo.cpuTemp = 0; // Not available
    
    // Simple CPU usage estimation - removed loop counter to reduce overhead
    systemInfo.cpuUsage = 0; // Simplified
}

// Update battery info
void Utilities::updateBatteryInfo() {
    if (batteryPin < 0) return;
    
    batteryInfo.voltage = readBatteryVoltage();
    
    // Calculate percentage (assuming LiPo battery)
    // 4.2V = 100%, 3.0V = 0%
    batteryInfo.percentage = mapFloat(batteryInfo.voltage, 3.0f, 4.2f, 0.0f, 100.0f);
    batteryInfo.percentage = constrainFloat(batteryInfo.percentage, 0.0f, 100.0f);
    
    // Battery status
    batteryInfo.lowBattery = (batteryInfo.voltage < 3.3f);
    batteryInfo.criticalBattery = (batteryInfo.voltage < 3.1f);
    
    // Charging detection would require additional hardware
    batteryInfo.charging = false;
}

// Read battery voltage
float Utilities::readBatteryVoltage() {
    if (batteryPin < 0) return 0.0f;
    
    // Read ADC and convert to voltage
    int adcValue = analogRead(batteryPin);
    float voltage = (adcValue / 4095.0f) * 3.3f * batteryScale;
    
    // Apply exponential filter for stability
    static float filteredVoltage = 0.0f;
    static bool firstReading = true;
    
    if (firstReading) {
        filteredVoltage = voltage;
        firstReading = false;
    } else {
        filteredVoltage = exponentialFilter(voltage, filteredVoltage, 0.1f);
    }
    
    return filteredVoltage;
}

// Set battery monitoring pin
void Utilities::setBatteryPin(int pin, float scale) {
    batteryPin = pin;
    batteryScale = scale;
    if (pin >= 0) {
        pinMode(batteryPin, INPUT);
    }
}

// Get system info
SystemInfo Utilities::getSystemInfo() {
    return systemInfo;
}

// Get battery info
BatteryInfo Utilities::getBatteryInfo() {
    return batteryInfo;
}

// System monitoring functions
float Utilities::getCPUTemperature() {
    return systemInfo.cpuTemp;
}

uint32_t Utilities::getFreeHeap() {
    return ESP.getFreeHeap();
}

uint32_t Utilities::getUptime() {
    return millis();
}

float Utilities::getCPUUsage() {
    return systemInfo.cpuUsage;
}

float Utilities::getBatteryVoltage() {
    return batteryInfo.voltage;
}

float Utilities::getBatteryPercentage() {
    return batteryInfo.percentage;
}

bool Utilities::isLowBattery() {
    return batteryInfo.lowBattery;
}

bool Utilities::isCriticalBattery() {
    return batteryInfo.criticalBattery;
}

// Utility functions - CORRECTED WITHOUT String
void Utilities::formatTime(uint32_t milliseconds, char* buffer, size_t bufferSize) {
    uint32_t seconds = milliseconds / 1000;
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;
    uint32_t days = hours / 24;
    
    if (days > 0) {
        snprintf(buffer, bufferSize, "%lud %luh", days, hours % 24);
    } else if (hours > 0) {
        snprintf(buffer, bufferSize, "%luh %lum", hours, minutes % 60);
    } else if (minutes > 0) {
        snprintf(buffer, bufferSize, "%lum %lus", minutes, seconds % 60);
    } else {
        snprintf(buffer, bufferSize, "%lus", seconds);
    }
}

void Utilities::formatBytes(size_t bytes, char* buffer, size_t bufferSize) {
    if (bytes < 1024) {
        snprintf(buffer, bufferSize, "%zu B", bytes);
    } else if (bytes < (1024 * 1024)) {
        snprintf(buffer, bufferSize, "%.2f KB", bytes / 1024.0);
    } else if (bytes < (1024 * 1024 * 1024)) {
        snprintf(buffer, bufferSize, "%.2f MB", bytes / 1024.0 / 1024.0);
    } else {
        snprintf(buffer, bufferSize, "%.2f GB", bytes / 1024.0 / 1024.0 / 1024.0);
    }
}

float Utilities::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) return out_min; // Avoid division by zero
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Utilities::constrainFloat(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

float Utilities::exponentialFilter(float newValue, float oldValue, float alpha) {
    return alpha * newValue + (1.0f - alpha) * oldValue;
}

bool Utilities::isInRange(float value, float center, float tolerance) {
    return (value >= center - tolerance) && (value <= center + tolerance);
}

// Debug helpers - CORRECTED WITHOUT String
void Utilities::printSystemInfo() {
    char timeBuffer[32];
    char heapBuffer[32];
    char totalBuffer[32];
    
    DEBUG_PRINTLN("=== System Information ===");
    formatTime(systemInfo.uptime, timeBuffer, sizeof(timeBuffer));
    DEBUG_PRINTF("Uptime: %s\n", timeBuffer);
    
    formatBytes(systemInfo.freeHeap, heapBuffer, sizeof(heapBuffer));
    formatBytes(systemInfo.totalHeap, totalBuffer, sizeof(totalBuffer));
    DEBUG_PRINTF("Free Heap: %s / %s\n", heapBuffer, totalBuffer);
    
    DEBUG_PRINTF("CPU Usage: %.1f%%\n", systemInfo.cpuUsage);
    if (batteryPin >= 0) {
        DEBUG_PRINTF("Battery: %.2fV (%.0f%%)\n", 
                     batteryInfo.voltage, batteryInfo.percentage);
    }
}

void Utilities::printMemoryInfo() {
    DEBUG_PRINTLN("=== Memory Information ===");
    DEBUG_PRINTF("Total heap: %d\n", ESP.getHeapSize());
    DEBUG_PRINTF("Free heap: %d\n", ESP.getFreeHeap());
    DEBUG_PRINTF("Min free heap: %d\n", ESP.getMinFreeHeap());
    DEBUG_PRINTF("Max alloc heap: %d\n", ESP.getMaxAllocHeap());
}

void Utilities::dumpI2CDevices() {
    DEBUG_PRINTLN("=== I2C Device Scan ===");
    DEBUG_PRINTLN("Scanning I2C bus...");
    
    byte count = 0;
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            DEBUG_PRINTF("I2C device found at address 0x%02X", address);
            
            // Identify known devices
            switch(address) {
                case 0x68:
                    DEBUG_PRINT(" (MPU6050)");
                    break;
                case 0x6B:
                    DEBUG_PRINT(" (QMI8658C)");
                    break;
                case 0x77:
                    DEBUG_PRINT(" (BMP280)");
                    break;
            }
            DEBUG_PRINTLN();
            count++;
        }
    }
    
    if (count == 0) {
        DEBUG_PRINTLN("No I2C devices found");
    } else {
        DEBUG_PRINTF("Found %d I2C devices\n", count);
    }
}