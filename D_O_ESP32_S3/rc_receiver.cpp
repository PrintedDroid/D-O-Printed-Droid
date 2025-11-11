/**
 * RC Receiver Implementation
 * Handles iBus protocol with failsafe and channel mapping
 */

#include "rc_receiver.h"
#include <Preferences.h>

// Global instance
RCReceiver rcReceiver;

// Constructor
RCReceiver::RCReceiver() :
    ibusSerial(nullptr),
    currentProtocol(RC_PROTOCOL_NONE),
    ibusIndex(0) {
    
    // Initialize structures
    memset(&status, 0, sizeof(RCStatus));
    memset(&data, 0, sizeof(RCData));
    memset(&failsafeData, 0, sizeof(RCData));
    memset(&calibration, 0, sizeof(calibration));
    
    // Set default failsafe values
    for (int i = 0; i < RC_CHANNELS; i++) {
        failsafeData.channels[i] = RC_MID_VALUE;
        failsafeData.normalized[i] = 0.0f;
    }
    failsafeData.channels[channelMap.throttle] = RC_MIN_VALUE; // Throttle to minimum
}

// Destructor
RCReceiver::~RCReceiver() {
    end();
}

// Initialize RC receiver
bool RCReceiver::begin(RCProtocol protocol) {
    DEBUG_PRINTLN("RC Receiver: Initializing...");
    
    currentProtocol = protocol;
    
    switch (protocol) {
        case RC_PROTOCOL_IBUS:
            // Initialize iBus serial
            ibusSerial = &Serial1;
            ibusSerial->begin(115200, SERIAL_8N1, IBUS_PIN, -1);
            DEBUG_PRINTLN("RC Receiver: iBus protocol initialized");
            break;
            
        case RC_PROTOCOL_SBUS:
            DEBUG_PRINTLN("RC Receiver: SBUS protocol not yet implemented");
            return false;
            
        default:
            DEBUG_PRINTLN("RC Receiver: No protocol selected");
            return false;
    }
    
    // Load calibration
    loadCalibration();
    
    // Set initial status
    status.protocol = protocol;
    status.connected = false;
    status.failsafe = true;
    
    DEBUG_PRINTLN("RC Receiver: Initialized successfully");
    return true;
}

// End RC receiver
void RCReceiver::end() {
    if (ibusSerial) {
        ibusSerial->end();
        ibusSerial = nullptr;
    }
    currentProtocol = RC_PROTOCOL_NONE;
}

// Main update function
bool RCReceiver::update() {
    bool newData = false;
    
    switch (currentProtocol) {
        case RC_PROTOCOL_IBUS:
            newData = readIBus();
            break;
            
        default:
            break;
    }
    
    if (newData) {
        // Normalize channels
        normalizeChannels();
        
        // Update status
        status.lastUpdate = millis();
        status.packetsReceived++;
        status.connected = true;
        status.failsafe = false;
        
        // Calculate signal strength (simple method based on packet rate)
        static uint32_t lastStrengthCalc = 0;
        static uint32_t packetsInSecond = 0;
        packetsInSecond++;
        
        if (millis() - lastStrengthCalc >= 1000) {
            // Expected ~50Hz update rate for iBus
            status.signalStrength = constrain(map(packetsInSecond, 0, 50, 0, 100), 0, 100);
            packetsInSecond = 0;
            lastStrengthCalc = millis();
        }
        
        data.updated = true;
    } else {
        // Check for timeout
        if (millis() - status.lastUpdate > 1000) {
            status.connected = false;
            status.failsafe = true;
            applyFailsafe();
        }
    }
    
    return newData;
}

// Read iBus data
bool RCReceiver::readIBus() {
    if (!ibusSerial || !ibusSerial->available()) {
        return false;
    }
    
    while (ibusSerial->available()) {
        uint8_t val = ibusSerial->read();
        
        // Look for start of packet
        if (ibusIndex == 0 && val != IBUS_HEADER1) {
            continue;
        }
        if (ibusIndex == 1 && val != IBUS_HEADER2) {
            ibusIndex = 0;
            continue;
        }
        
        // Store byte
        if (ibusIndex < IBUS_BUFFSIZE) {
            ibusBuffer[ibusIndex] = val;
            ibusIndex++;
        }
        
        // Check if we have a complete packet
        if (ibusIndex == IBUS_BUFFSIZE) {
            ibusIndex = 0;
            
            // Verify checksum
            if (parseIBusPacket()) {
                return true;
            } else {
                status.packetsLost++;
            }
        }
    }
    
    return false;
}

// Parse iBus packet
bool RCReceiver::parseIBusPacket() {
    // Calculate checksum
    uint16_t checksum = calculateIBusChecksum();
    uint16_t rxChecksum = ibusBuffer[30] | (ibusBuffer[31] << 8);
    
    if (checksum != rxChecksum) {
        DEBUG_PRINTLN("RC Receiver: iBus checksum error");
        return false;
    }
    
    // Extract channel data
    for (int i = 0; i < RC_CHANNELS; i++) {
        data.channels[i] = ibusBuffer[2 + i*2] | (ibusBuffer[3 + i*2] << 8);
        
        // Validate channel value
        if (data.channels[i] < 900 || data.channels[i] > 2100) {
            DEBUG_PRINTF("RC Receiver: Invalid channel %d value: %d\n", i, data.channels[i]);
            return false;
        }
    }
    
    return true;
}

// Calculate iBus checksum
uint16_t RCReceiver::calculateIBusChecksum() {
    uint16_t checksum = 0xFFFF;
    
    for (int i = 0; i < 30; i++) {
        checksum -= ibusBuffer[i];
    }
    
    return checksum;
}

// Normalize channels to -1.0 to 1.0
void RCReceiver::normalizeChannels() {
    for (int i = 0; i < RC_CHANNELS; i++) {
        uint16_t value = data.channels[i];
        
        if (calibration.calibrated) {
            // Use calibration data
            if (value < calibration.center[i]) {
                data.normalized[i] = map(value, calibration.min[i], calibration.center[i], -100, 0) / 100.0f;
            } else {
                data.normalized[i] = map(value, calibration.center[i], calibration.max[i], 0, 100) / 100.0f;
            }
        } else {
            // Use default mapping
            data.normalized[i] = map(value, RC_MIN_VALUE, RC_MAX_VALUE, -100, 100) / 100.0f;
        }
        
        // Apply deadzone
        applyDeadzone(i);
        
        // Constrain to valid range
        data.normalized[i] = constrain(data.normalized[i], -1.0f, 1.0f);
    }
}

// Apply deadzone to channel
void RCReceiver::applyDeadzone(uint8_t channel) {
    float value = data.normalized[channel];
    float deadzone = RC_DEADZONE / 1000.0f; // Convert to normalized units
    
    if (abs(value) < deadzone) {
        data.normalized[channel] = 0.0f;
    } else {
        // Rescale to remove deadzone gap
        if (value > 0) {
            data.normalized[channel] = (value - deadzone) / (1.0f - deadzone);
        } else {
            data.normalized[channel] = (value + deadzone) / (1.0f - deadzone);
        }
    }
}

// Apply failsafe values
void RCReceiver::applyFailsafe() {
    memcpy(&data, &failsafeData, sizeof(RCData));
    data.updated = true;
}

// Get channel value
uint16_t RCReceiver::getChannel(uint8_t channel) {
    if (channel < RC_CHANNELS) {
        return data.channels[channel];
    }
    return RC_MID_VALUE;
}

// Get normalized channel value
float RCReceiver::getNormalizedChannel(uint8_t channel) {
    if (channel < RC_CHANNELS) {
        return data.normalized[channel];
    }
    return 0.0f;
}

// Get all channels
void RCReceiver::getAllChannels(uint16_t* channels) {
    memcpy(channels, data.channels, sizeof(data.channels));
}

// Get all normalized values
void RCReceiver::getAllNormalized(float* normalized) {
    memcpy(normalized, data.normalized, sizeof(data.normalized));
}

// Set protocol
void RCReceiver::setProtocol(RCProtocol protocol) {
    if (protocol != currentProtocol) {
        end();
        begin(protocol);
    }
}

// Auto-detect protocol
void RCReceiver::autoDetectProtocol() {
    // Try each protocol for a short time
    // Not implemented in this version
    DEBUG_PRINTLN("RC Receiver: Auto-detect not implemented");
}

// Set channel map
void RCReceiver::setChannelMap(const RCChannelMap& map) {
    channelMap = map;
}

// Start calibration
void RCReceiver::startCalibration() {
    DEBUG_PRINTLN("RC Receiver: Starting calibration");
    
    // Initialize calibration values
    for (int i = 0; i < RC_CHANNELS; i++) {
        calibration.min[i] = 2000;
        calibration.max[i] = 1000;
        calibration.center[i] = 1500;
    }
    
    calibration.calibrated = false;
}

// Update calibration
void RCReceiver::updateCalibration() {
    if (!status.connected) return;
    
    // Update min/max values
    for (int i = 0; i < RC_CHANNELS; i++) {
        uint16_t value = data.channels[i];
        
        if (value < calibration.min[i]) {
            calibration.min[i] = value;
        }
        if (value > calibration.max[i]) {
            calibration.max[i] = value;
        }
    }
}

// End calibration
void RCReceiver::endCalibration() {
    // Calculate center values
    for (int i = 0; i < RC_CHANNELS; i++) {
        calibration.center[i] = (calibration.min[i] + calibration.max[i]) / 2;
    }
    
    calibration.calibrated = true;
    saveCalibration();
    
    DEBUG_PRINTLN("RC Receiver: Calibration complete");
    printDiagnostics();
}

// Save calibration
void RCReceiver::saveCalibration() {
    Preferences prefs;
    prefs.begin("rc_calib", false);
    
    for (int i = 0; i < RC_CHANNELS; i++) {
        String key = "min" + String(i);
        prefs.putUShort(key.c_str(), calibration.min[i]);
        
        key = "max" + String(i);
        prefs.putUShort(key.c_str(), calibration.max[i]);
        
        key = "ctr" + String(i);
        prefs.putUShort(key.c_str(), calibration.center[i]);
    }
    
    prefs.putBool("calibrated", calibration.calibrated);
    prefs.end();
}

// Load calibration
void RCReceiver::loadCalibration() {
    Preferences prefs;
    prefs.begin("rc_calib", true);
    
    calibration.calibrated = prefs.getBool("calibrated", false);
    
    if (calibration.calibrated) {
        for (int i = 0; i < RC_CHANNELS; i++) {
            String key = "min" + String(i);
            calibration.min[i] = prefs.getUShort(key.c_str(), 1000);
            
            key = "max" + String(i);
            calibration.max[i] = prefs.getUShort(key.c_str(), 2000);
            
            key = "ctr" + String(i);
            calibration.center[i] = prefs.getUShort(key.c_str(), 1500);
        }
    }
    
    prefs.end();
}

// Reset calibration
void RCReceiver::resetCalibration() {
    calibration.calibrated = false;
    
    for (int i = 0; i < RC_CHANNELS; i++) {
        calibration.min[i] = RC_MIN_VALUE;
        calibration.max[i] = RC_MAX_VALUE;
        calibration.center[i] = RC_MID_VALUE;
    }
}

// Set failsafe values
void RCReceiver::setFailsafeValues(const uint16_t* values) {
    memcpy(failsafeData.channels, values, sizeof(failsafeData.channels));
    normalizeChannels(); // Normalize the failsafe values
}

// Set single failsafe value
void RCReceiver::setFailsafeValue(uint8_t channel, uint16_t value) {
    if (channel < RC_CHANNELS) {
        failsafeData.channels[channel] = value;
    }
}

// Get status string
String RCReceiver::getStatusString() {
    String str = "RC Receiver Status:\n";
    str += "Protocol: ";
    
    switch (currentProtocol) {
        case RC_PROTOCOL_IBUS: str += "iBus\n"; break;
        case RC_PROTOCOL_SBUS: str += "SBUS\n"; break;
        default: str += "None\n"; break;
    }
    
    str += "Connected: " + String(status.connected ? "Yes" : "No") + "\n";
    str += "Failsafe: " + String(status.failsafe ? "Active" : "Inactive") + "\n";
    str += "Signal: " + String(status.signalStrength) + "%\n";
    str += "Packets: " + String(status.packetsReceived) + " received, ";
    str += String(status.packetsLost) + " lost\n";
    str += "Calibrated: " + String(calibration.calibrated ? "Yes" : "No") + "\n";
    
    return str;
}

// Print channels
void RCReceiver::printChannels() {
    DEBUG_PRINTLN("RC Channels:");
    for (int i = 0; i < RC_CHANNELS; i++) {
        DEBUG_PRINTF("  CH%d: %d (%.2f)\n", i+1, data.channels[i], data.normalized[i]);
    }
}

// Print diagnostics
void RCReceiver::printDiagnostics() {
    DEBUG_PRINTLN("=== RC Receiver Diagnostics ===");
    DEBUG_PRINTLN(getStatusString());
    
    if (calibration.calibrated) {
        DEBUG_PRINTLN("Calibration data:");
        for (int i = 0; i < RC_CHANNELS; i++) {
            DEBUG_PRINTF("  CH%d: min=%d, center=%d, max=%d\n", 
                        i+1, calibration.min[i], calibration.center[i], calibration.max[i]);
        }
    }
    
    printChannels();
}