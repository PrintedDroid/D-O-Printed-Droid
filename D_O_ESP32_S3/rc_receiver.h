/**
 * RC Receiver Handler for D-O Droid
 * Supports iBus protocol and PWM input
 * Handles 10 channels with failsafe
 */

#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <Arduino.h>
#include "config.h"

// RC protocols supported
enum RCProtocol {
    RC_PROTOCOL_IBUS,
    RC_PROTOCOL_SBUS,
    RC_PROTOCOL_NONE
};

// Channel configuration
#define RC_CHANNELS 10
#define RC_MIN_VALUE 1000
#define RC_MID_VALUE 1500
#define RC_MAX_VALUE 2000
#define RC_DEADZONE 50

// iBus protocol constants
#define IBUS_BUFFSIZE 32
#define IBUS_HEADER1 0x20
#define IBUS_HEADER2 0x40

// RC channel assignments (configurable)
struct RCChannelMap {
    uint8_t steering = 0;    // Channel 1
    uint8_t throttle = 1;    // Channel 2
    uint8_t headTilt = 2;    // Channel 3
    uint8_t headPan = 3;     // Channel 4
    uint8_t mute = 4;        // Channel 5
    uint8_t mode = 5;        // Channel 6
    uint8_t aux1 = 6;        // Channel 7
    uint8_t aux2 = 7;        // Channel 8
    uint8_t aux3 = 8;        // Channel 9
    uint8_t aux4 = 9;        // Channel 10
};

// RC status structure
struct RCStatus {
    bool connected;
    uint32_t lastUpdate;
    uint32_t packetsReceived;
    uint32_t packetsLost;
    uint8_t signalStrength;
    RCProtocol protocol;
    bool failsafe;
};

// RC data structure
struct RCData {
    uint16_t channels[RC_CHANNELS];
    float normalized[RC_CHANNELS];  // -1.0 to 1.0
    bool updated;
};

class RCReceiver {
private:
    // Serial port for iBus
    HardwareSerial* ibusSerial;
    
    // Current protocol
    RCProtocol currentProtocol;
    
    // Channel mapping
    RCChannelMap channelMap;
    
    // Status and data
    RCStatus status;
    RCData data;
    RCData failsafeData;
    
    // iBus buffer
    uint8_t ibusBuffer[IBUS_BUFFSIZE];
    uint8_t ibusIndex;
    
    // Calibration data
    struct {
        uint16_t min[RC_CHANNELS];
        uint16_t max[RC_CHANNELS];
        uint16_t center[RC_CHANNELS];
        bool calibrated;
    } calibration;
    
    // Private methods
    bool readIBus();
    bool parseIBusPacket();
    uint16_t calculateIBusChecksum();
    void normalizeChannels();
    void applyDeadzone(uint8_t channel);
    void applyFailsafe();
    void detectProtocol();
    
public:
    RCReceiver();
    ~RCReceiver();
    
    // Initialization
    bool begin(RCProtocol protocol = RC_PROTOCOL_IBUS);
    void end();
    
    // Protocol selection
    void setProtocol(RCProtocol protocol);
    RCProtocol getProtocol() { return currentProtocol; }
    void autoDetectProtocol();
    
    // Channel mapping
    void setChannelMap(const RCChannelMap& map);
    RCChannelMap getChannelMap() { return channelMap; }
    
    // Main update function
    bool update();
    
    // Get channel data
    uint16_t getChannel(uint8_t channel);
    float getNormalizedChannel(uint8_t channel);
    void getAllChannels(uint16_t* channels);
    void getAllNormalized(float* normalized);
    
    // Named channel access
    float getSteering() { return getNormalizedChannel(channelMap.steering); }
    float getThrottle() { return getNormalizedChannel(channelMap.throttle); }
    float getHeadTilt() { return getNormalizedChannel(channelMap.headTilt); }
    float getHeadPan() { return getNormalizedChannel(channelMap.headPan); }
    bool getMute() { return getChannel(channelMap.mute) > RC_MID_VALUE; }
    uint8_t getMode() { return map(getChannel(channelMap.mode), RC_MIN_VALUE, RC_MAX_VALUE, 0, 2); }
    
    // Status
    RCStatus getStatus() { return status; }
    bool isConnected() { return status.connected && !status.failsafe; }
    uint8_t getSignalStrength() { return status.signalStrength; }
    bool isFailsafe() { return status.failsafe; }
    
    // Calibration
    void startCalibration();
    void updateCalibration();
    void endCalibration();
    void saveCalibration();
    void loadCalibration();
    void resetCalibration();
    bool isCalibrated() { return calibration.calibrated; }
    
    // Failsafe
    void setFailsafeValues(const uint16_t* values);
    void setFailsafeValue(uint8_t channel, uint16_t value);
    void enableFailsafe(bool enable);
    
    // Utilities
    String getStatusString();
    void printChannels();
    void printDiagnostics();
    
    // Advanced features
    void setDeadzone(uint16_t deadzone);
    void setUpdateRate(uint32_t rate);
    void enableExponential(uint8_t channel, float expo);
    void setChannelReverse(uint8_t channel, bool reverse);
    void setChannelTrim(uint8_t channel, int16_t trim);
};

// Global instance
extern RCReceiver rcReceiver;

#endif // RC_RECEIVER_H