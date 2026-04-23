/**
 * RC Receiver Handler for D-O Droid
 *
 * Supports FlySky iBus and Futaba/FrSky SBUS. Runtime-switchable via CLI
 * (`rc protocol <ibus|sbus>`). Selection persists in NVS.
 *
 * ESP32-S3 advantage: UART RX-invert is a hardware flag, so SBUS works
 * **without** an external inverter. The `begin()` path sets `invert=true`
 * on Serial1 when SBUS is selected.
 *
 * Channel map is identical to the Mega sketch D-O_ibus_v3.4 — same
 * transmitter config works on both controller boards regardless of the
 * wire protocol.
 */

#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <Arduino.h>
#include "config.h"

// Protocol selector. NONE is the pre-begin() state.
enum RCProtocol {
    RC_PROTOCOL_NONE,
    RC_PROTOCOL_IBUS,
    RC_PROTOCOL_SBUS
};

// RC_CHANNELS, RC_MIN_VALUE, RC_MID_VALUE, RC_MAX_VALUE, RC_DEADBAND_US
// come from config.h now (kept aligned with the Mega line).

// iBus protocol constants
#define IBUS_BUFFSIZE 32
#define IBUS_HEADER1 0x20
#define IBUS_HEADER2 0x40

// SBUS protocol constants. 25-byte frame at 100000 8E2, inverted.
// Byte 0: 0x0F header. Bytes 1-22: 16 channels × 11 bits (little-endian,
// bit-packed). Byte 23: flags (bit 0/1 = digital ch17/18, bit 2 = frame
// lost, bit 3 = failsafe). Byte 24: 0x00 footer.
#define SBUS_BUFFSIZE  25
#define SBUS_HEADER    0x0F
#define SBUS_FOOTER    0x00
#define SBUS_FLAG_FAILSAFE   0x08
#define SBUS_FLAG_FRAME_LOST 0x04
// SBUS raw channel values are 0..2047 (11 bit). Convert to 1000..2000 µs
// via (raw * 5 / 8) + 880 — standard formula producing 172->987, 992->1500,
// 1811->2012, mirroring what iBus delivers natively.
#define SBUS_RAW_TO_US(raw) ((int)(raw) * 5 / 8 + 880)

// Mega-compatible channel map (see config.h for the named CH_* constants).
struct RCChannelMap {
    uint8_t drive1  = CH_DRIVE1;        // CH1 — steering (arcade) / motor 1 (tank)
    uint8_t drive2  = CH_DRIVE2;        // CH2 — throttle (arcade) / motor 2 (tank)
    uint8_t mainbar = CH_MAINBAR;       // CH3 — mainbar servo (must be assigned on FlySky!)
    uint8_t head1   = CH_HEAD1;         // CH4 — head pitch
    uint8_t head2   = CH_HEAD2;         // CH5 — head yaw
    uint8_t head3   = CH_HEAD3;         // CH6 — head roll
    uint8_t mute    = CH_SOUND_MUTE;    // CH7
    uint8_t mode    = CH_SOUND_MODE;    // CH8
    uint8_t mood    = CH_SOUND_MOOD;    // CH9
    uint8_t squeak  = CH_SOUND_SQUEAK;  // CH10
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

    // SBUS parser state
    uint8_t sbusBuffer[SBUS_BUFFSIZE];
    uint8_t sbusIndex;
    uint32_t sbusLastByteUs;   // for inter-frame gap detection
    
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
    bool readSBus();
    bool parseSBusPacket();
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

    // NVS persistence of the protocol choice
    static RCProtocol loadStoredProtocol();  // returns RC_PROTOCOL_IBUS if unset
    static void       storeProtocol(RCProtocol p);
    static const char* protocolName(RCProtocol p);
    
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
    
    // Named channel access — Mega-compatible (same channel semantics as v3.4)
    float   getSteering() { return getNormalizedChannel(channelMap.drive1); }   // CH1, arcade steering
    float   getThrottle() { return getNormalizedChannel(channelMap.drive2); }   // CH2, arcade throttle
    float   getDrive1()   { return getNormalizedChannel(channelMap.drive1); }   // CH1 raw (tank mixing)
    float   getDrive2()   { return getNormalizedChannel(channelMap.drive2); }   // CH2 raw (tank mixing)
    float   getMainbar()  { return getNormalizedChannel(channelMap.mainbar); }  // CH3 — mainbar servo
    float   getHead1()    { return getNormalizedChannel(channelMap.head1); }    // CH4 pitch
    float   getHead2()    { return getNormalizedChannel(channelMap.head2); }    // CH5 yaw
    float   getHead3()    { return getNormalizedChannel(channelMap.head3); }    // CH6 roll
    bool    getMute()     { return getChannel(channelMap.mute)   > RC_MID_VALUE; }  // CH7 (2-pos)
    bool    getMode()     { return getChannel(channelMap.mode)   > RC_MID_VALUE; }  // CH8 (2-pos)
    uint8_t getMood() {                                                              // CH9 (3-pos)
        uint16_t v = getChannel(channelMap.mood);
        if (v < 1300) return 0; // negative
        if (v > 1700) return 2; // positive
        return 1;               // mid / neutral
    }
    bool    getSqueak()   { return getChannel(channelMap.squeak) > RC_MID_VALUE; }  // CH10 (2-pos)

    // Legacy aliases (kept to avoid breaking old call-sites; will be removed once
    // the main sketch fully migrates to the Mega-compatible names above).
    float   getHeadTilt() { return getHead1(); }
    float   getHeadPan()  { return getHead2(); }
    
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