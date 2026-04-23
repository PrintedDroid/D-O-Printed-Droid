/**
 * RC Receiver Implementation
 * Handles iBus protocol with failsafe and channel mapping
 */

#include "rc_receiver.h"
#include <Preferences.h>

// Global instance
RCReceiver rcReceiver;

// --- Calibration sanity helper -------------------------------------------
// Guard against persisted or partially-captured calibration that would
// make normalizeChannels() produce nonsense (or worse, divide-by-zero via
// map() with equal endpoints). Required invariants for a valid channel
// calibration: min < center < max AND the full range must exceed
// MIN_CAL_RANGE_US so we actually saw a meaningful stick sweep.
static constexpr uint16_t MIN_CAL_RANGE_US = 300;   // ~30 % of 1000-2000 range
static inline bool isChannelCalibValid(uint16_t mn, uint16_t cn, uint16_t mx) {
    return (mn < cn) && (cn < mx) &&
           ((uint32_t)(mx - mn) >= MIN_CAL_RANGE_US);
}

// Constructor
RCReceiver::RCReceiver() :
    ibusSerial(nullptr),
    currentProtocol(RC_PROTOCOL_NONE),
    ibusIndex(0),
    sbusIndex(0),
    sbusLastByteUs(0) {
    
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
    // Drive channels default to center (= motors idle). Matches the Mega line,
    // which holds drive at RC_CENTER on signal loss rather than forcing throttle low.
    failsafeData.channels[channelMap.drive1] = RC_MID_VALUE;
    failsafeData.channels[channelMap.drive2] = RC_MID_VALUE;
}

// Destructor
RCReceiver::~RCReceiver() {
    end();
}

// Initialize RC receiver
bool RCReceiver::begin(RCProtocol protocol) {
    DEBUG_PRINTLN("RC Receiver: Initializing...");
    
    currentProtocol = protocol;
    ibusIndex = 0;
    sbusIndex = 0;

    switch (protocol) {
        case RC_PROTOCOL_IBUS:
            // iBus: 115200 8N1, non-inverted.
            ibusSerial = &Serial1;
            ibusSerial->begin(115200, SERIAL_8N1, IBUS_PIN, -1, /*invert=*/false);
            DEBUG_PRINTLN("RC Receiver: iBus protocol initialized");
            break;

        case RC_PROTOCOL_SBUS:
            // SBUS: 100000 8E2, inverted signal (UART-level invert handled by
            // ESP32 HW — no external inverter needed on AIO32).
            ibusSerial = &Serial1;
            ibusSerial->begin(100000, SERIAL_8E2, IBUS_PIN, -1, /*invert=*/true);
            DEBUG_PRINTLN("RC Receiver: SBUS protocol initialized (UART invert ON)");
            break;

        default:
            DEBUG_PRINTLN("RC Receiver: protocol not recognised");
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

        case RC_PROTOCOL_SBUS:
            newData = readSBus();
            break;

        default:
            break;
    }

    if (newData) {
        // Range-validation gate (mirrors the Mega v3.4 defensive pattern):
        // any channel outside [RC_MIN_VALID, RC_MAX_VALID] is treated as
        // corrupt — typical causes are an unassigned TX-channel returning
        // 0 or 65535, or a half-parsed iBus frame that slipped through
        // the library. Replace with failsafe value (RC_MID_VALUE for
        // drive/head, lib-internal default for sound switches) so
        // downstream mixing never sees garbage even if the iBus parser
        // hiccups. The regular failsafe-on-timeout path continues to run
        // in the else-branch below.
        for (int i = 0; i < RC_CHANNELS; ++i) {
            if (data.channels[i] < RC_MIN_VALID ||
                data.channels[i] > RC_MAX_VALID) {
                data.channels[i] = failsafeData.channels[i];
            }
        }

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

// ---------------------------------------------------------------------------
// SBUS parser — Futaba / FrSky 25-byte frame state machine
// ---------------------------------------------------------------------------
// Frames arrive at ~7 ms intervals (high speed mode) or ~14 ms (normal).
// A >=3 ms inter-byte gap resyncs the parser. We keep it simple: any gap
// >= 2 ms between bytes resets sbusIndex to look for a fresh header.
//
// We deliberately don't use a library — the parser is ~30 lines of code
// and keeps the RX timing under our control, matches the defensive style
// we already apply to iBus.
// ---------------------------------------------------------------------------
bool RCReceiver::readSBus() {
    if (!ibusSerial || !ibusSerial->available()) {
        return false;
    }

    uint32_t nowUs = micros();
    while (ibusSerial->available()) {
        // Inter-byte gap resync (SBUS frames have a ~3 ms gap between them)
        if (sbusIndex > 0 && (nowUs - sbusLastByteUs) > 2000) {
            sbusIndex = 0;
        }
        sbusLastByteUs = nowUs;

        uint8_t val = ibusSerial->read();

        // Only accept 0x0F at index 0 (frame start)
        if (sbusIndex == 0 && val != SBUS_HEADER) {
            continue;
        }

        if (sbusIndex < SBUS_BUFFSIZE) {
            sbusBuffer[sbusIndex++] = val;
        }

        if (sbusIndex == SBUS_BUFFSIZE) {
            sbusIndex = 0;
            if (parseSBusPacket()) {
                return true;
            } else {
                status.packetsLost++;
            }
        }
    }
    return false;
}

bool RCReceiver::parseSBusPacket() {
    // Validate header + footer. Some receivers use the low nibble of byte 24
    // for telemetry (0x04 = telemetry request) — we accept any value there
    // but require 0x0F as the header.
    if (sbusBuffer[0] != SBUS_HEADER) {
        return false;
    }

    // Unpack 16 × 11-bit channels from bytes 1..22 (little-endian bit stream).
    // We only store the first RC_CHANNELS of them (usually 10 on a D-O setup).
    uint16_t sbusChannels[16];
    sbusChannels[0]  = ((sbusBuffer[1]    | sbusBuffer[2] <<8))                         & 0x07FF;
    sbusChannels[1]  = ((sbusBuffer[2]>>3 | sbusBuffer[3] <<5))                         & 0x07FF;
    sbusChannels[2]  = ((sbusBuffer[3]>>6 | sbusBuffer[4] <<2 | sbusBuffer[5]<<10))     & 0x07FF;
    sbusChannels[3]  = ((sbusBuffer[5]>>1 | sbusBuffer[6] <<7))                         & 0x07FF;
    sbusChannels[4]  = ((sbusBuffer[6]>>4 | sbusBuffer[7] <<4))                         & 0x07FF;
    sbusChannels[5]  = ((sbusBuffer[7]>>7 | sbusBuffer[8] <<1 | sbusBuffer[9]<<9))      & 0x07FF;
    sbusChannels[6]  = ((sbusBuffer[9]>>2 | sbusBuffer[10]<<6))                         & 0x07FF;
    sbusChannels[7]  = ((sbusBuffer[10]>>5| sbusBuffer[11]<<3))                         & 0x07FF;
    sbusChannels[8]  = ((sbusBuffer[12]   | sbusBuffer[13]<<8))                         & 0x07FF;
    sbusChannels[9]  = ((sbusBuffer[13]>>3| sbusBuffer[14]<<5))                         & 0x07FF;
    sbusChannels[10] = ((sbusBuffer[14]>>6| sbusBuffer[15]<<2 | sbusBuffer[16]<<10))    & 0x07FF;
    sbusChannels[11] = ((sbusBuffer[16]>>1| sbusBuffer[17]<<7))                         & 0x07FF;
    sbusChannels[12] = ((sbusBuffer[17]>>4| sbusBuffer[18]<<4))                         & 0x07FF;
    sbusChannels[13] = ((sbusBuffer[18]>>7| sbusBuffer[19]<<1 | sbusBuffer[20]<<9))     & 0x07FF;
    sbusChannels[14] = ((sbusBuffer[20]>>2| sbusBuffer[21]<<6))                         & 0x07FF;
    sbusChannels[15] = ((sbusBuffer[21]>>5| sbusBuffer[22]<<3))                         & 0x07FF;

    // SBUS flags byte. Explicit failsafe bit is a win over iBus.
    uint8_t flags = sbusBuffer[23];
    if (flags & SBUS_FLAG_FAILSAFE) {
        // Receiver is in failsafe — we trust the flag more than stale
        // channel values, so we return false here to let the update()
        // timeout path kick in and apply our own failsafeData.
        DEBUG_PRINTLN("RC Receiver: SBUS failsafe flag set");
        return false;
    }
    if (flags & SBUS_FLAG_FRAME_LOST) {
        // One-off dropped frame — count it but keep last-known values.
        status.packetsLost++;
    }

    // Write into data.channels as microseconds (1000..2000 convention),
    // downstream normalizeChannels() + range-validation gate treat them
    // identically to iBus values.
    for (int i = 0; i < RC_CHANNELS; i++) {
        data.channels[i] = (uint16_t)SBUS_RAW_TO_US(sbusChannels[i]);
    }
    return true;
}

// ---------------------------------------------------------------------------
// NVS persistence for the protocol choice. Kept as static methods so the
// CLI can call them without needing the instance to be alive (and so the
// boot path can read the stored value before constructing the receiver).
// ---------------------------------------------------------------------------
RCProtocol RCReceiver::loadStoredProtocol() {
    Preferences prefs;
    prefs.begin("rc_proto", true);
    uint8_t stored = prefs.getUChar("proto", (uint8_t)RC_PROTOCOL_IBUS);
    prefs.end();
    if (stored == RC_PROTOCOL_SBUS) return RC_PROTOCOL_SBUS;
    return RC_PROTOCOL_IBUS;  // default + unknown fall-through
}

void RCReceiver::storeProtocol(RCProtocol p) {
    Preferences prefs;
    prefs.begin("rc_proto", false);
    prefs.putUChar("proto", (uint8_t)p);
    prefs.end();
}

const char* RCReceiver::protocolName(RCProtocol p) {
    switch (p) {
        case RC_PROTOCOL_IBUS: return "ibus";
        case RC_PROTOCOL_SBUS: return "sbus";
        default:               return "none";
    }
}

// Normalize channels to -1.0 to 1.0
//
// Per-channel defensive guard: even if `calibration.calibrated` is true,
// each channel's individual (min,center,max) triplet is re-validated here
// before use. A channel with nonsensical values (e.g. saved half-swept or
// a corrupted NVS read) silently falls back to the 1000/1500/2000 default
// mapping for THAT channel only — the other channels keep their valid
// calibration. This prevents a single broken channel from poisoning the
// full vehicle.
void RCReceiver::normalizeChannels() {
    for (int i = 0; i < RC_CHANNELS; i++) {
        uint16_t value = data.channels[i];

        bool useCalib = calibration.calibrated &&
                        isChannelCalibValid(calibration.min[i],
                                            calibration.center[i],
                                            calibration.max[i]);

        if (useCalib) {
            if (value < calibration.center[i]) {
                data.normalized[i] = map(value, calibration.min[i],
                                         calibration.center[i], -100, 0) / 100.0f;
            } else {
                data.normalized[i] = map(value, calibration.center[i],
                                         calibration.max[i], 0, 100) / 100.0f;
            }
        } else {
            // Fallback: raw 1000-2000 → -1..+1 mapping
            data.normalized[i] = map(value, RC_MIN_VALUE, RC_MAX_VALUE,
                                     -100, 100) / 100.0f;
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
    float deadzone = RC_DEADBAND_US / 1000.0f; // Convert to normalized units
    
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
//
// Per-channel validity check: a channel is only accepted as calibrated
// if its captured range (max - min) is at least MIN_CAL_RANGE_US wide
// and the implied center is strictly between min and max. Channels that
// were never meaningfully swept during calibration fall back to the
// safe defaults (1000/1500/2000). If ANY channel fails the check we
// also clear the global `calibrated` flag, because a partial calibration
// is worse than no calibration — users will re-run cal rather than fly
// with half-valid values they don't know about.
void RCReceiver::endCalibration() {
    bool allValid = true;

    for (int i = 0; i < RC_CHANNELS; i++) {
        // Compute midpoint as center estimate
        calibration.center[i] = (calibration.min[i] + calibration.max[i]) / 2;

        if (!isChannelCalibValid(calibration.min[i],
                                 calibration.center[i],
                                 calibration.max[i])) {
            DEBUG_PRINTF("RC Receiver: CH%d calibration invalid "
                         "(min=%u center=%u max=%u) — resetting to defaults\n",
                         i + 1, calibration.min[i], calibration.center[i],
                         calibration.max[i]);
            calibration.min[i]    = RC_MIN_VALUE;
            calibration.center[i] = RC_MID_VALUE;
            calibration.max[i]    = RC_MAX_VALUE;
            allValid = false;
        }
    }

    calibration.calibrated = allValid;
    saveCalibration();

    DEBUG_PRINTF("RC Receiver: Calibration %s\n",
                 allValid ? "complete (all channels valid)"
                          : "FAILED — incomplete sweep, run cal again");
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
//
// Defensive load: after pulling the triplets from NVS, every channel is
// validated (min < center < max + meaningful range). If ANY channel fails,
// we reset the entire calibration to safe defaults and clear the
// `calibrated` flag — because a partial-trust calibration is worse than
// none (the user thinks it's calibrated, but some channels behave wrong).
// Individual-channel fallback still happens inside normalizeChannels()
// as a second safety net.
void RCReceiver::loadCalibration() {
    Preferences prefs;
    prefs.begin("rc_calib", true);

    bool flagSaved = prefs.getBool("calibrated", false);

    if (flagSaved) {
        bool allValid = true;
        for (int i = 0; i < RC_CHANNELS; i++) {
            String key = "min" + String(i);
            calibration.min[i] = prefs.getUShort(key.c_str(), RC_MIN_VALUE);
            key = "max" + String(i);
            calibration.max[i] = prefs.getUShort(key.c_str(), RC_MAX_VALUE);
            key = "ctr" + String(i);
            calibration.center[i] = prefs.getUShort(key.c_str(), RC_MID_VALUE);

            if (!isChannelCalibValid(calibration.min[i],
                                     calibration.center[i],
                                     calibration.max[i])) {
                DEBUG_PRINTF("RC Receiver: stored calibration for CH%d is "
                             "invalid (min=%u center=%u max=%u)\n",
                             i + 1, calibration.min[i],
                             calibration.center[i], calibration.max[i]);
                allValid = false;
            }
        }

        if (allValid) {
            calibration.calibrated = true;
        } else {
            DEBUG_PRINTLN("RC Receiver: rejecting stored calibration, "
                          "reverting to safe defaults");
            for (int i = 0; i < RC_CHANNELS; i++) {
                calibration.min[i]    = RC_MIN_VALUE;
                calibration.center[i] = RC_MID_VALUE;
                calibration.max[i]    = RC_MAX_VALUE;
            }
            calibration.calibrated = false;
        }
    } else {
        calibration.calibrated = false;
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
        default:               str += "None\n"; break;
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