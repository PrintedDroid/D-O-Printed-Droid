/**
 * PID Auto-Tuning Implementation
 * Relay feedback method for D-O Droid
 */

#include "pid_autotune.h"
#include "display_handler.h"

// Global instance
PIDAutoTune pidAutoTune;

// Constructor
PIDAutoTune::PIDAutoTune() : 
    state(AUTOTUNE_IDLE),
    startTime(0),
    stateTime(0),
    setpoint(0),
    outputValue(0),
    isMax(true),
    isMin(true),
    inputs(nullptr),
    inputCount(0),
    inputIndex(0),
    absMax(-1000000),
    absMin(1000000),
    peak1(0),
    peak2(0),
    peak1Time(0),
    peak2Time(0),
    peakCount(0),
    maxOutput(0),
    minOutput(0),
    oscillationCount(0),
    oscillationAmplitude(0),
    oscillationPeriod(0) {
    
    // Default parameters
    params.outputStep = 100;     // 100/255 motor power
    params.noiseband = 0.5;      // 0.5 degree noise band
    params.lookbackSec = 10;     // 10 seconds lookback
    params.sampleTime = 10;      // 10ms sample time
    
    // Clear results
    memset(&results, 0, sizeof(AutoTuneResults));
}

// Destructor
PIDAutoTune::~PIDAutoTune() {
    if (inputs) {
        delete[] inputs;
        inputs = nullptr;
    }
}

// Start auto-tuning
bool PIDAutoTune::start(float currentAngle) {
    if (state != AUTOTUNE_IDLE) {
        DEBUG_PRINTLN("AutoTune: Already running!");
        return false;
    }
    
    DEBUG_PRINTLN("AutoTune: Starting auto-tune process");
    
    // Initialize
    state = AUTOTUNE_INIT;
    startTime = millis();
    stateTime = startTime;
    setpoint = currentAngle;  // Use current angle as setpoint
    outputValue = params.outputStep;
    noiseBand = params.noiseband;
    
    // Reset variables
    isMax = true;
    isMin = true;
    absMax = setpoint;
    absMin = setpoint;
    peak1 = setpoint;
    peak2 = setpoint;
    peak1Time = 0;
    peak2Time = 0;
    peakCount = 0;
    oscillationCount = 0;
    
    // Allocate input buffer
    uint16_t samples = (params.lookbackSec * 1000) / params.sampleTime;
    if (inputs) delete[] inputs;
    inputs = new float[samples];
    inputCount = samples;
    inputIndex = 0;
    
    // Fill buffer with current value
    for (uint16_t i = 0; i < inputCount; i++) {
        inputs[i] = setpoint;
    }
    
    // Set initial output
    maxOutput = params.outputStep;
    minOutput = -params.outputStep;
    
    state = AUTOTUNE_RELAY;
    
    displayHandler.showStatus("Auto-Tune Started", TFT_YELLOW);
    
    return true;
}

// Stop auto-tuning
void PIDAutoTune::stop() {
    if (state != AUTOTUNE_IDLE) {
        DEBUG_PRINTLN("AutoTune: Stopped by user");
        state = AUTOTUNE_IDLE;
        outputValue = 0;
        displayHandler.showStatus("Auto-Tune Stopped", TFT_RED);
    }
}

// Update auto-tuning
bool PIDAutoTune::update(float currentAngle) {
    if (state == AUTOTUNE_IDLE || state == AUTOTUNE_COMPLETE) {
        return false;
    }
    
    uint32_t now = millis();
    
    // Store current input
    inputs[inputIndex] = currentAngle;
    inputIndex = (inputIndex + 1) % inputCount;
    
    switch (state) {
        case AUTOTUNE_RELAY:
            // Relay feedback control
            if (currentAngle > setpoint + noiseBand) {
                outputValue = minOutput;
            } else if (currentAngle < setpoint - noiseBand) {
                outputValue = maxOutput;
            }
            
            // Update min/max
            if (currentAngle > absMax) absMax = currentAngle;
            if (currentAngle < absMin) absMin = currentAngle;
            
            // Detect peaks
            if (detectPeak()) {
                DEBUG_PRINTF("AutoTune: Peak detected! Count=%d, Amp=%.2f\n", 
                            peakCount, oscillationAmplitude);
                
                // Need at least 2 full oscillations
                if (peakCount >= 4) {
                    state = AUTOTUNE_CALCULATE;
                    stateTime = now;
                }
            }
            
            // Timeout after 60 seconds
            if (now - startTime > 60000) {
                DEBUG_PRINTLN("AutoTune: Timeout!");
                state = AUTOTUNE_ERROR;
            }
            break;
            
        case AUTOTUNE_CALCULATE:
            calculatePID();
            state = AUTOTUNE_COMPLETE;
            outputValue = 0;
            
            // Show results
            char msg[64];
            snprintf(msg, sizeof(msg), "Kp:%.1f Ki:%.2f Kd:%.2f", 
                    results.Kp, results.Ki, results.Kd);
            displayHandler.showStatus(msg, TFT_GREEN);
            
            DEBUG_PRINTLN("AutoTune: Complete!");
            DEBUG_PRINTF("  Ku=%.3f, Tu=%.3f\n", results.Ku, results.Tu);
            DEBUG_PRINTF("  Kp=%.3f, Ki=%.3f, Kd=%.3f\n", 
                        results.Kp, results.Ki, results.Kd);
            break;
            
        case AUTOTUNE_ERROR:
            outputValue = 0;
            displayHandler.showStatus("Auto-Tune Failed", TFT_RED);
            state = AUTOTUNE_IDLE;
            break;
    }
    
    return true;
}

// Detect oscillation peaks
bool PIDAutoTune::detectPeak() {
    bool peakFound = false;
    
    // Get oldest value from circular buffer
    uint16_t oldestIndex = (inputIndex + 1) % inputCount;
    float oldestValue = inputs[oldestIndex];
    
    // Check for max peak
    if (isMax) {
        if (oldestValue < absMax - noiseBand) {
            // Found a max peak
            isMax = false;
            peak2 = peak1;
            peak2Time = peak1Time;
            peak1 = absMax;
            peak1Time = millis();
            peakCount++;
            peakFound = true;
            
            // Calculate amplitude (peak to peak)
            if (peakCount > 2) {
                oscillationAmplitude = (absMax - absMin) / 2.0f;
            }
        }
    }
    
    // Check for min peak
    if (isMin) {
        if (oldestValue > absMin + noiseBand) {
            // Found a min peak
            isMin = false;
            peakCount++;
            
            // Calculate period if we have two peaks of same type
            if (peak2Time > 0 && peakCount > 3) {
                oscillationPeriod = (float)(peak1Time - peak2Time) / 1000.0f;
                oscillationCount++;
            }
        }
    }
    
    // Reset for next peak
    if (oldestValue > absMax) {
        absMax = oldestValue;
        isMax = true;
    }
    if (oldestValue < absMin) {
        absMin = oldestValue;
        isMin = true;
    }
    
    return peakFound && oscillationCount >= 2;
}

// Calculate PID parameters
void PIDAutoTune::calculatePID() {
    if (oscillationAmplitude <= 0 || oscillationPeriod <= 0) {
        results.valid = false;
        return;
    }
    
    // Calculate ultimate gain (Ku)
    // Ku = 4 * outputStep / (π * amplitude)
    results.Ku = (4.0f * params.outputStep) / (PI * oscillationAmplitude);
    results.Tu = oscillationPeriod * 2.0f;  // Full period
    
    // Default: Modified Ziegler-Nichols for less aggressive response
    calculateModifiedZN();
    
    results.valid = true;
}

// Classic Ziegler-Nichols
void PIDAutoTune::calculateZieglerNichols() {
    results.Kp = 0.6f * results.Ku;
    results.Ki = results.Kp / (results.Tu / 2.0f);
    results.Kd = results.Kp * (results.Tu / 8.0f);
}

// Modified ZN - less aggressive
void PIDAutoTune::calculateModifiedZN() {
    results.Kp = 0.33f * results.Ku;
    results.Ki = results.Kp / results.Tu;
    results.Kd = results.Kp * (results.Tu / 3.0f);
}

// Pessen Integral Rule
void PIDAutoTune::calculatePessen() {
    results.Kp = 0.7f * results.Ku;
    results.Ki = results.Kp / (results.Tu / 2.5f);
    results.Kd = results.Kp * (3.0f * results.Tu / 20.0f);
}

// No overshoot
void PIDAutoTune::calculateNoOvershoot() {
    results.Kp = 0.2f * results.Ku;
    results.Ki = results.Kp / results.Tu;
    results.Kd = results.Kp * (results.Tu / 3.0f);
}

// Cohen-Coon (not applicable for relay method)
void PIDAutoTune::calculateCohenCoon() {
    // Would need different test method
    calculateModifiedZN();
}

// Get progress percentage
float PIDAutoTune::getProgress() {
    if (state == AUTOTUNE_IDLE) return 0.0f;
    if (state == AUTOTUNE_COMPLETE) return 100.0f;
    
    // Estimate based on oscillations found
    float progress = (oscillationCount / 2.0f) * 100.0f;
    return constrain(progress, 0.0f, 95.0f);
}

// Get status string
String PIDAutoTune::getStatusString() {
    switch (state) {
        case AUTOTUNE_IDLE:
            return "Ready";
        case AUTOTUNE_INIT:
            return "Initializing...";
        case AUTOTUNE_RELAY:
            return "Oscillating (" + String(oscillationCount) + "/2)";
        case AUTOTUNE_CALCULATE:
            return "Calculating...";
        case AUTOTUNE_COMPLETE:
            return "Complete!";
        case AUTOTUNE_ERROR:
            return "Error!";
        default:
            return "Unknown";
    }
}