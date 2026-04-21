/**
 * PID Auto-Tuning System for D-O Droid
 * Implements relay feedback method for automatic PID parameter calculation
 */

#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include <Arduino.h>
#include "config.h"

// Auto-tune states
enum AutoTuneState {
    AUTOTUNE_IDLE,
    AUTOTUNE_INIT,
    AUTOTUNE_RELAY,
    AUTOTUNE_CALCULATE,
    AUTOTUNE_COMPLETE,
    AUTOTUNE_ERROR
};

// Auto-tune parameters
struct AutoTuneParams {
    float outputStep;      // Motor output amplitude (0-255)
    float noiseband;       // Noise threshold for switching
    uint16_t lookbackSec;  // How far back to look for peaks
    uint16_t sampleTime;   // Sample time in ms
};

// Auto-tune results
struct AutoTuneResults {
    float Ku;              // Ultimate gain
    float Tu;              // Ultimate period (seconds)
    float Kp;              // Calculated P gain
    float Ki;              // Calculated I gain
    float Kd;              // Calculated D gain
    bool valid;            // Results are valid
};

class PIDAutoTune {
private:
    // State management
    AutoTuneState state;
    uint32_t startTime;
    uint32_t stateTime;
    
    // Parameters
    AutoTuneParams params;
    AutoTuneResults results;
    
    // Working variables
    float setpoint;
    float outputValue;
    float noiseBand;
    bool isMax, isMin;
    float *inputs;
    uint16_t inputCount;
    uint16_t inputIndex;
    
    // Peak detection
    float absMax, absMin;
    float peak1, peak2;
    uint32_t peak1Time, peak2Time;
    uint8_t peakCount;
    float maxOutput, minOutput;
    
    // Oscillation detection
    uint8_t oscillationCount;
    float oscillationAmplitude;
    float oscillationPeriod;
    
    // Methods
    bool detectPeak();
    void calculatePID();
    float filterInput(float input);
    
public:
    PIDAutoTune();
    ~PIDAutoTune();
    
    // Configuration
    void setOutputStep(float step) { params.outputStep = step; }
    void setNoiseBand(float band) { params.noiseband = band; }
    void setLookbackSec(uint16_t sec) { params.lookbackSec = sec; }
    void setSampleTime(uint16_t ms) { params.sampleTime = ms; }
    
    // Control
    bool start(float currentAngle);
    void stop();
    bool update(float currentAngle);
    
    // Status
    AutoTuneState getState() { return state; }
    bool isRunning() { return state != AUTOTUNE_IDLE && state != AUTOTUNE_COMPLETE; }
    bool isComplete() { return state == AUTOTUNE_COMPLETE; }
    float getProgress();
    String getStatusString();
    
    // Results
    AutoTuneResults getResults() { return results; }
    float getOutput() { return outputValue; }
    
    // PID calculation methods
    void calculateZieglerNichols();      // Classic ZN
    void calculateModifiedZN();          // Less aggressive
    void calculateCohenCoon();           // For first-order systems
    void calculatePessen();              // Some overshoot
    void calculateNoOvershoot();         // Conservative
};

// Global instance
extern PIDAutoTune pidAutoTune;

#endif // PID_AUTOTUNE_H