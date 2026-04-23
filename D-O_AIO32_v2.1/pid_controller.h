/**
 * PID Controller with anti-windup, deadbands, bounds, ramp, and D-term filtering.
 *
 * Based on BBDroids `bb::PIDController` by Björn Giesler (Apache License 2.0,
 * https://github.com/giesler/BBDroids — `Arduino/LibBB/src/BBControllers.*`).
 * The 2nd-order Butterworth low-pass used for D-term smoothing is adapted from
 * Curio Res's filter tutorial (https://github.com/curiores/ArduinoTutorials).
 *
 * Ported for D-O AIO32 (ESP32-S3) — standalone, no BBDroids runtime needed.
 * Adaptations:
 *   - Direct float update(input) API instead of ControlInput/ControlOutput
 *     virtuals, because our Balance loop has a single sensor source and a
 *     single actuator channel. The full abstraction can be re-introduced if
 *     we add drive-mode velocity/position controllers later.
 *   - Removed BBError/Console/Runloop dependencies.
 *
 * Licensed under Apache License 2.0.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

// --------------------------------------------------------------------------
// 2nd-order Butterworth low-pass filter — standalone reimplementation.
// --------------------------------------------------------------------------
class PidLowPassFilter {
public:
    PidLowPassFilter(float cutoffHz = 5.0f, float sampleFreqHz = 100.0f, bool adaptive = false);

    void  setCutoff(float cutoffHz);
    void  setSampleFrequency(float sampleFreqHz);
    void  setAdaptive(bool adaptive);
    float filter(float xn);
    void  reset();

private:
    void computeCoefficients();
    float a_[2], b_[3];
    float omega0_;
    float dt_;
    bool  adaptive_;
    float tn1_;
    float x_[3];
    float y_[3];
    float cutoffHz_;
    float sampleFreqHz_;
    bool  needsRecalc_;
};

// --------------------------------------------------------------------------
// PIDController — direct float API
//
// Usage:
//   PIDController pid;
//   pid.setGains(Kp, Ki, Kd);
//   pid.setControlBounds(-100.0f, 100.0f);
//   pid.setIBounds(-50.0f, 50.0f);
//   pid.setGoal(0.0f);               // target pitch in degrees
//   ...
//   float u = pid.update(currentPitch);   // returns control output
// --------------------------------------------------------------------------
class PIDController {
public:
    PIDController();

    // Reset accumulated error state (call after mode changes)
    void reset();

    // Main update — pass the current measured value, returns control output.
    // Uses internal micros() timer to compute dt, so just call it every loop.
    float update(float currentValue);

    // --- Gains ---
    void  setGains(float kp, float ki, float kd);
    void  getGains(float& kp, float& ki, float& kd) const;

    // --- Setpoint ---
    void  setGoal(float goal);        // optionally ramps if setRamp > 0
    void  setPresentAsGoal(float currentValue);
    float goal() const { return goal_; }
    float error() const { return lastErr_; }
    float output() const { return lastControl_; }

    // --- Integral anti-windup ---
    void  setIBounds(float iMin, float iMax);
    void  setIUnbounded();
    bool  isIBounded() const { return iBounded_; }

    // --- Control output bounds ---
    void  setControlBounds(float controlMin, float controlMax);
    void  setControlUnbounded();
    bool  isControlBounded() const { return controlBounded_; }

    // --- Deadbands ---
    // Control deadband: if |output| within [min,max] -> emit controlOffset
    // Error   deadband: if error within [min,max] -> treat as zero
    void  setControlDeadband(float deadbandMin, float deadbandMax);
    void  setErrorDeadband(float errDeadbandMin, float errDeadbandMax);

    // --- Setpoint ramp (units/second) — 0 disables ---
    void  setRamp(float ramp) { ramp_ = ramp; }
    float ramp() const { return ramp_; }

    // --- Misc ---
    void  setControlOffset(float offset) { controlOffset_ = offset; }
    float controlOffset() const { return controlOffset_; }
    void  setInputScale(float scale) { inputScale_ = scale; }
    float inputScale() const { return inputScale_; }
    void  setReverse(bool yes) { reverse_ = yes; }
    bool  reverse() const { return reverse_; }
    void  setInhibit(bool yes) { inhibit_ = yes; }
    bool  inhibit() const { return inhibit_; }

    // --- Introspection ---
    void  getControlState(float& err, float& errI, float& errD, float& control) const;

private:
    // Gains
    float kp_, ki_, kd_;

    // State
    float goal_, curSetpoint_;
    float lastErr_, errI_, lastErrD_, lastErrDFiltered_, lastControl_;
    unsigned long lastCycleUS_;

    // Bounds
    float iMin_, iMax_;                bool iBounded_;
    float controlMin_, controlMax_;    bool controlBounded_;
    float deadbandMin_, deadbandMax_;
    float errDeadbandMin_, errDeadbandMax_;

    // Config
    float ramp_;
    float controlOffset_;
    float inputScale_;
    bool  reverse_;
    bool  inhibit_;

    // D-term smoothing
    PidLowPassFilter differentialFilter_;
};

#endif // PID_CONTROLLER_H
