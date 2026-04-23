/**
 * PID Controller implementation — see pid_controller.h for provenance.
 *
 * Based on BBDroids `bb::PIDController` (Apache 2.0, Björn Giesler 2023)
 * and Curio Res's 2nd-order Butterworth low-pass tutorial.
 *
 * Licensed under Apache License 2.0.
 */

#include "pid_controller.h"
#include <math.h>
#include <limits.h>

#ifndef EPSILON
  #define EPSILON(x) (fabsf(x) < 1e-6f)
#endif

// ==========================================================================
// PidLowPassFilter
// ==========================================================================

PidLowPassFilter::PidLowPassFilter(float cutoffHz, float sampleFreqHz, bool adaptive) :
    adaptive_(adaptive),
    tn1_(0.0f),
    cutoffHz_(cutoffHz),
    sampleFreqHz_(sampleFreqHz),
    needsRecalc_(true)
{
    dt_ = 1.0f / sampleFreqHz_;
    tn1_ = -dt_;
    for (int k = 0; k < 3; ++k) { x_[k] = 0.0f; y_[k] = 0.0f; }
    computeCoefficients();
}

void PidLowPassFilter::setCutoff(float cutoffHz) {
    cutoffHz_ = cutoffHz;
    needsRecalc_ = true;
}

void PidLowPassFilter::setSampleFrequency(float sampleFreqHz) {
    sampleFreqHz_ = sampleFreqHz;
    dt_  = 1.0f / sampleFreqHz_;
    tn1_ = -dt_;
    needsRecalc_ = true;
}

void PidLowPassFilter::setAdaptive(bool adaptive) {
    adaptive_ = adaptive;
    needsRecalc_ = true;
}

void PidLowPassFilter::reset() {
    for (int k = 0; k < 3; ++k) { x_[k] = 0.0f; y_[k] = 0.0f; }
}

void PidLowPassFilter::computeCoefficients() {
    omega0_ = 2.0f * (float)M_PI * cutoffHz_;

    if (adaptive_) {
        float t = micros() / 1.0e6f;
        dt_ = t - tn1_;
        tn1_ = t;
    }

    float alpha   = omega0_ * dt_;
    float alphaSq = alpha * alpha;
    float beta[]  = { 1.0f, sqrtf(2.0f), 1.0f };
    float D = alphaSq * beta[0] + 2.0f * alpha * beta[1] + 4.0f * beta[2];
    b_[0] = alphaSq / D;
    b_[1] = 2.0f * b_[0];
    b_[2] = b_[0];
    a_[0] = -(2.0f * alphaSq * beta[0] - 8.0f * beta[2]) / D;
    a_[1] = -(beta[0] * alphaSq - 2.0f * beta[1] * alpha + 4.0f * beta[2]) / D;
}

float PidLowPassFilter::filter(float xn) {
    if (adaptive_ || needsRecalc_) {
        computeCoefficients();
        needsRecalc_ = false;
    }
    y_[0] = 0.0f;
    x_[0] = xn;
    for (int k = 0; k < 2; ++k) {
        y_[0] += a_[k] * y_[k + 1] + b_[k] * x_[k];
    }
    y_[0] += b_[2] * x_[2];

    for (int k = 2; k > 0; --k) {
        y_[k] = y_[k - 1];
        x_[k] = x_[k - 1];
    }
    return y_[0];
}

// ==========================================================================
// PIDController
// ==========================================================================

PIDController::PIDController() :
    kp_(0.0f), ki_(0.0f), kd_(0.0f),
    goal_(0.0f), curSetpoint_(0.0f),
    lastErr_(0.0f), errI_(0.0f), lastErrD_(0.0f),
    lastErrDFiltered_(0.0f), lastControl_(0.0f),
    lastCycleUS_(0),
    iMin_(0.0f), iMax_(0.0f), iBounded_(false),
    controlMin_(0.0f), controlMax_(0.0f), controlBounded_(false),
    deadbandMin_(0.0f), deadbandMax_(0.0f),
    errDeadbandMin_(0.0f), errDeadbandMax_(0.0f),
    ramp_(0.0f),
    controlOffset_(0.0f),
    inputScale_(1.0f),
    reverse_(false),
    inhibit_(false),
    differentialFilter_(5.0f, 100.0f, false)   // 5 Hz cutoff, 100 Hz sample
{
    reset();
}

void PIDController::reset() {
    lastErr_ = 0.0f;
    errI_    = 0.0f;
    lastErrD_ = 0.0f;
    lastErrDFiltered_ = 0.0f;
    lastControl_ = 0.0f;
    curSetpoint_ = goal_;
    lastCycleUS_ = micros();
    differentialFilter_.reset();
}

float PIDController::update(float currentValue) {
    unsigned long us = micros();
    unsigned long timediffUS = (us < lastCycleUS_)
                                 ? ((ULONG_MAX - lastCycleUS_) + us)
                                 : (us - lastCycleUS_);
    lastCycleUS_ = us;
    float dt = timediffUS / 1.0e6f;
    if (dt <= 0.0f) dt = 1e-6f;  // guard div-by-zero on first tick

    // Setpoint ramp — if ramp_ > 0, step curSetpoint_ toward goal_
    if (!EPSILON(ramp_) && !EPSILON(curSetpoint_ - goal_)) {
        float delta = fabsf(ramp_ * dt);
        if (curSetpoint_ < goal_) {
            curSetpoint_ += delta;
            if (curSetpoint_ > goal_) curSetpoint_ = goal_;
        } else {
            curSetpoint_ -= delta;
            if (curSetpoint_ < goal_) curSetpoint_ = goal_;
        }
    }

    // Error
    float err = reverse_
                  ? (curSetpoint_ + currentValue * inputScale_)
                  : (curSetpoint_ - currentValue * inputScale_);
    if (err > errDeadbandMin_ && err < errDeadbandMax_) err = 0.0f;

    // Integral
    errI_ += err * dt;
    if (iBounded_) {
        errI_ = constrain(errI_, iMin_, iMax_);
    }

    // Derivative (filtered)
    lastErrD_         = (err - lastErr_) / dt;
    lastErr_          = err;
    lastErrDFiltered_ = differentialFilter_.filter(lastErrD_);

    // Combine
    lastControl_ = kp_ * lastErr_ + ki_ * errI_ + kd_ * lastErrDFiltered_;
    if (controlBounded_) {
        lastControl_ = constrain(lastControl_, controlMin_, controlMax_);
    }

    // Control deadband + offset + reverse
    float controlOut;
    if (lastControl_ > deadbandMin_ && lastControl_ < deadbandMax_) {
        controlOut = controlOffset_;
    } else if (reverse_) {
        controlOut = -(lastControl_ + controlOffset_);
    } else {
        controlOut = lastControl_ + controlOffset_;
    }

    if (inhibit_) return 0.0f;
    return controlOut;
}

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::getGains(float& kp, float& ki, float& kd) const {
    kp = kp_; ki = ki_; kd = kd_;
}

void PIDController::setGoal(float goal) {
    goal_ = goal;
    if (EPSILON(ramp_)) curSetpoint_ = goal_;
    // else: let update() ramp curSetpoint_ toward goal_ over time
}

void PIDController::setPresentAsGoal(float currentValue) {
    goal_ = currentValue;
    curSetpoint_ = currentValue;
}

void PIDController::setIBounds(float iMin, float iMax) {
    iBounded_ = true;
    iMin_ = iMin;
    iMax_ = iMax;
}

void PIDController::setIUnbounded() {
    iMin_ = iMax_ = 0.0f;
    iBounded_ = false;
}

void PIDController::setControlBounds(float controlMin, float controlMax) {
    controlBounded_ = true;
    controlMin_ = controlMin;
    controlMax_ = controlMax;
}

void PIDController::setControlUnbounded() {
    controlMin_ = controlMax_ = 0.0f;
    controlBounded_ = false;
}

void PIDController::setControlDeadband(float deadbandMin, float deadbandMax) {
    deadbandMin_ = deadbandMin;
    deadbandMax_ = deadbandMax;
}

void PIDController::setErrorDeadband(float errDeadbandMin, float errDeadbandMax) {
    errDeadbandMin_ = errDeadbandMin;
    errDeadbandMax_ = errDeadbandMax;
}

void PIDController::getControlState(float& err, float& errI, float& errD, float& control) const {
    err     = lastErr_;
    errI    = errI_;
    errD    = lastErrDFiltered_;
    control = lastControl_;
}
