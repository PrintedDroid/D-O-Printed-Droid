/**
 * Madgwick AHRS — implementation. See madgwick_ahrs.h for provenance.
 * Apache License 2.0.
 */

#include "madgwick_ahrs.h"
#include <math.h>

static const float DEG2RAD = 0.01745329252f;   // PI / 180
static const float RAD2DEG = 57.29577951f;     // 180 / PI

MadgwickAHRS::MadgwickAHRS()
    : q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),
      beta_(0.1f),
      sampleFreqHz_(100.0f),
      invSampleFreq_(0.01f),
      lastUpdateUs_(0),
      firstUpdate_(true)
{}

void MadgwickAHRS::begin(float sampleFrequencyHz) {
    sampleFreqHz_  = (sampleFrequencyHz > 0.0f) ? sampleFrequencyHz : 100.0f;
    invSampleFreq_ = 1.0f / sampleFreqHz_;
    reset();
}

void MadgwickAHRS::reset() {
    q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
    firstUpdate_  = true;
    lastUpdateUs_ = 0;
}

// Fast inverse square root — the classic quake-ish trick is unreliable on
// some compilers / float representations; a straightforward sqrtf is
// fast enough on ESP32-S3 and numerically clean, so we use that.
inline float MadgwickAHRS::invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

void MadgwickAHRS::updateIMU(float gxDeg, float gyDeg, float gzDeg,
                             float ax, float ay, float az) {
    // Work out real dt from micros() so the filter is immune to irregular
    // call frequencies — important when the caller polls at 50 Hz in
    // STATE_READY vs "as fast as possible" in STATE_RUNNING.
    uint32_t nowUs = micros();
    float dt;
    if (firstUpdate_) {
        dt = invSampleFreq_;
        firstUpdate_ = false;
    } else {
        uint32_t deltaUs = nowUs - lastUpdateUs_;
        dt = (float)deltaUs * 1.0e-6f;
        if (dt <= 0.0f)      dt = invSampleFreq_;
        if (dt > 0.5f)       dt = invSampleFreq_;  // clamp against gaps
    }
    lastUpdateUs_ = nowUs;

    // Convert gyro to rad/s
    float gx = gxDeg * DEG2RAD;
    float gy = gyDeg * DEG2RAD;
    float gz = gzDeg * DEG2RAD;

    // Rate of change of quaternion from gyroscope (pure integration).
    float qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
    float qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
    float qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
    float qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);

    // Compute feedback only if accelerometer sample is valid (non-zero)
    float accNormSq = ax * ax + ay * ay + az * az;
    if (accNormSq > 0.0f) {
        // Normalise accel vector
        float recipNorm = invSqrt(accNormSq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * q0_;
        float _2q1 = 2.0f * q1_;
        float _2q2 = 2.0f * q2_;
        float _2q3 = 2.0f * q3_;
        float _4q0 = 4.0f * q0_;
        float _4q1 = 4.0f * q1_;
        float _4q2 = 4.0f * q2_;
        float _8q1 = 8.0f * q1_;
        float _8q2 = 8.0f * q2_;
        float q0q0 = q0_ * q0_;
        float q1q1 = q1_ * q1_;
        float q2q2 = q2_ * q2_;
        float q3q3 = q3_ * q3_;

        // Gradient-descent step — objective function is the error between
        // estimated direction of gravity and measured direction of gravity.
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay
                   - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                   - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;

        // Normalise step magnitude
        float sNormSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (sNormSq > 0.0f) {
            recipNorm = invSqrt(sNormSq);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta_ * s0;
            qDot2 -= beta_ * s1;
            qDot3 -= beta_ * s2;
            qDot4 -= beta_ * s3;
        }
    }

    // Integrate rate of change of quaternion
    q0_ += qDot1 * dt;
    q1_ += qDot2 * dt;
    q2_ += qDot3 * dt;
    q3_ += qDot4 * dt;

    // Normalise quaternion
    float qNormSq = q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_;
    if (qNormSq > 0.0f) {
        float recipNorm = invSqrt(qNormSq);
        q0_ *= recipNorm;
        q1_ *= recipNorm;
        q2_ *= recipNorm;
        q3_ *= recipNorm;
    }
}

// Euler extraction from the quaternion. Tait-Bryan ZYX convention
// (yaw around Z, pitch around Y, roll around X), which matches what the
// Arduino-Madgwick library exposes via getPitch/Roll/Yaw so existing
// call-sites in imu_handler.cpp work unchanged.
float MadgwickAHRS::getRoll() const {
    return RAD2DEG * atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                            1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));
}

float MadgwickAHRS::getPitch() const {
    float v = 2.0f * (q0_ * q2_ - q3_ * q1_);
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    return RAD2DEG * asinf(v);
}

float MadgwickAHRS::getYaw() const {
    return RAD2DEG * atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                            1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_));
}
