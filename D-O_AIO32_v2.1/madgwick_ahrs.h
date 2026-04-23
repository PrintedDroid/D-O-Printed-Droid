/**
 * Madgwick AHRS — standalone 6-DoF IMU quaternion fusion filter.
 *
 * Drop-in replacement for PaulStoffregen's Arduino-Madgwick library, with
 * the critical difference that `beta` (gyro-bias correction gain) is fully
 * exposed via setBeta() / getBeta() and default-configurable via config.h.
 *
 * Based on the original algorithm published by Sebastian Madgwick:
 *   "An efficient orientation filter for inertial and inertial/magnetic
 *    sensor arrays", 2010. Public algorithm; this implementation is a
 *    clean-room rewrite so the file is free of any particular upstream
 *    library's copyright — licensed under Apache 2.0 alongside the rest
 *    of the AIO32 branch.
 *
 * Units (drop-in compatible with the rest of the IMU pipeline):
 *   Gyro in deg/s, converted to rad/s internally.
 *   Accel in any unit — normalised to a unit vector each update, so
 *   whether the caller supplies m/s² or g makes no difference to the
 *   filter output.
 *
 * Output:
 *   getRoll() / getPitch() / getYaw() in degrees (same convention as
 *   PaulStoffregen's library so existing call-sites keep working).
 */

#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <Arduino.h>

class MadgwickAHRS {
public:
    MadgwickAHRS();

    // Configure sample rate (Hz). Call once at setup; can be re-called if
    // the sample rate is ever changed at runtime.
    void begin(float sampleFrequencyHz);

    // Gyro-bias correction gain. Higher = faster convergence to the
    // accelerometer's absolute reading at the cost of more noise in the
    // orientation output. Practical range 0.03–0.5.
    //   0.03   ultra-smooth, very slow to catch up on big jumps
    //   0.1    PaulStoffregen default, good for quadcopters
    //   0.2    noticeable response while still PID-friendly
    //   0.5    nearly immediate, noisy
    void  setBeta(float beta)       { beta_ = beta; }
    float getBeta() const           { return beta_; }

    // 6-DoF update (no magnetometer). Gyro in deg/s, accel in any unit.
    // dt is computed internally from micros().
    void updateIMU(float gxDeg, float gyDeg, float gzDeg,
                   float ax, float ay, float az);

    // Euler outputs in degrees (Tait-Bryan, ZYX).
    float getRoll()  const;
    float getPitch() const;
    float getYaw()   const;

    // Raw quaternion — useful if the caller wants slerp or direct
    // quaternion math instead of Euler. Convention: q0 + q1*i + q2*j + q3*k.
    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const {
        q0 = q0_; q1 = q1_; q2 = q2_; q3 = q3_;
    }

    // Reset to identity quaternion (useful after mode switches or
    // calibration events).
    void reset();

private:
    float q0_, q1_, q2_, q3_;     // orientation quaternion
    float beta_;                  // gyro-bias correction gain
    float sampleFreqHz_;
    float invSampleFreq_;         // 1 / sampleFreq (seconds per sample)
    uint32_t lastUpdateUs_;
    bool     firstUpdate_;

    static inline float invSqrt(float x);
};

#endif // MADGWICK_AHRS_H
