/**
 * IMU Handler for D-O Droid
 * Uses QMI8658C (onboard TENSTAR ESP32-S3)
 *
 * Note: MPU6050/BNO055 support disabled - QMI8658C is sufficient for self-balancing
 */

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#if IMU_USE_MADGWICK
  // Our own Madgwick implementation (see madgwick_ahrs.h) — used instead
  // of the Arduino-Madgwick library so we can change beta at runtime.
  #include "madgwick_ahrs.h"
#endif

// Forward declarations for IMU libraries
class SensorQMI8658;

// IMU operation modes. AIO32 has two physical IMUs:
//   QMI8658C on the TENSTAR module (I2C 0x6B)
//   LSM6DS3  on the AIO32 PCB      (I2C 0x6A, preferred for balance as it
//                                   sits fixed to the droid frame)
enum IMUMode {
    IMU_MODE_QMI_ONLY,      // Only QMI8658C
    IMU_MODE_LSM_ONLY,      // Only LSM6DS3
    IMU_MODE_FUSION,        // Both IMUs fused via Kalman filter (advanced)
    IMU_MODE_AUTO           // Auto-select: prefer LSM6DS3 if available, else QMI8658C
};

// IMU status flags
struct IMUStatus {
    bool qmi_available;
    bool lsm_available;
    bool qmi_initialized;
    bool lsm_initialized;
    IMUMode active_mode;
    uint32_t last_update;
    uint32_t update_count;
    float update_rate;
};

// Orientation data structure
struct Orientation {
    float roll;         // Roll angle in degrees
    float pitch;        // Pitch angle in degrees
    float yaw;          // Yaw angle in degrees
    float roll_rate;    // Roll rate in deg/s
    float pitch_rate;   // Pitch rate in deg/s
    float yaw_rate;     // Yaw rate in deg/s
    uint32_t timestamp; // Timestamp in milliseconds
};

// Raw sensor data
struct RawIMUData {
    float ax, ay, az;   // Accelerometer (m/s²)
    float gx, gy, gz;   // Gyroscope (deg/s)
    float temperature;  // Temperature (°C)
};

// Kalman filter structure
struct KalmanFilter {
    float Q_angle;
    float Q_gyro;
    float R_angle;
    float x_angle;
    float x_bias;
    float P[2][2];
    float K[2];
    float S;
    float y;
};

class IMUHandler {
private:
    // IMU objects
    SensorQMI8658* qmi8658;
    // LSM6DS3 uses direct I2C register access (no library), analogous to
    // the Mega sketch's clone-friendly MPU6050 handling.

    // Current IMU mode
    IMUMode currentMode;

    // Status tracking
    IMUStatus status;

    // Sensor data
    RawIMUData qmiData;
    RawIMUData lsmData;
    RawIMUData fusedData;

    // Orientation data
    Orientation orientation;
    Orientation qmiOrientation;
    Orientation lsmOrientation;

    // Calibration offsets
    struct {
        float ax_offset, ay_offset, az_offset;
        float gx_offset, gy_offset, gz_offset;
    } qmiCalibration, lsmCalibration;
    
    // Complementary filter variables
    float compFilterRoll, compFilterPitch;
    uint32_t lastFilterUpdate;
    
    // Kalman filters for sensor fusion (legacy path, used when
    // IMU_USE_MADGWICK is false)
    KalmanFilter kalmanRoll;
    KalmanFilter kalmanPitch;

#if IMU_USE_MADGWICK
    // Our own Madgwick AHRS implementation — same 6-DoF quaternion fusion
    // as the original, but with beta exposed via setBeta()/getBeta() so
    // it can be retuned at runtime via CLI (imu beta <val>). See
    // madgwick_ahrs.h for the algorithm provenance.
    // Fed with gyro (deg/s) + accel (g or m/s², normalised internally).
    MadgwickAHRS madgwick_;
#endif
    
    // Private methods
    bool initQMI8658C();
    bool initLSM6DS3();
    bool checkI2CBus();
    void readQMI8658C();
    void readLSM6DS3();
    void updateOrientation(RawIMUData& data, Orientation& orient);
    void fuseIMUData();
    void applyComplementaryFilter(RawIMUData& data, Orientation& orient);
    void applyKalmanFilter(RawIMUData& data, Orientation& orient);
    void calibrateIMU(RawIMUData& data, bool isQMI);
    float calculateUpdateRate();
    
    // Kalman filter functions
    void initKalmanFilter(KalmanFilter& kf);
    float updateKalmanFilter(KalmanFilter& kf, float newAngle, float newRate, float dt);

public:
    IMUHandler();
    ~IMUHandler();
    
    // Initialization
    bool begin(IMUMode mode = IMU_MODE_AUTO);
    void end();
    
    // Mode control
    bool setMode(IMUMode mode);
    IMUMode getMode() { return currentMode; }
    
    // Calibration
    void calibrate(uint16_t samples = 1000);
    void saveCalibration();
    void loadCalibration();
    void resetCalibration();
    // Print current calibration offsets to the given stream (usually Serial).
    // Used by the CLI 'imu bias show' command.
    void printBias(Stream& out);
    
    // Main update function
    bool update();
    
    // Data access
    Orientation getOrientation() { return orientation; }
    float getRoll() { return orientation.roll; }
    float getPitch() { return orientation.pitch; }
    float getYaw() { return orientation.yaw; }
    float getRollRate() { return orientation.roll_rate; }
    float getPitchRate() { return orientation.pitch_rate; }
    float getYawRate() { return orientation.yaw_rate; }
    
    // Raw data access
    RawIMUData getRawData() { return fusedData; }
    RawIMUData getQMIData() { return qmiData; }
    RawIMUData getLSMData() { return lsmData; }
    
    // Status and diagnostics
    IMUStatus getStatus() { return status; }
    bool isHealthy();
    void getStatusString(char* buffer, size_t bufferSize);
    void printDiagnostics();
    
    // Advanced features
    void setComplementaryFilterAlpha(float alpha);
    void setKalmanParameters(float Q_angle, float Q_gyro, float R_angle);

    // Madgwick beta accessor (runtime-tunable). Higher values react faster
    // to accelerometer changes at the cost of higher noise in the output.
#if IMU_USE_MADGWICK
    void  setMadgwickBeta(float beta) { madgwick_.setBeta(beta); }
    float getMadgwickBeta() const     { return madgwick_.getBeta(); }
#endif
    void enableMotionDetection(float threshold);
    void enableTapDetection();
    
    // Temperature
    float getTemperature();
};

// Global IMU handler instance
extern IMUHandler imuHandler;

#endif // IMU_HANDLER_H