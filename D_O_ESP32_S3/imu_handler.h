/**
 * IMU Handler for D-O Droid
 * Manages both QMI8658C and MPU6050 IMUs
 * Supports individual or fused operation
 */

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// Forward declarations for IMU libraries
class SensorQMI8658;
class Adafruit_MPU6050;

// IMU operation modes
enum IMUMode {
    IMU_MODE_QMI_ONLY,      // Only QMI8658C
    IMU_MODE_MPU_ONLY,      // Only MPU6050
    IMU_MODE_FUSION,        // Both IMUs with sensor fusion
    IMU_MODE_AUTO           // Auto-select based on availability
};

// IMU status flags
struct IMUStatus {
    bool qmi_available;
    bool mpu_available;
    bool qmi_initialized;
    bool mpu_initialized;
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
    Adafruit_MPU6050* mpu6050;
    
    // Current IMU mode
    IMUMode currentMode;
    
    // Status tracking
    IMUStatus status;
    
    // Sensor data
    RawIMUData qmiData;
    RawIMUData mpuData;
    RawIMUData fusedData;
    
    // Orientation data
    Orientation orientation;
    Orientation qmiOrientation;
    Orientation mpuOrientation;
    
    // Calibration offsets
    struct {
        float ax_offset, ay_offset, az_offset;
        float gx_offset, gy_offset, gz_offset;
    } qmiCalibration, mpuCalibration;
    
    // Complementary filter variables
    float compFilterRoll, compFilterPitch;
    uint32_t lastFilterUpdate;
    
    // Kalman filters for sensor fusion
    KalmanFilter kalmanRoll;
    KalmanFilter kalmanPitch;
    
    // Private methods
    bool initQMI8658C();
    bool initMPU6050();
    bool checkI2CBus();
    void readQMI8658C();
    void readMPU6050();
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
    RawIMUData getMPUData() { return mpuData; }
    
    // Status and diagnostics
    IMUStatus getStatus() { return status; }
    bool isHealthy();
    void getStatusString(char* buffer, size_t bufferSize);
    void printDiagnostics();
    
    // Advanced features
    void setComplementaryFilterAlpha(float alpha);
    void setKalmanParameters(float Q_angle, float Q_gyro, float R_angle);
    void enableMotionDetection(float threshold);
    void enableTapDetection();
    
    // Temperature
    float getTemperature();
};

// Global IMU handler instance
extern IMUHandler imuHandler;

#endif // IMU_HANDLER_H