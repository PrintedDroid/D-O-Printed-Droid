/**
 * IMU Handler Implementation - CORRECTED VERSION
 * Manages QMI8658C and MPU6050 for accurate orientation tracking
 * 
 * FIXED: Added timeout protection to prevent watchdog resets
 * FIXED: Removed String objects
 * ADDED: Better error handling and I2C bus recovery
 */

#include "imu_handler.h"
#include <SensorQMI8658.hpp>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include "esp_task_wdt.h"

// Global instance
IMUHandler imuHandler;

// Preferences for calibration storage
Preferences preferences;

// Check I2C bus health
bool IMUHandler::checkI2CBus() {
    // Try to detect any device on the bus
    uint8_t deviceCount = 0;
    
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            deviceCount++;
            DEBUG_PRINTF("IMU Handler: I2C device found at 0x%02X\n", address);
        } else if (error == 5) {
            // Timeout - bus might be stuck
            DEBUG_PRINTLN("IMU Handler: I2C bus timeout - attempting recovery");
            
            // Try to recover I2C bus
            Wire.end();
            delay(100);
            Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
            Wire.setClock(100000);
            delay(100);
            
            // Check again
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0) {
                DEBUG_PRINTLN("IMU Handler: I2C bus recovered!");
                return true;
            }
        }
        
        // Feed watchdog during scan
        if (address % 16 == 0) {
            esp_task_wdt_reset();
        }
    }
    
    if (deviceCount == 0) {
        DEBUG_PRINTLN("IMU Handler: No I2C devices found!");
        DEBUG_PRINTLN("Check: GPIO21 (TFT_I2C_POWER) is HIGH?");
        DEBUG_PRINTLN("Check: SDA/SCL connections?");
        return false;
    }
    
    return true;
}

// Constructor
IMUHandler::IMUHandler() : 
    qmi8658(nullptr), 
    mpu6050(nullptr),
    currentMode(IMU_MODE_AUTO),
    compFilterRoll(0.0f),
    compFilterPitch(0.0f),
    lastFilterUpdate(0) {
    
    // Initialize status
    memset(&status, 0, sizeof(IMUStatus));
    memset(&qmiCalibration, 0, sizeof(qmiCalibration));
    memset(&mpuCalibration, 0, sizeof(mpuCalibration));
}

// Destructor
IMUHandler::~IMUHandler() {
    end();
}

// Initialize IMU system - WITH TIMEOUT PROTECTION
bool IMUHandler::begin(IMUMode mode) {
    DEBUG_PRINTLN("IMU Handler: Starting initialization...");
    
    // Feed watchdog at start
    esp_task_wdt_reset();
    
    // Initialize I2C if not already done
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000); // 100kHz for stability
    delay(10);
    
    // Reset status
    memset(&status, 0, sizeof(IMUStatus));
    status.active_mode = mode;
    
    // Try to initialize QMI8658C with timeout
    if (USE_QMI8658C) {
        DEBUG_PRINTLN("IMU Handler: Attempting to init QMI8658C...");
        esp_task_wdt_reset(); // Feed watchdog
        
        uint32_t startTime = millis();
        status.qmi_available = initQMI8658C();
        
        if (millis() - startTime > 1000) {
            DEBUG_PRINTLN("IMU Handler: QMI8658C init timeout!");
            status.qmi_available = false;
        }
        
        if (status.qmi_available) {
            DEBUG_PRINTLN("IMU Handler: QMI8658C initialized successfully");
        } else {
            DEBUG_PRINTLN("IMU Handler: QMI8658C not found or init failed");
        }
    }
    
    // Try to initialize MPU6050 with timeout
    if (USE_MPU6050) {
        DEBUG_PRINTLN("IMU Handler: Attempting to init MPU6050...");
        esp_task_wdt_reset(); // Feed watchdog
        
        uint32_t startTime = millis();
        status.mpu_available = initMPU6050();
        
        if (millis() - startTime > 1000) {
            DEBUG_PRINTLN("IMU Handler: MPU6050 init timeout!");
            status.mpu_available = false;
        }
        
        if (status.mpu_available) {
            DEBUG_PRINTLN("IMU Handler: MPU6050 initialized successfully");
        } else {
            DEBUG_PRINTLN("IMU Handler: MPU6050 not found or init failed");
        }
    }
    
    // Load calibration data
    loadCalibration();
    
    // Initialize Kalman filters
    initKalmanFilter(kalmanRoll);
    initKalmanFilter(kalmanPitch);
    
    // Determine actual mode based on availability
    if (mode == IMU_MODE_AUTO) {
        if (status.qmi_available && status.mpu_available) {
            currentMode = IMU_MODE_FUSION;
            DEBUG_PRINTLN("IMU Handler: Using FUSION mode (both IMUs)");
        } else if (status.qmi_available) {
            currentMode = IMU_MODE_QMI_ONLY;
            DEBUG_PRINTLN("IMU Handler: Using QMI8658C only");
        } else if (status.mpu_available) {
            currentMode = IMU_MODE_MPU_ONLY;
            DEBUG_PRINTLN("IMU Handler: Using MPU6050 only");
        } else {
            DEBUG_PRINTLN("IMU Handler: ERROR - No IMUs available!");
            return false;
        }
    } else {
        currentMode = mode;
    }
    
    status.active_mode = currentMode;
    lastFilterUpdate = millis();
    
    DEBUG_PRINTF("IMU Handler: Initialization complete. Active mode: %d\n", currentMode);
    return (status.qmi_available || status.mpu_available);
}

// Initialize QMI8658C - WITH TIMEOUT AND ERROR HANDLING
bool IMUHandler::initQMI8658C() {
    // First check I2C bus health
    if (!checkI2CBus()) {
        DEBUG_PRINTLN("IMU Handler: I2C bus error - check connections");
        return false;
    }
    
    // Check if device exists at expected address with retries
    uint8_t retries = 3;
    while (retries > 0) {
        Wire.beginTransmission(QMI8658C_ADDRESS);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            break;  // Success
        } else if (error == 1) {
            DEBUG_PRINTLN("IMU Handler: I2C data too long");
        } else if (error == 2) {
            DEBUG_PRINTLN("IMU Handler: I2C NACK on address");
        } else if (error == 3) {
            DEBUG_PRINTLN("IMU Handler: I2C NACK on data");
        } else if (error == 4) {
            DEBUG_PRINTLN("IMU Handler: I2C other error");
        } else if (error == 5) {
            DEBUG_PRINTLN("IMU Handler: I2C timeout");
        }
        
        retries--;
        if (retries > 0) {
            DEBUG_PRINTF("IMU Handler: Retrying... (%d left)\n", retries);
            delay(100);
            esp_task_wdt_reset();
        }
    }
    
    if (retries == 0) {
        DEBUG_PRINTLN("IMU Handler: QMI8658C not responding at address 0x6B");
        return false;
    }
    
    try {
        qmi8658 = new SensorQMI8658();
        
        if (!qmi8658) {
            DEBUG_PRINTLN("IMU Handler: Failed to allocate QMI8658C object");
            return false;
        }
        
        // Set a short timeout for initialization
        uint32_t initStart = millis();
        
        // Initialize with I2C using the correct slave address
        bool initSuccess = false;
        
        // Try initialization with timeout check
        while (millis() - initStart < 500) { // 500ms timeout
            if (qmi8658->begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN)) {
                initSuccess = true;
                break;
            }
            delay(10);
            esp_task_wdt_reset();
        }
        
        if (!initSuccess) {
            DEBUG_PRINTLN("IMU Handler: QMI8658C begin() failed");
            delete qmi8658;
            qmi8658 = nullptr;
            return false;
        }
        
        DEBUG_PRINTF("IMU Handler: QMI8658C found, ID: 0x%X\n", qmi8658->getChipID());
        
        // Configure accelerometer
        qmi8658->configAccelerometer(
            SensorQMI8658::ACC_RANGE_4G,
            SensorQMI8658::ACC_ODR_125Hz,
            SensorQMI8658::LPF_MODE_0
        );
        
        // Configure gyroscope
        qmi8658->configGyroscope(
            SensorQMI8658::GYR_RANGE_512DPS,
            SensorQMI8658::GYR_ODR_112_1Hz,
            SensorQMI8658::LPF_MODE_3
        );
        
        // Enable both sensors
        qmi8658->enableAccelerometer();
        qmi8658->enableGyroscope();
        
        status.qmi_initialized = true;
        return true;
        
    } catch (...) {
        DEBUG_PRINTLN("IMU Handler: Exception during QMI8658C init");
    }
    
    if (qmi8658) {
        delete qmi8658;
        qmi8658 = nullptr;
    }
    return false;
}

// Initialize MPU6050 - WITH TIMEOUT AND ERROR HANDLING
bool IMUHandler::initMPU6050() {
    // First check I2C bus health
    if (!checkI2CBus()) {
        DEBUG_PRINTLN("IMU Handler: I2C bus error - check connections");
        return false;
    }
    
    // Check if device exists at expected address with retries
    uint8_t retries = 3;
    while (retries > 0) {
        Wire.beginTransmission(MPU6050_ADDRESS);
        uint8_t error = Wire.endTransmission();
        
        if (error == 0) {
            break;  // Success
        } else {
            DEBUG_PRINTF("IMU Handler: MPU6050 I2C error %d\n", error);
        }
        
        retries--;
        if (retries > 0) {
            DEBUG_PRINTF("IMU Handler: Retrying... (%d left)\n", retries);
            delay(100);
            esp_task_wdt_reset();
        }
    }
    
    if (retries == 0) {
        DEBUG_PRINTLN("IMU Handler: MPU6050 not responding at address 0x68");
        return false;
    }
    
    try {
        mpu6050 = new Adafruit_MPU6050();
        
        if (!mpu6050) {
            DEBUG_PRINTLN("IMU Handler: Failed to allocate MPU6050 object");
            return false;
        }
        
        // Set a short timeout for initialization
        uint32_t initStart = millis();
        bool initSuccess = false;
        
        // Try initialization with timeout check
        while (millis() - initStart < 500) { // 500ms timeout
            if (mpu6050->begin(MPU6050_ADDRESS, &Wire)) {
                initSuccess = true;
                break;
            }
            delay(10);
            esp_task_wdt_reset();
        }
        
        if (!initSuccess) {
            DEBUG_PRINTLN("IMU Handler: MPU6050 begin() failed");
            delete mpu6050;
            mpu6050 = nullptr;
            return false;
        }
        
        DEBUG_PRINTLN("IMU Handler: MPU6050 found and initialized");
        
        // Configure sensor
        mpu6050->setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu6050->setGyroRange(MPU6050_RANGE_500_DEG);
        mpu6050->setFilterBandwidth(MPU6050_BAND_21_HZ);
        
        // Enable interrupt on data ready if pin is defined
        if (MPU6050_INT_PIN >= 0) {
            pinMode(MPU6050_INT_PIN, INPUT);
        }
        
        status.mpu_initialized = true;
        return true;
        
    } catch (...) {
        DEBUG_PRINTLN("IMU Handler: Exception during MPU6050 init");
    }
    
    if (mpu6050) {
        delete mpu6050;
        mpu6050 = nullptr;
    }
    return false;
}

// Main update function
bool IMUHandler::update() {
    if (!status.qmi_initialized && !status.mpu_initialized) {
        return false;
    }
    
    uint32_t now = millis();
    
    // Read sensor data based on mode
    switch (currentMode) {
        case IMU_MODE_QMI_ONLY:
            if (status.qmi_initialized) {
                readQMI8658C();
                updateOrientation(qmiData, orientation);
            }
            break;
            
        case IMU_MODE_MPU_ONLY:
            if (status.mpu_initialized) {
                readMPU6050();
                updateOrientation(mpuData, orientation);
            }
            break;
            
        case IMU_MODE_FUSION:
            if (status.qmi_initialized) readQMI8658C();
            if (status.mpu_initialized) readMPU6050();
            fuseIMUData();
            updateOrientation(fusedData, orientation);
            break;
    }
    
    // Update status
    status.last_update = now;
    status.update_count++;
    status.update_rate = calculateUpdateRate();
    
    return true;
}

// Read QMI8658C data  
void IMUHandler::readQMI8658C() {
    if (!qmi8658) {
        DEBUG_PRINTLN("IMU Handler: QMI8658C null pointer!");
        return;
    }
    
    // Get accelerometer data (already in m/s²)
    if (qmi8658->getAccelerometer(qmiData.ax, qmiData.ay, qmiData.az)) {
        // Data is already in m/s²
    } else {
        DEBUG_PRINTLN("IMU Handler: Failed to read QMI8658C accelerometer");
    }
    
    // Get gyroscope data (already in degrees/sec)
    if (qmi8658->getGyroscope(qmiData.gx, qmiData.gy, qmiData.gz)) {
        // Data is already in deg/s
    } else {
        DEBUG_PRINTLN("IMU Handler: Failed to read QMI8658C gyroscope");
    }
    
    // Get temperature
    qmiData.temperature = qmi8658->getTemperature_C();
    
    // Apply calibration
    calibrateIMU(qmiData, true);
}

// Read MPU6050 data
void IMUHandler::readMPU6050() {
    if (!mpu6050) {
        DEBUG_PRINTLN("IMU Handler: MPU6050 null pointer!");
        return;
    }
    
    sensors_event_t a, g, temp;
    
    // Check if getEvent succeeds
    if (mpu6050->getEvent(&a, &g, &temp)) {
        mpuData.ax = a.acceleration.x;
        mpuData.ay = a.acceleration.y;
        mpuData.az = a.acceleration.z;
        mpuData.gx = g.gyro.x * RAD_TO_DEG;
        mpuData.gy = g.gyro.y * RAD_TO_DEG;
        mpuData.gz = g.gyro.z * RAD_TO_DEG;
        mpuData.temperature = temp.temperature;
        
        // Apply calibration
        calibrateIMU(mpuData, false);
    } else {
        DEBUG_PRINTLN("IMU Handler: Failed to read MPU6050 data");
    }
}

// Fuse data from both IMUs
void IMUHandler::fuseIMUData() {
    if (status.qmi_initialized && status.mpu_initialized) {
        // Simple averaging for now, can be improved with weighted fusion
        fusedData.ax = (qmiData.ax + mpuData.ax) / 2.0f;
        fusedData.ay = (qmiData.ay + mpuData.ay) / 2.0f;
        fusedData.az = (qmiData.az + mpuData.az) / 2.0f;
        fusedData.gx = (qmiData.gx + mpuData.gx) / 2.0f;
        fusedData.gy = (qmiData.gy + mpuData.gy) / 2.0f;
        fusedData.gz = (qmiData.gz + mpuData.gz) / 2.0f;
        fusedData.temperature = (qmiData.temperature + mpuData.temperature) / 2.0f;
    } else if (status.qmi_initialized) {
        fusedData = qmiData;
    } else if (status.mpu_initialized) {
        fusedData = mpuData;
    }
}

// Update orientation from raw data
void IMUHandler::updateOrientation(RawIMUData& data, Orientation& orient) {
    uint32_t now = millis();
    float dt = (now - lastFilterUpdate) / 1000.0f;
    lastFilterUpdate = now;
    
    if (dt > 0.5f) dt = 0.01f; // Limit max dt to handle startup
    
    // Calculate angles from accelerometer
    float accelRoll = atan2(data.ay, data.az) * RAD_TO_DEG;
    float accelPitch = atan2(-data.ax, sqrt(data.ay * data.ay + data.az * data.az)) * RAD_TO_DEG;
    
    // Apply complementary filter
    compFilterRoll = COMP_FILTER_ALPHA * (compFilterRoll + data.gx * dt) + 
                     (1.0f - COMP_FILTER_ALPHA) * accelRoll;
    compFilterPitch = COMP_FILTER_ALPHA * (compFilterPitch + data.gy * dt) + 
                      (1.0f - COMP_FILTER_ALPHA) * accelPitch;
    
    // Update Kalman filter for even better accuracy
    orient.roll = updateKalmanFilter(kalmanRoll, accelRoll, data.gx, dt);
    orient.pitch = updateKalmanFilter(kalmanPitch, accelPitch, data.gy, dt);
    orient.yaw += data.gz * dt; // Simple integration for yaw
    
    // Store rates
    orient.roll_rate = data.gx;
    orient.pitch_rate = data.gy;
    orient.yaw_rate = data.gz;
    orient.timestamp = now;
}

// Initialize Kalman filter
void IMUHandler::initKalmanFilter(KalmanFilter& kf) {
    kf.Q_angle = KALMAN_Q_ANGLE;
    kf.Q_gyro = KALMAN_Q_GYRO;
    kf.R_angle = KALMAN_R_ANGLE;
    kf.x_angle = 0.0f;
    kf.x_bias = 0.0f;
    kf.P[0][0] = 0.0f;
    kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f;
    kf.P[1][1] = 0.0f;
}

// Update Kalman filter
float IMUHandler::updateKalmanFilter(KalmanFilter& kf, float newAngle, float newRate, float dt) {
    // Predict
    kf.x_angle += dt * (newRate - kf.x_bias);
    kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + kf.Q_angle);
    kf.P[0][1] -= dt * kf.P[1][1];
    kf.P[1][0] -= dt * kf.P[1][1];
    kf.P[1][1] += kf.Q_gyro * dt;
    
    // Update
    kf.y = newAngle - kf.x_angle;
    kf.S = kf.P[0][0] + kf.R_angle;
    kf.K[0] = kf.P[0][0] / kf.S;
    kf.K[1] = kf.P[1][0] / kf.S;
    
    kf.x_angle += kf.K[0] * kf.y;
    kf.x_bias += kf.K[1] * kf.y;
    
    float P00_temp = kf.P[0][0];
    float P01_temp = kf.P[0][1];
    
    kf.P[0][0] -= kf.K[0] * P00_temp;
    kf.P[0][1] -= kf.K[0] * P01_temp;
    kf.P[1][0] -= kf.K[1] * P00_temp;
    kf.P[1][1] -= kf.K[1] * P01_temp;
    
    return kf.x_angle;
}

// Calibrate IMU
void IMUHandler::calibrate(uint16_t samples) {
    DEBUG_PRINTLN("IMU Handler: Starting calibration...");
    
    // Reset calibration values
    resetCalibration();
    
    float qmi_ax_sum = 0, qmi_ay_sum = 0, qmi_az_sum = 0;
    float qmi_gx_sum = 0, qmi_gy_sum = 0, qmi_gz_sum = 0;
    float mpu_ax_sum = 0, mpu_ay_sum = 0, mpu_az_sum = 0;
    float mpu_gx_sum = 0, mpu_gy_sum = 0, mpu_gz_sum = 0;
    
    // Collect samples
    for (uint16_t i = 0; i < samples; i++) {
        if (status.qmi_initialized) {
            readQMI8658C();
            qmi_ax_sum += qmiData.ax;
            qmi_ay_sum += qmiData.ay;
            qmi_az_sum += qmiData.az;
            qmi_gx_sum += qmiData.gx;
            qmi_gy_sum += qmiData.gy;
            qmi_gz_sum += qmiData.gz;
        }
        
        if (status.mpu_initialized) {
            readMPU6050();
            mpu_ax_sum += mpuData.ax;
            mpu_ay_sum += mpuData.ay;
            mpu_az_sum += mpuData.az;
            mpu_gx_sum += mpuData.gx;
            mpu_gy_sum += mpuData.gy;
            mpu_gz_sum += mpuData.gz;
        }
        
        // Feed watchdog during calibration
        if (i % 100 == 0) {
            esp_task_wdt_reset();
        }
        
        delay(5);
    }
    
    // Calculate offsets
    if (status.qmi_initialized) {
        qmiCalibration.ax_offset = qmi_ax_sum / samples;
        qmiCalibration.ay_offset = qmi_ay_sum / samples;
        qmiCalibration.az_offset = qmi_az_sum / samples - 9.81f; // Gravity
        qmiCalibration.gx_offset = qmi_gx_sum / samples;
        qmiCalibration.gy_offset = qmi_gy_sum / samples;
        qmiCalibration.gz_offset = qmi_gz_sum / samples;
    }
    
    if (status.mpu_initialized) {
        mpuCalibration.ax_offset = mpu_ax_sum / samples;
        mpuCalibration.ay_offset = mpu_ay_sum / samples;
        mpuCalibration.az_offset = mpu_az_sum / samples - 9.81f; // Gravity
        mpuCalibration.gx_offset = mpu_gx_sum / samples;
        mpuCalibration.gy_offset = mpu_gy_sum / samples;
        mpuCalibration.gz_offset = mpu_gz_sum / samples;
    }
    
    // Save calibration
    saveCalibration();
    
    DEBUG_PRINTLN("IMU Handler: Calibration complete");
}

// Apply calibration to IMU data
void IMUHandler::calibrateIMU(RawIMUData& data, bool isQMI) {
    if (isQMI) {
        data.ax -= qmiCalibration.ax_offset;
        data.ay -= qmiCalibration.ay_offset;
        data.az -= qmiCalibration.az_offset;
        data.gx -= qmiCalibration.gx_offset;
        data.gy -= qmiCalibration.gy_offset;
        data.gz -= qmiCalibration.gz_offset;
    } else {
        data.ax -= mpuCalibration.ax_offset;
        data.ay -= mpuCalibration.ay_offset;
        data.az -= mpuCalibration.az_offset;
        data.gx -= mpuCalibration.gx_offset;
        data.gy -= mpuCalibration.gy_offset;
        data.gz -= mpuCalibration.gz_offset;
    }
}

// Save calibration to preferences
void IMUHandler::saveCalibration() {
    preferences.begin("imu_calib", false);
    
    // Save QMI calibration
    preferences.putFloat("qmi_ax", qmiCalibration.ax_offset);
    preferences.putFloat("qmi_ay", qmiCalibration.ay_offset);
    preferences.putFloat("qmi_az", qmiCalibration.az_offset);
    preferences.putFloat("qmi_gx", qmiCalibration.gx_offset);
    preferences.putFloat("qmi_gy", qmiCalibration.gy_offset);
    preferences.putFloat("qmi_gz", qmiCalibration.gz_offset);
    
    // Save MPU calibration
    preferences.putFloat("mpu_ax", mpuCalibration.ax_offset);
    preferences.putFloat("mpu_ay", mpuCalibration.ay_offset);
    preferences.putFloat("mpu_az", mpuCalibration.az_offset);
    preferences.putFloat("mpu_gx", mpuCalibration.gx_offset);
    preferences.putFloat("mpu_gy", mpuCalibration.gy_offset);
    preferences.putFloat("mpu_gz", mpuCalibration.gz_offset);
    
    preferences.end();
}

// Load calibration from preferences
void IMUHandler::loadCalibration() {
    preferences.begin("imu_calib", true);
    
    // Load QMI calibration
    qmiCalibration.ax_offset = preferences.getFloat("qmi_ax", 0.0f);
    qmiCalibration.ay_offset = preferences.getFloat("qmi_ay", 0.0f);
    qmiCalibration.az_offset = preferences.getFloat("qmi_az", 0.0f);
    qmiCalibration.gx_offset = preferences.getFloat("qmi_gx", 0.0f);
    qmiCalibration.gy_offset = preferences.getFloat("qmi_gy", 0.0f);
    qmiCalibration.gz_offset = preferences.getFloat("qmi_gz", 0.0f);
    
    // Load MPU calibration
    mpuCalibration.ax_offset = preferences.getFloat("mpu_ax", 0.0f);
    mpuCalibration.ay_offset = preferences.getFloat("mpu_ay", 0.0f);
    mpuCalibration.az_offset = preferences.getFloat("mpu_az", 0.0f);
    mpuCalibration.gx_offset = preferences.getFloat("mpu_gx", 0.0f);
    mpuCalibration.gy_offset = preferences.getFloat("mpu_gy", 0.0f);
    mpuCalibration.gz_offset = preferences.getFloat("mpu_gz", 0.0f);
    
    preferences.end();
}

// Reset calibration
void IMUHandler::resetCalibration() {
    memset(&qmiCalibration, 0, sizeof(qmiCalibration));
    memset(&mpuCalibration, 0, sizeof(mpuCalibration));
}

// Calculate update rate
float IMUHandler::calculateUpdateRate() {
    static uint32_t lastCalc = 0;
    static uint32_t lastCount = 0;
    
    uint32_t now = millis();
    if (now - lastCalc >= 1000) {
        float rate = (float)(status.update_count - lastCount) * 1000.0f / (float)(now - lastCalc);
        lastCalc = now;
        lastCount = status.update_count;
        status.update_rate = rate;
        return rate;
    }
    
    return status.update_rate;
}

// Check if IMU system is healthy
bool IMUHandler::isHealthy() {
    // Check if at least one IMU is working
    if (!status.qmi_initialized && !status.mpu_initialized) {
        return false;
    }
    
    // Check update rate
    if (status.update_rate < 50.0f) { // Less than 50Hz is problematic
        return false;
    }
    
    // Check for reasonable orientation values
    if (abs(orientation.roll) > 180 || abs(orientation.pitch) > 180) {
        return false;
    }
    
    return true;
}

// Get status string for debugging - CORRECTED WITHOUT String
void IMUHandler::getStatusString(char* buffer, size_t bufferSize) {
    char modeStr[20];
    
    switch (currentMode) {
        case IMU_MODE_QMI_ONLY: strcpy(modeStr, "QMI Only"); break;
        case IMU_MODE_MPU_ONLY: strcpy(modeStr, "MPU Only"); break;
        case IMU_MODE_FUSION: strcpy(modeStr, "Fusion"); break;
        case IMU_MODE_AUTO: strcpy(modeStr, "Auto"); break;
        default: strcpy(modeStr, "Unknown"); break;
    }
    
    snprintf(buffer, bufferSize,
             "IMU Status:\n"
             "Mode: %s\n"
             "QMI8658C: %s\n"
             "MPU6050: %s\n"
             "Update Rate: %.1f Hz\n"
             "Roll: %.1f°\n"
             "Pitch: %.1f°\n"
             "Yaw: %.1f°",
             modeStr,
             status.qmi_initialized ? "OK" : "FAIL",
             status.mpu_initialized ? "OK" : "FAIL",
             status.update_rate,
             orientation.roll,
             orientation.pitch,
             orientation.yaw);
}

// Print diagnostics
void IMUHandler::printDiagnostics() {
    DEBUG_PRINTLN("=== IMU Diagnostics ===");
    char statusBuffer[256];
    getStatusString(statusBuffer, sizeof(statusBuffer));
    DEBUG_PRINTLN(statusBuffer);
    
    if (status.qmi_initialized) {
        DEBUG_PRINTLN("QMI8658C Data:");
        DEBUG_PRINTF("  Accel: %.2f, %.2f, %.2f\n", qmiData.ax, qmiData.ay, qmiData.az);
        DEBUG_PRINTF("  Gyro: %.2f, %.2f, %.2f\n", qmiData.gx, qmiData.gy, qmiData.gz);
        DEBUG_PRINTF("  Temp: %.1f°C\n", qmiData.temperature);
    }
    
    if (status.mpu_initialized) {
        DEBUG_PRINTLN("MPU6050 Data:");
        DEBUG_PRINTF("  Accel: %.2f, %.2f, %.2f\n", mpuData.ax, mpuData.ay, mpuData.az);
        DEBUG_PRINTF("  Gyro: %.2f, %.2f, %.2f\n", mpuData.gx, mpuData.gy, mpuData.gz);
        DEBUG_PRINTF("  Temp: %.1f°C\n", mpuData.temperature);
    }
}

// Set complementary filter alpha
void IMUHandler::setComplementaryFilterAlpha(float alpha) {
    if (alpha >= 0.0f && alpha <= 1.0f) {
        // Would need to add this as a member variable if adjustable
        // For now using the defined constant
    }
}

// Set Kalman filter parameters
void IMUHandler::setKalmanParameters(float Q_angle, float Q_gyro, float R_angle) {
    kalmanRoll.Q_angle = Q_angle;
    kalmanRoll.Q_gyro = Q_gyro;
    kalmanRoll.R_angle = R_angle;
    
    kalmanPitch.Q_angle = Q_angle;
    kalmanPitch.Q_gyro = Q_gyro;
    kalmanPitch.R_angle = R_angle;
}

// Get temperature
float IMUHandler::getTemperature() {
    switch (currentMode) {
        case IMU_MODE_QMI_ONLY:
            return qmiData.temperature;
        case IMU_MODE_MPU_ONLY:
            return mpuData.temperature;
        case IMU_MODE_FUSION:
            return fusedData.temperature;
        default:
            return 0.0f;
    }
}

// Clean up
void IMUHandler::end() {
    if (qmi8658) {
        delete qmi8658;
        qmi8658 = nullptr;
    }
    
    if (mpu6050) {
        delete mpu6050;
        mpu6050 = nullptr;
    }
    
    status.qmi_initialized = false;
    status.mpu_initialized = false;
}