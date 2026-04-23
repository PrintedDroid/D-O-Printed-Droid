/**
 * IMU Handler Implementation
 *
 * Two-IMU-capable:
 *   - QMI8658C on TENSTAR module (I2C 0x6B, via SensorLib)
 *   - LSM6DS3  on AIO32 PCB       (I2C 0x6A, via direct register access)
 *
 * Modes: QMI_ONLY, LSM_ONLY, FUSION (simple averaging for now), AUTO.
 */

#include "imu_handler.h"
#include <SensorQMI8658.hpp>

#include <Preferences.h>
#include "esp_task_wdt.h"

// ---------------------------------------------------------------------------
// Apply a mount-orientation transform to an accel+gyro sample in place.
// Called by readQMI8658C() and readLSM6DS3() right after the raw data is
// in physical units (m/s² accel, deg/s gyro). Keeps mount handling in one
// place so a future variant just needs another case in this switch.
//
// Mount enum convention:
//   NORMAL       → x_droid = +x_chip, y_droid = +y_chip, z_droid = +z_chip
//   ROTATE_Y_180 → x_droid = -x_chip, y_droid = +y_chip, z_droid = -z_chip
//   ROTATE_X_180 → x_droid = +x_chip, y_droid = -y_chip, z_droid = -z_chip
//   ROTATE_Z_180 → x_droid = -x_chip, y_droid = -y_chip, z_droid = +z_chip
//   YAW_90_CW    → x_droid = +y_chip, y_droid = -x_chip, z_droid = +z_chip
//   YAW_90_CCW   → x_droid = -y_chip, y_droid = +x_chip, z_droid = +z_chip
//
// Gyro transforms exactly like accel under these pure rotations.
static void applyIMUMount(int mount,
                          float& ax, float& ay, float& az,
                          float& gx, float& gy, float& gz) {
    float tx, ty, tz;
    switch (mount) {
        case IMU_MOUNT_NORMAL:
        default:
            return;  // no change
        case IMU_MOUNT_ROTATE_Y_180:
            ax = -ax;           az = -az;
            gx = -gx;           gz = -gz;
            return;
        case IMU_MOUNT_ROTATE_X_180:
            ay = -ay;           az = -az;
            gy = -gy;           gz = -gz;
            return;
        case IMU_MOUNT_ROTATE_Z_180:
            ax = -ax;           ay = -ay;
            gx = -gx;           gy = -gy;
            return;
        case IMU_MOUNT_YAW_90_CW:
            tx = ax; ty = ay;   ax =  ty; ay = -tx;
            tx = gx; ty = gy;   gx =  ty; gy = -tx;
            return;
        case IMU_MOUNT_YAW_90_CCW:
            tx = ax; ty = ay;   ax = -ty; ay =  tx;
            tx = gx; ty = gy;   gx = -ty; gy =  tx;
            return;
    }
}

// Global instance
IMUHandler imuHandler;

// Preferences for calibration storage
Preferences preferences;

// Check I2C bus health
bool IMUHandler::checkI2CBus() {
    // Probe only the devices that are expected on this board. A full 1..126 scan
    // plus mid-scan Wire.end()/Wire.begin() recovery made ESP32-S3 startup brittle.
    const uint8_t probeAddresses[] = { QMI8658C_ADDRESS, BMP280_ADDRESS, LSM6DS3_ADDRESS };
    uint8_t deviceCount = 0;

    for (uint8_t i = 0; i < sizeof(probeAddresses); ++i) {
        uint8_t address = probeAddresses[i];
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            deviceCount++;
            DEBUG_PRINTF("IMU Handler: I2C device found at 0x%02X\n", address);
        } else if (error == 5) {
            DEBUG_PRINTF("IMU Handler: I2C timeout while probing 0x%02X\n", address);
        } else {
            DEBUG_PRINTF("IMU Handler: I2C probe 0x%02X failed with error %u\n", address, error);
        }

        if ((i & 1) == 1) {
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
    // LSM6DS3 uses no library object (direct I2C register access)
    currentMode(IMU_MODE_QMI_ONLY),  // Default to QMI only
    compFilterRoll(0.0f),
    compFilterPitch(0.0f),
    lastFilterUpdate(0) {
    
    // Initialize status
    memset(&status, 0, sizeof(IMUStatus));
    memset(&qmiCalibration, 0, sizeof(qmiCalibration));
    memset(&lsmCalibration, 0, sizeof(lsmCalibration));
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

    // The sketch already powers and initializes Wire in setup() before calling
    // imuHandler.begin(). Reinitializing it here destabilized startup on ESP32-S3.
    Wire.setClock(100000); // keep the intended bus speed
    delay(10);
    
    // Reset status
    memset(&status, 0, sizeof(IMUStatus));
    status.active_mode = mode;
    
    // Try to initialize QMI8658C with timeout
    if (USE_QMI8658C) {
        Serial.println("  [QMI] init start..."); Serial.flush();
        esp_task_wdt_reset();

        uint32_t startTime = millis();
        status.qmi_available = initQMI8658C();

        if (millis() - startTime > 1000) {
            Serial.println("  [QMI] init timeout"); Serial.flush();
            status.qmi_available = false;
        }

        Serial.printf("  [QMI] available=%s\n",
                      status.qmi_available ? "true" : "false");
        Serial.flush();
    } else {
        Serial.println("  [QMI] skipped (USE_QMI8658C=false)"); Serial.flush();
    }

    // Try to initialize LSM6DS3 (AIO32 PCB, I2C 0x6A) with timeout
    #if USE_LSM6DS3
    Serial.println("  [LSM] init start..."); Serial.flush();
    esp_task_wdt_reset();

    {
        uint32_t startTime = millis();
        status.lsm_available = initLSM6DS3();

        if (millis() - startTime > 1500) {
            Serial.println("  [LSM] init timeout"); Serial.flush();
            status.lsm_available = false;
        }

        Serial.printf("  [LSM] available=%s\n",
                      status.lsm_available ? "true" : "false");
        Serial.flush();
    }
    #else
    Serial.println("  [LSM] skipped (USE_LSM6DS3=false)"); Serial.flush();
    status.lsm_available = false;
    #endif

    // Load calibration data
    loadCalibration();

    // Initialize Kalman filters (kept as fallback when IMU_USE_MADGWICK=false)
    initKalmanFilter(kalmanRoll);
    initKalmanFilter(kalmanPitch);

#if IMU_USE_MADGWICK
    // Our Madgwick implementation (madgwick_ahrs.*) — set sample rate and
    // initial beta. Beta can be retuned at runtime via imuHandler.setMadgwickBeta()
    // or the CLI "imu beta <val>" command (persisted in NVS).
    madgwick_.begin((float)IMU_UPDATE_RATE);
    madgwick_.setBeta(IMU_MADGWICK_BETA);
    Serial.printf("  [IMU] Madgwick AHRS enabled @ %d Hz, beta=%.3f\n",
                  IMU_UPDATE_RATE, madgwick_.getBeta());
    Serial.flush();
#endif

    // Determine actual mode based on availability. AUTO prefers LSM6DS3
    // (frame-fixed on the AIO32 PCB) over the TENSTAR-module-mounted QMI.
    if (mode == IMU_MODE_AUTO) {
        if (status.qmi_available && status.lsm_available) {
            currentMode = IMU_MODE_FUSION;
            DEBUG_PRINTLN("IMU Handler: Using FUSION mode (both IMUs)");
        } else if (status.lsm_available) {
            currentMode = IMU_MODE_LSM_ONLY;
            DEBUG_PRINTLN("IMU Handler: Using LSM6DS3 only");
        } else if (status.qmi_available) {
            currentMode = IMU_MODE_QMI_ONLY;
            DEBUG_PRINTLN("IMU Handler: Using QMI8658C only");
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
    return (status.qmi_available || status.lsm_available);
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
        
        // Avoid repeated begin() calls; SensorLib reconfigures the bus internally.
        bool initSuccess = qmi8658->begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN);
        esp_task_wdt_reset();
        
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

// Initialize LSM6DS3 via direct I2C register access (no library).
// The LSM6DS3 sits on the AIO32 PCB at I2C 0x6A (SA0 low). Same register-
// level approach used by the Mega sketch for MPU6050 clone compatibility.
//
// Key registers used:
//   0x0F WHO_AM_I          expected 0x69 (LSM6DS3) or 0x6A (LSM6DS3TR-C)
//   0x10 CTRL1_XL          accelerometer: ODR + full-scale + BW
//   0x11 CTRL2_G           gyroscope: ODR + full-scale
//   0x12 CTRL3_C           common: IF_INC (auto-increment), BDU (block update)
//   0x22..0x27 OUTX_L_G..  gyroscope  raw (LE int16, 6 bytes)
//   0x28..0x2D OUTX_L_XL.. accelerometer raw (LE int16, 6 bytes)
//   0x20 OUT_TEMP_L        temperature raw (LE int16)
bool IMUHandler::initLSM6DS3() {
    // Unconditional logging for this path — DEBUG_MODE=0 in production builds
    // otherwise swallows the diagnostic trail that tells us why a detect fails.
    Serial.printf("  [LSM] probing 0x%02X...\n", LSM6DS3_ADDRESS); Serial.flush();

    // Bus-state reset before probing. Symptom seen on AIO32 v2.1: after the
    // SensorQMI8658 library initialises the QMI on 0x6B, the subsequent
    // address probe on 0x6A returns I2C error 2 (NACK on address) even
    // though a minimal scanner sketch finds the chip cleanly. Explanation
    // is that the library leaves the bus at a non-standard clock or with a
    // pending transaction. A plain setClock() call plus a tiny settling
    // delay restores the bus to the state the LSM6DS3 expects.
    Wire.setClock(100000);
    delay(5);
    esp_task_wdt_reset();

    // Check if device responds on the bus (quick probe, no full bus scan).
    uint8_t retries = 3;
    uint8_t lastError = 0xFF;
    while (retries > 0) {
        Wire.beginTransmission(LSM6DS3_ADDRESS);
        lastError = Wire.endTransmission();
        if (lastError == 0) break;
        Serial.printf("  [LSM] probe I2C error %d\n", lastError); Serial.flush();
        retries--;
        if (retries > 0) {
            // More aggressive recovery between retries: release the driver,
            // wait, then re-bind the pins. This clears any pending START
            // condition or stuck slave from the previous library.
            Wire.end();
            delay(50);
            Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
            Wire.setClock(100000);
            delay(10);
            esp_task_wdt_reset();
        }
    }
    if (retries == 0) {
        Serial.printf("  [LSM] not responding at 0x%02X (last err=%d)\n",
                      LSM6DS3_ADDRESS, lastError);
        Serial.flush();
        // As a final diagnostic: scan a tiny neighbourhood (0x68..0x6F) and
        // report which addresses DO ack. If the chip is actually at 0x6B
        // (SA0 floated high) we want to see that in the log so the user can
        // change LSM6DS3_ADDRESS in config.h without guessing.
        Serial.print("  [LSM] neighbourhood scan (0x68..0x6F):"); Serial.flush();
        for (uint8_t addr = 0x68; addr <= 0x6F; ++addr) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                Serial.printf(" 0x%02X", addr);
            }
        }
        Serial.println(); Serial.flush();
        return false;
    }
    Serial.println("  [LSM] ACK at probe"); Serial.flush();

    // Read WHO_AM_I. Known-good values:
    //   0x69  LSM6DS3 (original), LSM6DS3H
    //   0x6A  LSM6DS3TR-C, LSM6DSL, LSM6DSM
    //   0x6C  LSM6DSO, LSM6DSOX, LSM6DSR
    // We accept any of these because the GY-LSM6DS3 module could ship with any
    // of them depending on stock batch.
    Wire.beginTransmission(LSM6DS3_ADDRESS);
    Wire.write(0x0F);
    if (Wire.endTransmission(false) != 0) {
        Serial.println("  [LSM] WHO_AM_I write addr failed"); Serial.flush();
        return false;
    }
    Wire.requestFrom((int)LSM6DS3_ADDRESS, 1);
    if (Wire.available() < 1) {
        Serial.println("  [LSM] WHO_AM_I read timeout"); Serial.flush();
        return false;
    }
    uint8_t whoami = Wire.read();
    Serial.printf("  [LSM] WHO_AM_I = 0x%02X\n", whoami); Serial.flush();
    if (whoami != 0x69 && whoami != 0x6A && whoami != 0x6C) {
        Serial.println("  [LSM] WHO_AM_I unrecognized"); Serial.flush();
        return false;
    }

    // Configure accelerometer: 416 Hz ODR, 4 g full-scale.
    //   CTRL1_XL = 0110 10 00  = 0x68  (ODR=416Hz[0110], FS=4g[10], BW=auto[00])
    Wire.beginTransmission(LSM6DS3_ADDRESS);
    Wire.write(0x10);
    Wire.write(0x68);
    if (Wire.endTransmission() != 0) {
        DEBUG_PRINTLN("IMU Handler: LSM6DS3 CTRL1_XL write failed");
        return false;
    }

    // Configure gyroscope: 416 Hz ODR, 500 dps full-scale.
    //   CTRL2_G = 0110 01 00  = 0x64  (ODR=416Hz[0110], FS=500dps[01], 0[0])
    Wire.beginTransmission(LSM6DS3_ADDRESS);
    Wire.write(0x11);
    Wire.write(0x64);
    if (Wire.endTransmission() != 0) {
        DEBUG_PRINTLN("IMU Handler: LSM6DS3 CTRL2_G write failed");
        return false;
    }

    // Common control: enable block-data-update + register auto-increment.
    //   CTRL3_C = 0100 0100 = 0x44  (BDU=1, IF_INC=1)
    Wire.beginTransmission(LSM6DS3_ADDRESS);
    Wire.write(0x12);
    Wire.write(0x44);
    if (Wire.endTransmission() != 0) {
        DEBUG_PRINTLN("IMU Handler: LSM6DS3 CTRL3_C write failed");
        return false;
    }

    status.lsm_initialized = true;
    DEBUG_PRINTLN("IMU Handler: LSM6DS3 configured @ 416 Hz, 4 g, 500 dps");
    return true;
}

// Main update function
bool IMUHandler::update() {
    if (!status.qmi_initialized && !status.lsm_initialized) {
        return false;
    }

    uint32_t now = millis();

    // Read sensor data based on mode (always read; the fusion step below
    // decides which source drives the orientation output).
    switch (currentMode) {
        case IMU_MODE_QMI_ONLY:
            if (status.qmi_initialized) readQMI8658C();
            break;
        case IMU_MODE_LSM_ONLY:
            if (status.lsm_initialized) readLSM6DS3();
            break;
        case IMU_MODE_FUSION:
            if (status.qmi_initialized) readQMI8658C();
            if (status.lsm_initialized) readLSM6DS3();
            fuseIMUData();
            break;
        case IMU_MODE_AUTO:
            // AUTO is resolved at begin() into one of the concrete modes;
            // this case is defensive and should not be reached at runtime.
            break;
    }

#if IMU_USE_MADGWICK
    // Pick the driving raw-data source for the AHRS filter:
    //   QMI_ONLY   -> qmiData
    //   LSM_ONLY   -> lsmData
    //   FUSION     -> fusedData (averaged by fuseIMUData())
    RawIMUData* src = nullptr;
    switch (currentMode) {
        case IMU_MODE_QMI_ONLY: src = &qmiData; break;
        case IMU_MODE_LSM_ONLY: src = &lsmData; break;
        case IMU_MODE_FUSION:   src = &fusedData; break;
        default: break;
    }

    if (src) {
        // Madgwick wants gyro in deg/s (already correct in our RawIMUData)
        // and accel in "g" (we store m/s^2 in RawIMUData; divide by 9.80665).
        const float INV_G = 1.0f / 9.80665f;
        madgwick_.updateIMU(
            src->gx, src->gy, src->gz,
            src->ax * INV_G, src->ay * INV_G, src->az * INV_G);

        orientation.roll  = madgwick_.getRoll();
        orientation.pitch = madgwick_.getPitch();
        orientation.yaw   = madgwick_.getYaw();
        orientation.roll_rate  = src->gx;
        orientation.pitch_rate = src->gy;
        orientation.yaw_rate   = src->gz;
        orientation.timestamp  = now;
    }
#else
    // Legacy path: complementary filter + Kalman per source
    switch (currentMode) {
        case IMU_MODE_QMI_ONLY:
            if (status.qmi_initialized) updateOrientation(qmiData, orientation);
            break;
        case IMU_MODE_LSM_ONLY:
            if (status.lsm_initialized) updateOrientation(lsmData, orientation);
            break;
        case IMU_MODE_FUSION:
            updateOrientation(fusedData, orientation);
            break;
        case IMU_MODE_AUTO:
            break;
    }
#endif

    // Update status
    status.last_update = now;
    status.update_count++;
    status.update_rate = calculateUpdateRate();

    return true;
}

// Read QMI8658C data — with unit conversion + axis remap to droid frame.
//
// SensorQMI8658 library returns:
//   - Accelerometer in units of g (gravity), NOT m/s² (the old comment lied)
//   - Gyroscope   in deg/s (correct)
//
// Mount transform is applied via the shared applyIMUMount() helper, using
// the per-chip config knobs IMU_QMI_MOUNT + QMI_ACCEL_SCALE_G_TO_MS2 from
// config.h. Change the enum there for a different PCB layout.
void IMUHandler::readQMI8658C() {
    if (!qmi8658) {
        DEBUG_PRINTLN("IMU Handler: QMI8658C null pointer!");
        return;
    }

    float ax_raw = 0, ay_raw = 0, az_raw = 0;
    float gx_raw = 0, gy_raw = 0, gz_raw = 0;

    if (!qmi8658->getAccelerometer(ax_raw, ay_raw, az_raw)) {
        DEBUG_PRINTLN("IMU Handler: Failed to read QMI8658C accelerometer");
    }
    if (!qmi8658->getGyroscope(gx_raw, gy_raw, gz_raw)) {
        DEBUG_PRINTLN("IMU Handler: Failed to read QMI8658C gyroscope");
    }

#if QMI_ACCEL_SCALE_G_TO_MS2
    // Library returns g — scale to m/s² so it matches the LSM6DS3 pipeline
    // and downstream Madgwick expectations (we divide by G there).
    const float G = 9.80665f;
    ax_raw *= G;
    ay_raw *= G;
    az_raw *= G;
#endif

    // Apply per-chip mount transform (see config.h IMU_QMI_MOUNT)
    applyIMUMount(IMU_QMI_MOUNT,
                  ax_raw, ay_raw, az_raw,
                  gx_raw, gy_raw, gz_raw);

    qmiData.ax = ax_raw;
    qmiData.ay = ay_raw;
    qmiData.az = az_raw;
    qmiData.gx = gx_raw;
    qmiData.gy = gy_raw;
    qmiData.gz = gz_raw;

    qmiData.temperature = qmi8658->getTemperature_C();

    // Apply calibration offsets
    calibrateIMU(qmiData, true);
}

// Read LSM6DS3 via direct register burst. Layout matches ST datasheet
// (auto-increment is enabled in initLSM6DS3 via CTRL3_C.IF_INC=1).
//
// Sensitivities at our configured full-scale settings (see initLSM6DS3):
//   Accel @ +-4 g  -> 0.122 mg/LSB -> 0.122e-3 * 9.80665 m/s^2 per LSB
//   Gyro  @ 500dps -> 17.50 mdps/LSB -> 0.01750 deg/s per LSB
void IMUHandler::readLSM6DS3() {
    if (!status.lsm_initialized) return;

    // Read 6 bytes of gyro starting at OUTX_L_G (0x22)
    Wire.beginTransmission(LSM6DS3_ADDRESS);
    Wire.write(0x22);
    if (Wire.endTransmission(false) != 0) return;
    Wire.requestFrom((int)LSM6DS3_ADDRESS, 6);
    if (Wire.available() < 6) return;

    int16_t gxRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t gyRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t gzRaw = (int16_t)(Wire.read() | (Wire.read() << 8));

    // Read 6 bytes of accel starting at OUTX_L_XL (0x28)
    Wire.beginTransmission(LSM6DS3_ADDRESS);
    Wire.write(0x28);
    if (Wire.endTransmission(false) != 0) return;
    Wire.requestFrom((int)LSM6DS3_ADDRESS, 6);
    if (Wire.available() < 6) return;

    int16_t axRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t ayRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t azRaw = (int16_t)(Wire.read() | (Wire.read() << 8));

    // Accelerometer: 0.122 mg/LSB @ +-4g -> m/s^2
    const float ACC_SCALE = 0.000122f * 9.80665f;
    float axVal = axRaw * ACC_SCALE;
    float ayVal = ayRaw * ACC_SCALE;
    float azVal = azRaw * ACC_SCALE;

    // Gyroscope: 17.50 mdps/LSB @ 500 dps -> deg/s
    const float GYR_SCALE = 0.01750f;
    float gxVal = gxRaw * GYR_SCALE;
    float gyVal = gyRaw * GYR_SCALE;
    float gzVal = gzRaw * GYR_SCALE;

    // Apply per-chip mount transform (see config.h IMU_LSM_MOUNT).
    // On AIO32 v2.1 LSM is IMU_MOUNT_NORMAL so this is a no-op; the
    // helper is called unconditionally so future PCB variants with
    // differently-oriented LSMs just flip the config enum.
    applyIMUMount(IMU_LSM_MOUNT,
                  axVal, ayVal, azVal,
                  gxVal, gyVal, gzVal);

    lsmData.ax = axVal;
    lsmData.ay = ayVal;
    lsmData.az = azVal;
    lsmData.gx = gxVal;
    lsmData.gy = gyVal;
    lsmData.gz = gzVal;

    // Temperature: OUT_TEMP_L/H at 0x20 (16-bit, 256 LSB/degC, 25 degC bias)
    Wire.beginTransmission(LSM6DS3_ADDRESS);
    Wire.write(0x20);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((int)LSM6DS3_ADDRESS, 2);
        if (Wire.available() >= 2) {
            int16_t tRaw = (int16_t)(Wire.read() | (Wire.read() << 8));
            lsmData.temperature = (tRaw / 256.0f) + 25.0f;
        }
    }

    // Apply calibration offsets (stored in lsmCalibration)
    calibrateIMU(lsmData, false);
}

// Fuse data from both IMUs (QMI8658C + LSM6DS3). Simple averaging for now;
// upgrade path is weighted fusion based on per-sensor variance or Kalman.
void IMUHandler::fuseIMUData() {
    if (status.qmi_initialized && status.lsm_initialized) {
        fusedData.ax = (qmiData.ax + lsmData.ax) * 0.5f;
        fusedData.ay = (qmiData.ay + lsmData.ay) * 0.5f;
        fusedData.az = (qmiData.az + lsmData.az) * 0.5f;
        fusedData.gx = (qmiData.gx + lsmData.gx) * 0.5f;
        fusedData.gy = (qmiData.gy + lsmData.gy) * 0.5f;
        fusedData.gz = (qmiData.gz + lsmData.gz) * 0.5f;
        fusedData.temperature = (qmiData.temperature + lsmData.temperature) * 0.5f;
    } else if (status.qmi_initialized) {
        fusedData = qmiData;
    } else if (status.lsm_initialized) {
        fusedData = lsmData;
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
    float lsm_ax_sum = 0, lsm_ay_sum = 0, lsm_az_sum = 0;
    float lsm_gx_sum = 0, lsm_gy_sum = 0, lsm_gz_sum = 0;

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

        if (status.lsm_initialized) {
            readLSM6DS3();
            lsm_ax_sum += lsmData.ax;
            lsm_ay_sum += lsmData.ay;
            lsm_az_sum += lsmData.az;
            lsm_gx_sum += lsmData.gx;
            lsm_gy_sum += lsmData.gy;
            lsm_gz_sum += lsmData.gz;
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

    if (status.lsm_initialized) {
        lsmCalibration.ax_offset = lsm_ax_sum / samples;
        lsmCalibration.ay_offset = lsm_ay_sum / samples;
        lsmCalibration.az_offset = lsm_az_sum / samples - 9.81f; // Gravity
        lsmCalibration.gx_offset = lsm_gx_sum / samples;
        lsmCalibration.gy_offset = lsm_gy_sum / samples;
        lsmCalibration.gz_offset = lsm_gz_sum / samples;
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
        data.ax -= lsmCalibration.ax_offset;
        data.ay -= lsmCalibration.ay_offset;
        data.az -= lsmCalibration.az_offset;
        data.gx -= lsmCalibration.gx_offset;
        data.gy -= lsmCalibration.gy_offset;
        data.gz -= lsmCalibration.gz_offset;
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
    
    // Save LSM6DS3 calibration
    preferences.putFloat("lsm_ax", lsmCalibration.ax_offset);
    preferences.putFloat("lsm_ay", lsmCalibration.ay_offset);
    preferences.putFloat("lsm_az", lsmCalibration.az_offset);
    preferences.putFloat("lsm_gx", lsmCalibration.gx_offset);
    preferences.putFloat("lsm_gy", lsmCalibration.gy_offset);
    preferences.putFloat("lsm_gz", lsmCalibration.gz_offset);
    
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
    
    // Load LSM6DS3 calibration
    lsmCalibration.ax_offset = preferences.getFloat("lsm_ax", 0.0f);
    lsmCalibration.ay_offset = preferences.getFloat("lsm_ay", 0.0f);
    lsmCalibration.az_offset = preferences.getFloat("lsm_az", 0.0f);
    lsmCalibration.gx_offset = preferences.getFloat("lsm_gx", 0.0f);
    lsmCalibration.gy_offset = preferences.getFloat("lsm_gy", 0.0f);
    lsmCalibration.gz_offset = preferences.getFloat("lsm_gz", 0.0f);
    
    preferences.end();
}

// Reset calibration
void IMUHandler::resetCalibration() {
    memset(&qmiCalibration, 0, sizeof(qmiCalibration));
    memset(&lsmCalibration, 0, sizeof(lsmCalibration));
}

// Print calibration offsets to a stream. Accel offsets are in m/s², gyro
// offsets in deg/s — same units as the raw data they are subtracted from.
// Z accel is adjusted for the 9.81 m/s² gravity baseline (a calibrated
// sensor at rest reports az_offset ≈ 0 after that subtraction).
void IMUHandler::printBias(Stream& out) {
    out.println(F("IMU bias offsets (subtracted from raw reads):"));
    out.printf ("  QMI  accel: %+7.3f, %+7.3f, %+7.3f  (m/s^2)\n",
                qmiCalibration.ax_offset,
                qmiCalibration.ay_offset,
                qmiCalibration.az_offset);
    out.printf ("  QMI  gyro : %+7.3f, %+7.3f, %+7.3f  (deg/s)\n",
                qmiCalibration.gx_offset,
                qmiCalibration.gy_offset,
                qmiCalibration.gz_offset);
    out.printf ("  LSM  accel: %+7.3f, %+7.3f, %+7.3f  (m/s^2)\n",
                lsmCalibration.ax_offset,
                lsmCalibration.ay_offset,
                lsmCalibration.az_offset);
    out.printf ("  LSM  gyro : %+7.3f, %+7.3f, %+7.3f  (deg/s)\n",
                lsmCalibration.gx_offset,
                lsmCalibration.gy_offset,
                lsmCalibration.gz_offset);
    out.println(F("(run 'imu cal' with the droid stationary to refresh.)"));
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
    if (!status.qmi_initialized && !status.lsm_initialized) {
        return false;
    }

    // Check update rate. In STATE_READY the main loop polls at 50 Hz
    // (20 ms interval) which jitters around 48–52 Hz measured; a 50 Hz
    // hard threshold kicks `isHealthy()` to false intermittently and
    // blocks state-reactions like tilt-warning. Use 30 Hz as the "still
    // usable" floor — below that the filter really isn't tracking.
    if (status.update_rate < 30.0f) {
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
        case IMU_MODE_LSM_ONLY: strcpy(modeStr, "LSM Only"); break;
        case IMU_MODE_FUSION:   strcpy(modeStr, "Fusion");   break;
        case IMU_MODE_AUTO:     strcpy(modeStr, "Auto");     break;
        default:                strcpy(modeStr, "Unknown");  break;
    }

    // LSM status needs three possible states because the sensor is compile-
    // gated behind USE_LSM6DS3: "not compiled", "compiled but absent", "OK".
    const char* lsmStatus;
    #if USE_LSM6DS3
        lsmStatus = status.lsm_initialized ? "OK" : "FAIL";
    #else
        lsmStatus = "off";
    #endif

    snprintf(buffer, bufferSize,
             "IMU Status:\n"
             "Mode: %s\n"
             "QMI8658C: %s\n"
             "LSM6DS3: %s\n"
             "Update Rate: %.1f Hz\n"
             "Roll: %.1f\xB0\n"
             "Pitch: %.1f\xB0\n"
             "Yaw: %.1f\xB0",
             modeStr,
             status.qmi_initialized ? "OK" : "FAIL",
             lsmStatus,
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
    
    if (status.lsm_initialized) {
        DEBUG_PRINTLN("LSM6DS3 Data:");
        DEBUG_PRINTF("  Accel: %.2f, %.2f, %.2f\n", lsmData.ax, lsmData.ay, lsmData.az);
        DEBUG_PRINTF("  Gyro: %.2f, %.2f, %.2f\n", lsmData.gx, lsmData.gy, lsmData.gz);
        DEBUG_PRINTF("  Temp: %.1f\xB0""C\n", lsmData.temperature);
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
        case IMU_MODE_LSM_ONLY:
            return lsmData.temperature;
        case IMU_MODE_FUSION:
            return fusedData.temperature;
        default:
            return 0.0f;
    }
}

// Switch the active IMU mode at runtime. Validates that the requested
// sensor(s) are actually initialised before accepting the change — callers
// (e.g. the B5-long cycleIMUMode handler) are expected to filter based on
// availability already, but this is the defensive second line.
bool IMUHandler::setMode(IMUMode mode) {
    switch (mode) {
        case IMU_MODE_QMI_ONLY:
            if (!status.qmi_initialized) return false;
            break;
        case IMU_MODE_LSM_ONLY:
            if (!status.lsm_initialized) return false;
            break;
        case IMU_MODE_FUSION:
            if (!(status.qmi_initialized && status.lsm_initialized)) return false;
            break;
        case IMU_MODE_AUTO:
            // AUTO only makes sense during begin(); refuse at runtime so the
            // active mode is always a concrete choice.
            return false;
    }
    currentMode = mode;
    status.active_mode = mode;
    return true;
}

// Clean up
void IMUHandler::end() {
    if (qmi8658) {
        delete qmi8658;
        qmi8658 = nullptr;
    }
    // LSM6DS3 has no object to free (direct register access)

    status.qmi_initialized = false;
    status.lsm_initialized = false;
}
