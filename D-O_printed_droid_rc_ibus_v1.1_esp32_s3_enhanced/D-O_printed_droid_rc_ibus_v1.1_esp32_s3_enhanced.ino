
#include <Wire.h>
#include <Servo.h>
#include "DFRobotDFPlayerMini.h"
#include <MPU6050.h>
#include <EEPROM.h>

// Libraries for IBUS and SBUS
#include <IBusBM.h>
#include <SBUS.h>
#include <ESP32Servo.h>

MPU6050 mpu;  // Create MPU instance
IBusBM ibus;  // Create IBUS instance
SBUS sbus(1); // Use UART1 for SBUS

#define DFPLAYER_ENABLED
#define SERVOS_ENABLED

// Receiver type selection
#define USE_SBUS true  // Set to 'false' if using IBUS

// Pin Definitions
int dir1pin = 15;
int spe1pin = 14;
int dir2pin = 17;
int spe2pin = 16;

int ibus_sbus_pin = 18;
int servo3_pin = 22;
int servo4_pin = 23;
int servo5_pin = 25;
int servo6_pin = 26;

float baseline_angle = 0.0;
float desired_angle = -0.3;
float filtered_angle = 0.0;
float pid_p = 20, pid_i = 20, pid_d = 0;
float kp = 25, ki = 0, kd = 0.8;

int numSamples = 100;
float alpha = 0.9;
bool calibration_success = false;

HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

void setup() {
    Serial.begin(115200);
    Wire.begin(8, 9);  // SDA on GPIO 8, SCL on GPIO 9 for ESP32-S3

    // Initialize EEPROM for saving PID values
    EEPROM.begin(64);

    // Initialize MPU with error handling
    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.println("MPU connected");
    } else {
        Serial.println("MPU connection failed");
    }

    // Choose between SBUS and IBUS
    if (USE_SBUS) {
        sbus.begin();
        Serial.println("SBUS activated.");
    } else {
        ibus.begin(Serial1, 115200);  // IBUS on Serial1 with 115200 baud
        Serial.println("IBUS activated.");
    }

#ifdef DFPLAYER_ENABLED
    mySerial.begin(9600, SERIAL_8N1, 21, 19);  // RX on GPIO 21, TX on GPIO 19
    if (myDFPlayer.begin(mySerial)) {
        Serial.println("DFPlayer Mini online.");
    } else {
        Serial.println("DFPlayer Mini not detected.");
    }
#endif

    // Auto-level initialization
    autoLevel();
}

// Function to calibrate MPU to a zero-angle baseline with feedback
void autoLevel() {
    float angleSum = 0;
    int failed_attempts = 0;

    for (int i = 0; i < numSamples; i++) {
        int16_t accX, accY, accZ;
        mpu.getAcceleration(&accX, &accY, &accZ);
        float angle = atan2(accY, accZ) * 180 / PI;
        angleSum += angle;

        // Retry if readings are inconsistent
        if (abs(angle) > 5) {
            failed_attempts++;
            if (failed_attempts > 3) {
                Serial.println("Calibration failed - retrying...");
                i = 0;  // Restart calibration
                angleSum = 0;
                failed_attempts = 0;
            }
        }
        delay(5);
    }

    baseline_angle = angleSum / numSamples;
    desired_angle = baseline_angle;
    calibration_success = true;
    Serial.print("Auto-level baseline angle set to: ");
    Serial.println(baseline_angle);
}

// Low-pass filter to smooth angle readings
float lowPassFilter(float currentAngle) {
    return alpha * filtered_angle + (1 - alpha) * currentAngle;
}

// Function to compute the PID control output
float computePID(float current_angle) {
    static float previous_error = 0;
    static float integral = 0;

    float error = desired_angle - current_angle;
    integral += error;
    float derivative = error - previous_error;
    previous_error = error;

    return (kp * error) + (ki * integral) + (kd * derivative);
}

// PID tuning interface through Serial Monitor
void pidTuning() {
    if (Serial.available()) {
        char command = Serial.read();
        switch (command) {
            case 'p':
                kp += 1; break;
            case 'i':
                ki += 0.1; break;
            case 'd':
                kd += 0.1; break;
            case 'P':
                kp -= 1; break;
            case 'I':
                ki -= 0.1; break;
            case 'D':
                kd -= 0.1; break;
            case 's':
                savePIDValues(); break;
        }
        Serial.print("PID Values - Kp: "); Serial.print(kp);
        Serial.print(", Ki: "); Serial.print(ki);
        Serial.print(", Kd: "); Serial.println(kd);
    }
}

// Save PID values to EEPROM
void savePIDValues() {
    EEPROM.writeFloat(0, kp);
    EEPROM.writeFloat(4, ki);
    EEPROM.writeFloat(8, kd);
    EEPROM.commit();
    Serial.println("PID values saved to EEPROM");
}

// Load PID values from EEPROM
void loadPIDValues() {
    kp = EEPROM.readFloat(0);
    ki = EEPROM.readFloat(4);
    kd = EEPROM.readFloat(8);
    Serial.println("PID values loaded from EEPROM");
}

void loop() {
    // Load saved PID values if not loaded yet
    loadPIDValues();

    // PID tuning interface
    pidTuning();

    // Read receiver data based on SBUS or IBUS selection
    if (USE_SBUS) {
        if (sbus.read()) {
            uint16_t channels[16];
            bool failsafe, lost_frame;

            sbus.getReceivedChannels(channels, failsafe, lost_frame);
            int control = channels[0];
            Serial.print("SBUS Channel 0: ");
            Serial.println(control);
        }
    } else {
        if (ibus.readChannel(0)) {
            int control = ibus.getChannel(0);
            Serial.print("IBUS Channel 0: ");
            Serial.println(control);
        }
    }

    // MPU reading and filtering
    int16_t accX, accY, accZ;
    mpu.getAcceleration(&accX, &accY, &accZ);
    float angle = atan2(accY, accZ) * 180 / PI;
    filtered_angle = lowPassFilter(angle);
    
    float pidOutput = computePID(filtered_angle);
    
    // Debugging output for tuning and calibration
    Serial.print("Filtered Angle: ");
    Serial.print(filtered_angle);
    Serial.print(" | PID Output: ");
    Serial.println(pidOutput);
}
