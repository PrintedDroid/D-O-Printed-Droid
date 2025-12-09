
#include <Wire.h>
#include <IBusBM.h>
#include <Servo.h>
#include "DFRobotDFPlayerMini.h"
#include <MPU6050.h>

MPU6050 mpu;  // Create MPU instance

#define IBUS_ENABLED
#define DFPLAYER_ENABLED
#define SERVOS_ENABLED

// Pin Definitions for ESP32-S3 Mini
int dir1pin = 3;
int spe1pin = 2;
int dir2pin = 5;
int spe2pin = 4;

int ibus_pin = 6;
int dfplayer_rx = 1;  // RX Pin
int dfplayer_tx = 0;  // TX Pin

int servo3_pin = 7;
int servo4_pin = 8;
int servo5_pin = 9;
int servo6_pin = 10;

float baseline_angle = 0.0;  // Baseline angle for auto-leveling
float desired_angle = -0.3;  // Target angle, updated with baseline
float filtered_angle = 0.0;  // Filtered angle for smoother control
float pid_p = 20, pid_i = 20, pid_d = 0;
float kp = 25, ki = 0, kd = 0.8;

int numSamples = 100;  // Number of samples for auto-level
float alpha = 0.9;  // Low-pass filter coefficient

HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

void setup() {
    Serial.begin(115200);
    Wire.begin(11, 12);  // SDA on GPIO 11, SCL on GPIO 12 for ESP32-S3 Mini

    // Initialize MPU
    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.println("MPU connected");
    } else {
        Serial.println("MPU connection failed");
    }

    // Auto-level function
    autoLevel();

    // Initialize other components
    pinMode(dir1pin, OUTPUT);
    pinMode(spe1pin, OUTPUT);
    pinMode(dir2pin, OUTPUT);
    pinMode(spe2pin, OUTPUT);

#ifdef DFPLAYER_ENABLED
    mySerial.begin(9600, SERIAL_8N1, dfplayer_rx, dfplayer_tx);  // RX/TX on GPIO 1 and 0
    if (myDFPlayer.begin(mySerial)) {
        Serial.println("DFPlayer Mini online.");
    } else {
        Serial.println("DFPlayer Mini not detected.");
    }
#endif
}

// Function to calibrate the MPU to a zero angle baseline
void autoLevel() {
    float angleSum = 0;

    for (int i = 0; i < numSamples; i++) {
        int16_t accX, accY, accZ;
        mpu.getAcceleration(&accX, &accY, &accZ);

        // Calculate tilt angle using accelerometer data
        float angle = atan2(accY, accZ) * 180 / PI;
        angleSum += angle;

        delay(5);  // Short delay for quick calibration
    }

    baseline_angle = angleSum / numSamples;
    desired_angle = baseline_angle;  // Set desired angle based on baseline
    Serial.print("Auto-level baseline angle set to: ");
    Serial.println(baseline_angle);
}

// Low-pass filter to smooth the angle readings
float lowPassFilter(float currentAngle) {
    return alpha * filtered_angle + (1 - alpha) * currentAngle;
}

// Function to compute the PID control
float computePID(float current_angle) {
    static float previous_error = 0;
    static float integral = 0;

    float error = desired_angle - current_angle;
    integral += error;
    float derivative = error - previous_error;
    previous_error = error;

    return (kp * error) + (ki * integral) + (kd * derivative);
}

void loop() {
    // MPU Readings and filtering
    int16_t accX, accY, accZ;
    mpu.getAcceleration(&accX, &accY, &accZ);
    float angle = atan2(accY, accZ) * 180 / PI;

    // Apply low-pass filter
    filtered_angle = lowPassFilter(angle);
    
    // Calculate PID control output
    float pidOutput = computePID(filtered_angle);
    
    // Debugging information to assist in tuning and testing
    Serial.print("Filtered Angle: ");
    Serial.print(filtered_angle);
    Serial.print(" | PID Output: ");
    Serial.println(pidOutput);
    
    // Update motor control, servo, or other actuators here based on PID output
    // Example (pseudo-code, replace with actual motor control logic):
    // motorSpeedLeft = baseSpeed + pidOutput;
    // motorSpeedRight = baseSpeed - pidOutput;
}
