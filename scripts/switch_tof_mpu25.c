#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_VL53L0X.h>

// Motor Pins
#define M2_PWM PA1  // Right Motor PWM
#define M1_PWM PA6  // Left Motor PWM
#define M2_in1 PA2  // Right Motor Direction 1
#define M2_in2 PA3  // Right Motor Direction 2
#define M1_in1 PA4  // Left Motor Direction 1
#define M1_in2 PA5  // Left Motor Direction 2

// Encoder Pins
#define M2_ENC_B PB3  // Right Encoder A
#define M2_ENC_A PA15 // Right Encoder B
#define M1_ENC_A PB9  // Left Encoder A
#define M1_ENC_B PB8  // Left Encoder B

// TOF Sensor Pins
#define TOF_LEFT_XSHUT PB15  
#define TOF_CENTER_XSHUT PA9  
#define TOF_RIGHT_XSHUT PB13  

// Constants
#define MAXSPEED 85
#define MIN_OBSTACLE_DISTANCE 10  // Stop if obstacle is closer than 10 cm
#define COUNTS_PER_ROTATION 415
#define WHEEL_DIAMETER 4.4
#define DISTANCE_PER_TICK (PI * WHEEL_DIAMETER) / COUNTS_PER_ROTATION

// Global Variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
MPU6050 mpu(Wire);
Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofCenter;
Adafruit_VL53L0X tofRight;

// PID Parameters
// TOF-based wall following PID
float tofkp = 2, tofkd = 5;
float prevTofError = 0;

// Distance-based PID
float distkp = 1.0, distkd = 1.0;
float prevDistError = 0;

// MPU-based tilt correction PID
double kpT = 1, kiT = 0.0, kdT = 0.1;
double targetAngle = 0.0;
double tilt_error = 0, prev_tilt_error = 0, integral_tilt = 0;

// Wheel-specific PID
double kpL = 0.09, kdL = 1.1;   // Left Wheel PID
double kpR = 0.11, kdR = 1.2;   // Right Wheel PID

void leftEncoderISR() {
    leftEncoderCount += (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? 1 : -1;
}

void rightEncoderISR() {
    rightEncoderCount += (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? 1 : -1;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize ToF sensors
    pinMode(TOF_LEFT_XSHUT, OUTPUT);
    pinMode(TOF_CENTER_XSHUT, OUTPUT);
    pinMode(TOF_RIGHT_XSHUT, OUTPUT);
    digitalWrite(TOF_LEFT_XSHUT, LOW);
    digitalWrite(TOF_CENTER_XSHUT, LOW);
    digitalWrite(TOF_RIGHT_XSHUT, LOW);
    delay(10);

    // Initialize left ToF
    digitalWrite(TOF_LEFT_XSHUT, HIGH);
    delay(50);
    if (!tofLeft.begin()) {
        Serial.println("Failed to initialize Left ToF!");
    }
    tofLeft.setAddress(0x30);

    // Initialize center ToF
    digitalWrite(TOF_CENTER_XSHUT, HIGH);
    delay(50);
    if (!tofCenter.begin()) {
        Serial.println("Failed to initialize Center ToF!");
    }
    tofCenter.setAddress(0x31);

    // Initialize right ToF
    digitalWrite(TOF_RIGHT_XSHUT, HIGH);
    delay(50);
    if (!tofRight.begin()) {
        Serial.println("Failed to initialize Right ToF!");
    }
    tofRight.setAddress(0x32);

    // Initialize MPU6050
    mpu.begin();
    mpu.calcOffsets(true);  // Auto-calibrate

    // Motor control pins
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M1_in1, OUTPUT);
    pinMode(M1_in2, OUTPUT);
    pinMode(M2_in1, OUTPUT);
    pinMode(M2_in2, OUTPUT);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), rightEncoderISR, CHANGE);

    // Status LED
    pinMode(PB12, OUTPUT);
    digitalWrite(PB12, HIGH);

    delay(1000); // Initial stabilization
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -MAXSPEED, MAXSPEED);
    rightSpeed = constrain(rightSpeed, -MAXSPEED, MAXSPEED);

    digitalWrite(M1_in1, leftSpeed >= 0);
    digitalWrite(M1_in2, leftSpeed < 0);
    digitalWrite(M2_in1, rightSpeed >= 0);
    digitalWrite(M2_in2, rightSpeed < 0);

    analogWrite(M1_PWM, abs(leftSpeed));
    analogWrite(M2_PWM, abs(rightSpeed));
}

void brake() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}

int getDistance(Adafruit_VL53L0X &sensor) {
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);
    return (measure.RangeStatus != 4) ? measure.RangeMilliMeter / 10 : -1;
}

bool hasBothSideWalls(int leftDist, int rightDist) {
    // Consider walls present if they're between 5cm and 30cm away
    return (leftDist >= 4 && leftDist <= 8) && (rightDist >= 4 && rightDist <= 8);
}

void moveDistance(float targetDistance) {
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    
    double rotations_required = targetDistance / (PI * WHEEL_DIAMETER);
    double setpoint_counts = rotations_required * COUNTS_PER_ROTATION;
    double slow_down_distance = (5.0 / (PI * WHEEL_DIAMETER)) * COUNTS_PER_ROTATION;

    long previous_errorL = 0;
    long previous_errorR = 0;
    double errorL, errorR, control_signalL, control_signalR;
    int min_speed = 70; // Minimum speed to overcome friction

    while (true) {
        // Update sensors
        mpu.update();
        int leftDistance = getDistance(tofLeft);
        int rightDistance = getDistance(tofRight);
        int centerDistance = getDistance(tofCenter);

        // Check for obstacles
        if (centerDistance != -1 && centerDistance < MIN_OBSTACLE_DISTANCE) {
            Serial.println("Obstacle detected! Stopping.");
            brake();
            return;
        }

        // Calculate distance errors
        errorL = setpoint_counts - leftEncoderCount;
        errorR = setpoint_counts - rightEncoderCount;

        // Calculate current distance traveled
        float currentDistance = ((leftEncoderCount + rightEncoderCount) / 2) * DISTANCE_PER_TICK;
        float distanceError = targetDistance - currentDistance;

        // Determine control strategy based on wall presence
        float correction = 0;
        
        if (hasBothSideWalls(leftDistance, rightDistance)) {
            // TOF-based wall following when both walls are present
            int tofError = leftDistance - rightDistance;
            float derivative = tofError - prevTofError;
            correction = tofkp * tofError + tofkd * derivative;
            prevTofError = tofError;
            
            Serial.println("Using TOF correction");
        } else {
            // MPU-based tilt correction when walls are missing
            tilt_error = targetAngle - mpu.getAngleZ();
            integral_tilt += tilt_error;
            double derivative_tilt = tilt_error - prev_tilt_error;
            prev_tilt_error = tilt_error;
            
            correction = kpT * tilt_error + kiT * integral_tilt + kdT * derivative_tilt;
            Serial.println("Using MPU correction");
        }

        // Slow down as we approach target
        double speed_factor = 1.0;
        if (abs(errorL) < slow_down_distance && abs(errorR) < slow_down_distance) {
            speed_factor = abs(errorL) / slow_down_distance;
            speed_factor = constrain(speed_factor, 0.3, 1.0);
        }

        // Distance-based PID control
        float distDerivative = distanceError - prevDistError;
        float baseSpeed = distkp * distanceError + distkd * distDerivative;
        prevDistError = distanceError;

        // Apply minimum speed
        if (baseSpeed > 0 && baseSpeed < min_speed) baseSpeed = min_speed;
        if (baseSpeed < 0 && baseSpeed > -min_speed) baseSpeed = -min_speed;

        // Calculate final speeds with correction
        int leftSpeed = baseSpeed * speed_factor - correction;
        int rightSpeed = baseSpeed * speed_factor + correction;

        // Apply motor limits
        leftSpeed = constrain(leftSpeed, -MAXSPEED, MAXSPEED);
        rightSpeed = constrain(rightSpeed, -MAXSPEED, MAXSPEED);

        setMotorSpeed(leftSpeed, rightSpeed);

        // Debug output
        Serial.print("LDist: ");
        Serial.print(leftDistance);
        Serial.print(" RDist: ");
        Serial.print(rightDistance);
        Serial.print(" Tilt: ");
        Serial.print(mpu.getAngleX());
        Serial.print(" Correction: ");
        Serial.print(correction);
        Serial.print(" LSpd: ");
        Serial.print(leftSpeed);
        Serial.print(" RSpd: ");
        Serial.println(rightSpeed);

        // Stop condition
        if (abs(distanceError) < 0.5) {
            Serial.println("Target distance reached.");
            brake();
            return;
        }

        delay(10); // Small delay to prevent overwhelming the serial monitor
    }
}

void loop() {
    // Move exactly 50cm with combined TOF/MPU control
    moveDistance(25.0);
    
    // Wait before next movement
    delay(2000);
}