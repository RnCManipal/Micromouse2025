#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_VL53L0X.h>

#define M2_PWM PA1  // Right Motor PWM
#define M1_PWM PA6  // Left Motor PWM
#define M2_in1 PA2  // Right Motor Direction 1
#define M2_in2 PA3  // Right Motor Direction 2
#define M1_in1 PA4  // Left Motor Direction 1
#define M1_in2 PA5  // Left Motor Direction 2

#define M2_ENC_B PB3  // Right Encoder A
#define M2_ENC_A PA15 // Right Encoder B
#define M1_ENC_A PB9  // Left Encoder A
#define M1_ENC_B PB8  // Left Encoder B

float targetDistance = 25.0; // Target distance in cm
float wheelDiameter = 4.4;   // Wheel diameter in cm
int encoderTicksPerRev = 410;  // Ticks per revolution
float distancePerTick = (PI * wheelDiameter) / encoderTicksPerRev;
long targetCounts = targetDistance / distancePerTick;
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

#define TOF_LEFT_XSHUT PB15  
#define TOF_CENTER_XSHUT PA9  
#define TOF_RIGHT_XSHUT PB13  
#define MAXSPEED 85
#define MIN_OBSTACLE_DISTANCE 0  // Stop if an obstacle is closer than 10 cm

Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofCenter;
Adafruit_VL53L0X tofRight;

// PID parameters
float tofkp = 7.0, tofkd = 18.0;
float distkp=1.0,distkd=1.0;
float prevError = 0;

void leftEncoderISR() {
    leftEncoderCount += (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? 1 : -1;
}

void rightEncoderISR() {
    rightEncoderCount += (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? 1 : -1;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(TOF_LEFT_XSHUT, OUTPUT);
    pinMode(TOF_CENTER_XSHUT, OUTPUT);
    pinMode(TOF_RIGHT_XSHUT, OUTPUT);

    pinMode(PB12, OUTPUT);
    digitalWrite(PB12, HIGH);

    // Reset and initialize ToF sensors
    digitalWrite(TOF_LEFT_XSHUT, LOW);
    digitalWrite(TOF_CENTER_XSHUT, LOW);
    digitalWrite(TOF_RIGHT_XSHUT, LOW);
    delay(10);

    digitalWrite(TOF_LEFT_XSHUT, HIGH);
    delay(50);
    if (!tofLeft.begin()) {
        Serial.println("Failed to initialize Left ToF! Reattempting...");
    }
    tofLeft.setAddress(0x30);

    digitalWrite(TOF_CENTER_XSHUT, HIGH);
    delay(50);
    if (!tofCenter.begin()) {
        Serial.println("Failed to initialize Center ToF! Reattempting...");
    }
    tofCenter.setAddress(0x31);

    digitalWrite(TOF_RIGHT_XSHUT, HIGH);
    delay(50);
    if (!tofRight.begin()) {
        Serial.println("Failed to initialize Right ToF! Reattempting...");
    }
    tofRight.setAddress(0x32);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M1_in1, OUTPUT);
    pinMode(M1_in2, OUTPUT);
    pinMode(M2_in1, OUTPUT);
    pinMode(M2_in2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), rightEncoderISR, CHANGE);
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

float computePID(int error, float kp, float kd) {
    float derivative = error - prevError;
    float correction = kp * error + kd * derivative;
    prevError = error;
    return correction;
}


int getDistance(Adafruit_VL53L0X &sensor) {
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);
    return (measure.RangeStatus != 4) ? measure.RangeMilliMeter / 10 : -1;
}

void moveForward(int targetdist) {
    int leftDistance = getDistance(tofLeft);
    int rightDistance = getDistance(tofRight);
    int centerDistance = getDistance(tofCenter);

    if (leftDistance == -1) leftDistance = getDistance(tofLeft);
    if (rightDistance == -1) rightDistance = getDistance(tofRight);
    if (centerDistance == -1) centerDistance = getDistance(tofCenter);

    if (leftDistance == -1 || rightDistance == -1 || centerDistance == -1) {
        Serial.println("Sensor error! Stopping motors.");
        setMotorSpeed(0, 0);
        return;
    }

    if (centerDistance < MIN_OBSTACLE_DISTANCE) {
        Serial.println("Obstacle detected! Stopping.");
        setMotorSpeed(0, 0);
        return;
    }

    // Distance traveled based on encoder counts
    int currentdist = ((leftEncoderCount + rightEncoderCount) / 2) * distancePerTick;
    int errordist = targetdist - currentdist;

    // PID-based distance control
    static float prevDistError = 0;
    float distspeed = distkp * errordist + distkd * (errordist - prevDistError);
    prevDistError = errordist;

    // Apply minimum speed threshold
    int min_speed = 50;
    if (distspeed > 0 && distspeed < min_speed) distspeed = min_speed;
    if (distspeed < 0 && distspeed > -min_speed) distspeed = -min_speed;

    // PID-based correction using ToF sensors
    int errortof = leftDistance - rightDistance;
    float correction = computePID(errortof, tofkp, tofkd);

    // Calculate final speeds
    int leftSpeed = distspeed - correction;
    int rightSpeed = distspeed + correction;

    // Apply speed limits
    setMotorSpeed(leftSpeed, rightSpeed);

    // Debugging output
    Serial.print("Left: "); Serial.print(leftDistance);
    Serial.print(" cm | Right: "); Serial.print(rightDistance);
    Serial.print(" cm | Center: "); Serial.print(centerDistance);
    Serial.print(" cm | Correction: "); Serial.println(correction);

    // Stop condition when close to target
    if (abs(errordist) < 2) {
        Serial.println("Target distance reached. Stopping.");
        setMotorSpeed(0, 0);
    }
}


void loop() {
    moveForward(25);
    delay(50);
}
