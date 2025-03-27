#include "movement.h"

float tofkp = 2, tofkd = 5;
float prevTofError = 0;

float distkp = 1.0, distkd = 1.0;
float prevDistError = 0;

double kpT = 1, kiT = 0.0, kdT = 0.1;
double targetAngle = 0.0;
double tilt_error = 0, prev_tilt_error = 0, integral_tilt = 0;

double kpL = 0.09, kdL = 1.1;
double kpR = 0.11, kdR = 1.2;

void moveForward(int targetdist) {
    int leftDistance = getDistance(tofLeft);
    int rightDistance = getDistance(tofRight);
    int centerDistance = getDistance(tofCenter);

    if (leftDistance == -1) leftDistance = getDistance(tofLeft);
    if (rightDistance == -1) rightDistance = getDistance(tofRight);
    if (centerDistance == -1) centerDistance = getDistance(tofCenter);

    if (leftDistance == -1 || rightDistance == -1 || centerDistance == -1) {
        Serial.println("Sensor error! Stopping motors.");
        Motor_SetSpeed(0, 0);
        return;
    }

    if (centerDistance < MIN_OBSTACLE_DISTANCE) {
        Serial.println("Obstacle detected! Stopping.");
        Motor_SetSpeed(0, 0);
        return;
    }

    int currentdist = ((leftEncoderCount + rightEncoderCount) / 2) * DISTANCE_PER_TICK;
    int errordist = targetdist - currentdist;

    static float prevDistError = 0;
    float distspeed = distkp * errordist + distkd * (errordist - prevDistError);
    prevDistError = errordist;

    int min_speed = 50;
    if (distspeed > 0 && distspeed < min_speed) distspeed = min_speed;
    if (distspeed < 0 && distspeed > -min_speed) distspeed = -min_speed;

    int errortof = leftDistance - rightDistance;
    float correction = computePID(errortof, tofkp, tofkd);

    int leftSpeed = distspeed - correction;
    int rightSpeed = distspeed + correction;

    Motor_SetSpeed(leftSpeed, rightSpeed);

    Serial.print("Left: "); Serial.print(leftDistance);
    Serial.print(" cm | Right: "); Serial.print(rightDistance);
    Serial.print(" cm | Center: "); Serial.print(centerDistance);
    Serial.print(" cm | Correction: "); Serial.println(correction);

    if (abs(errordist) < 2) {
        Serial.println("Target distance reached. Stopping.");
        Motor_SetSpeed(0, 0);
    }
}

void Motor_SetSpeed(int spdL, int spdR) {
    spdL = constrain(spdL * 1.48, -255, 255);
    spdR = constrain(spdR * 1.48, -255, 255);

    if (spdL == 0) {
        digitalWrite(M1_in1, LOW);
        digitalWrite(M1_in2, LOW);
        analogWrite(M1_PWM, 0);
    } else if (spdL < 0) {
        digitalWrite(M1_in1, LOW);
        digitalWrite(M1_in2, HIGH);
        analogWrite(M1_PWM, abs(spdL));
    } else {
        digitalWrite(M1_in1, HIGH);
        digitalWrite(M1_in2, LOW);
        analogWrite(M1_PWM, spdL);
    }

    if (spdR == 0) {
        digitalWrite(M2_in1, LOW);
        digitalWrite(M2_in2, LOW);
        analogWrite(M2_PWM, 0);
    } else if (spdR < 0) {
        digitalWrite(M2_in1, LOW);
        digitalWrite(M2_in2, HIGH);
        analogWrite(M2_PWM, abs(spdR));
    } else {
        digitalWrite(M2_in1, HIGH);
        digitalWrite(M2_in2, LOW);
        analogWrite(M2_PWM, spdR);
    }
}

float computePID(int error, float kp, float kd) {
    static float prevError = 0;
    float derivative = error - prevError;
    float correction = kp * error + kd * derivative;
    prevError = error;
    return correction;
}

void rotateInPlace(float targetAngleDegrees, int maxSpeed) {
    mpu.update();
    float initialYaw = mpu.getAngleZ();
    float targetYaw = initialYaw + targetAngleDegrees;

    while (true) {
        mpu.update();
        float currentYaw = mpu.getAngleZ();
        float error = targetYaw - currentYaw;

        float derivative = error - prev_tilt_error;
        float output = (kpT * error) + (kdT * derivative);
        prev_tilt_error = error;

        Serial.print("Current angle: ");
        Serial.println(currentYaw);
        Serial.print("Current error: ");
        Serial.println(error);

        int speed = constrain(abs(output), 50, maxSpeed * 1.48);

        if (abs(error) < 10) {
            speed /= 2;
        }

        int direction = (error > 0) ? -1 : 1;
        Motor_SetSpeed(-direction * speed, direction * speed);
    }

    brake();
}

void TurnLeft() {
    rotateInPlace(90.0, 150);
}

void TurnRight() {
    rotateInPlace(-90.0, 150);
}

void Turn180() {
    rotateInPlace(180.0, 150);
}

void brake() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
}
