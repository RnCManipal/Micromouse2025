#include "Wire.h"
#include <MPU6050_light.h>

#define M1_PWM PA1  // Left Motor PWM
#define M2_PWM PA6  // Right Motor PWM
#define M1_in1 PA2  // Left Motor Direction 1
#define M1_in2 PA3  // Left Motor Direction 2
#define M2_in1 PA4  // Right Motor Direction 1
#define M2_in2 PA5  // Right Motor Direction 2

MPU6050 mpu(Wire);

// PID Constants
float Kp = 5.0;
float Kd = 0.5;
float previousError = 0;

// PWM Scaling for 5V logic operation
const float maxVoltage = 7.4;  // Change based on battery voltage
const float targetVoltage = 5.0;
float pwmScale = targetVoltage / maxVoltage;  // Scale factor

void Motor_SetSpeed(int spdL, int spdR) {
    spdL = constrain(spdL * pwmScale, -255, 255);
    spdR = constrain(spdR * pwmScale, -255, 255);

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

void brake() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
}

// PID-based rotation function
void rotateInPlace(float targetAngleDegrees, int maxSpeed) {
    mpu.update();
    float initialYaw = mpu.getAngleZ();
    float targetYaw = initialYaw + targetAngleDegrees;
    int flag=1;
    while (true) {
        mpu.update();
        float currentYaw = mpu.getAngleZ();
        float error = targetYaw - currentYaw;
        

        // PID calculations
        float derivative = error - previousError;
        float output = (Kp * error) + (Kd * derivative);
        previousError = error;

        Serial.print("Current angle: ");
        Serial.println(currentYaw);
        Serial.print("Current error: ");
        Serial.println(error);

        int speed = constrain(abs(output), 50, maxSpeed * pwmScale);

        // Slow down before reaching the target
        if (abs(error) < 10) {
            speed /= 2;
        }
    
        int direction = (error > 0) ? -1 : 1;
        Motor_SetSpeed(-direction * speed, direction * speed);
        
        // if (abs(error) < 1) {
        //     previousError = 0;
        //     break;
        // }
    }

    brake();
}

void setup() {
    delay(5000);
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    mpu.begin();
    delay(500);
    mpu.calcOffsets(true);

    // Increase MPU6050 read speed
    Wire.setClock(400000);

    // Enable Motor Driver
    pinMode(PB12, OUTPUT);
    digitalWrite(PB12, HIGH);

    // Motor Control Pins
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M1_in1, OUTPUT);
    pinMode(M1_in2, OUTPUT);
    pinMode(M2_in1, OUTPUT);
    pinMode(M2_in2, OUTPUT);
}

void loop() {
    rotateInPlace(90.0, 150); // Rotate 85 degrees at max speed 150
    delay(5000);
}