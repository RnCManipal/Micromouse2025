#include "Sensors.h"

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
MPU6050 mpu(Wire);
Adafruit_VL6180X tofLeft;
Adafruit_VL6180X tofCenter;
Adafruit_VL6180X tofRight;

void leftEncoderISR() {
    leftEncoderCount += (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? -1 : 1;
}

void rightEncoderISR() {
    rightEncoderCount += (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? -1 : 1;
}
#include <Adafruit_VL6180X.h>

int getDistance(Adafruit_VL6180X &sensor) {
    uint8_t distance = sensor.readRange();
    uint8_t status = sensor.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
        return distance;  // distance is already in cm
    } else {
        return -1;  // error or out of range
    }
}

bool hasBothSideWalls(int leftDist, int rightDist) {
    // Consider walls present if they're between 5cm and 30cm away
    return (leftDist >= 4 && leftDist <= 8) && (rightDist >= 4 && rightDist <= 8);
}
