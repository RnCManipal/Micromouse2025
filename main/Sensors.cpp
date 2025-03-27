#include "Sensors.h"

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
MPU6050 mpu(Wire);
Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofCenter;
Adafruit_VL53L0X tofRight;

void leftEncoderISR() {
    leftEncoderCount += (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? 1 : -1;
}

void rightEncoderISR() {
    rightEncoderCount += (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? 1 : -1;
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
