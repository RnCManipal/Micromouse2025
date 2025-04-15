#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

extern Adafruit_VL6180X tofLeft;
extern Adafruit_VL6180X tofCenter;
extern Adafruit_VL6180X tofRight;
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;
extern MPU6050 mpu;

void leftEncoderISR();
void rightEncoderISR();
int getDistance(Adafruit_VL6180X &sensor);
bool hasBothSideWalls(int leftDist, int rightDist);

#endif // SENSORS_H