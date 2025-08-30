#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

extern Adafruit_VL6180X tofLeft;
extern Adafruit_VL6180X tofCenter;
extern Adafruit_VL6180X tofRight;
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;
extern MPU6050 mpu;



struct euler_t {
    float yaw;
    float pitch;
    float roll;
};

// BNO object and sensor value
extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;

// Functions
void setReports(sh2_SensorId_t reportType, long report_interval);
void processBNOEvents();          // Reads new events and updates ypr
euler_t getEuler();       
float readYaw();  // Returns updated yaw in degrees

void leftEncoderISR();
void rightEncoderISR();
int getDistance(Adafruit_VL6180X &sensor);
bool hasBothSideWalls(int leftDist, int rightDist);



#endif // SENSORS_H