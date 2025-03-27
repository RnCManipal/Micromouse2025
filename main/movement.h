#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "config.h"
#include "Sensors.h"

extern float tofkp, tofkd, prevTofError;
extern float distkp, distkd, prevDistError;
extern double kpT, kiT, kdT, targetAngle;
extern double tilt_error, prev_tilt_error, integral_tilt;
extern double kpL, kdL, kpR, kdR;

void moveForward(int targetdist);
void Motor_SetSpeed(int spdL, int spdR);
float computePID(int error, float kp, float kd);
void rotateInPlace(float targetAngleDegrees, int maxSpeed);
void TurnLeft();
void TurnRight();
void Turn180();
void brake();

#endif // MOVEMENT_H