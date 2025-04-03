#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "config.h"
#include "Sensors.h"

#define COUNTS_PER_REVOLUTION 415
#define WHEEL_DIAMETER_CM 4.4
#define MIN_MOTOR_SPEED 30      
#define MAX_MOTOR_SPEED 180     
#define STOP_THRESHOLD 355
#define SLOWDOWN_FACTOR 0.7

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
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void brakeMotors();
void updateDisplay(const char* status);
#endif // MOVEMENT_H