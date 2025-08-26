#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "config.h"
#include "Sensors.h"

#define WHEEL_DIAMETER_CM 4.4
#define MIN_MOTOR_SPEED 30      
#define MAX_MOTOR_SPEED 180     
#define STOP_THRESHOLD 363 //355
#define SLOWDOWN_FACTOR 0.8

extern float tofkp, tofkd, prevTofError;
extern float distkp, distkd, prevDistError;
extern double kpT, kiT, kdT, targetAngle;
extern double tilt_error, prev_tilt_error, integral_tilt;
extern double kpL, kdL, kpR, kdR;
extern int currentDir;              // global variable
extern float initAngles[4];         // N, E, S, W yaw targets

float wrapAngle(float angle);   
void moveForward(int distanceCm, double KP_DIST_LEFT,double KD_DIST_LEFT, double KP_DIST_RIGHT,double KD_DIST_RIGHT);
void Motor_SetSpeed(int spdL, int spdR);

float computePID(int error, float kp, float kd);
void rotateInPlace(float targetAngleDegrees, int maxSpeed);
void rotateToFixed(float targetYaw, int maxSpeed);
void TurnLeft();
void TurnRight();
void Turn180();
void setFixedAngles();

void brake();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void brakeMotors();
void updateDisplay(const char* status);
#endif // MOVEMENT_H