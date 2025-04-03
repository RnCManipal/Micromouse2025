#ifndef CONFIG_H
#define CONFIG_H

#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Motor Pins
#define M2_PWM PA1  // Right Motor PWM
#define M1_PWM PA6  // Left Motor PWM
#define M2_in1 PA2  // Right Motor Direction 1
#define M2_in2 PA3  // Right Motor Direction 2
#define M1_in1 PA4  // Left Motor Direction 1
#define M1_in2 PA5  // Left Motor Direction 2

// Encoder Pins
#define M2_ENC_B PB3  // Right Encoder A
#define M2_ENC_A PA15 // Right Encoder B
#define M1_ENC_A PB9  // Left Encoder A
#define M1_ENC_B PB8  // Left Encoder B

// TOF Sensor Pins
#define TOF_LEFT_XSHUT PB15  
#define TOF_CENTER_XSHUT PA9  
#define TOF_RIGHT_XSHUT PB13  

// Constants
#define MAXSPEED 180
#define MIN_OBSTACLE_DISTANCE 5  // Stop if obstacle is closer than 10 cm
#define COUNTS_PER_ROTATION 415
#define WHEEL_DIAMETER 4.4
#define DISTANCE_PER_TICK (PI * WHEEL_DIAMETER) / COUNTS_PER_ROTATION

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
extern Adafruit_SSD1306 display;

// Global Variables
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;
extern MPU6050 mpu;
extern Adafruit_VL53L0X tofLeft;
extern Adafruit_VL53L0X tofCenter;
extern Adafruit_VL53L0X tofRight;

// PID Parameters
// TOF-based wall following PID
extern float tofkp, tofkd;
extern float prevTofError;

// Distance-based PID
extern float distkp, distkd;
extern float prevDistError;

// MPU-based tilt correction PID
extern double kpT, kiT, kdT;
extern double targetAngle;
extern double tilt_error, prev_tilt_error, integral_tilt;

// Wheel-specific PID
extern double kpL, kdL;   // Left Wheel PID
extern double kpR, kdR;   // Right Wheel PID

#endif // CONFIG_H
