#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Motor Control Pins
#define RIGHT_MOTOR_PWM PA1
#define LEFT_MOTOR_PWM PA6
#define RIGHT_MOTOR_IN1 PA2
#define RIGHT_MOTOR_IN2 PA3
#define LEFT_MOTOR_IN1 PA4
#define LEFT_MOTOR_IN2 PA5

// Encoder Pins
#define RIGHT_ENC_B PB3    
#define RIGHT_ENC_A PA15   
#define LEFT_ENC_A PB9     
#define LEFT_ENC_B PB8     

// Robot Parameters
#define COUNTS_PER_REVOLUTION 415
#define WHEEL_DIAMETER_CM 4.4
#define MIN_MOTOR_SPEED 30      
#define MAX_MOTOR_SPEED 180     
#define STOP_THRESHOLD 355
#define SLOWDOWN_FACTOR 0.7   

// PID Constants
const double KP_DIST_LEFT = 0.082, KD_DIST_LEFT = 0.3;
const double KP_DIST_RIGHT = 0.082, KD_DIST_RIGHT = 0.3;
const double KP_YAW = 2, KI_YAW = 0.0, KD_YAW = 0.5;

// Global Variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
MPU6050 mpu(Wire);
double targetYaw = 0.0;
bool flag=true;
int lspeed=0,rspeed=0;
// Encoder ISRs
void leftEncoderISR() {
    static unsigned long lastInterrupt = 0;
    unsigned long now = millis();
    if (now - lastInterrupt > 2) {
        leftEncoderCount += (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) ? 1 : -1;
        lastInterrupt = now;
    }
}

void rightEncoderISR() {
    static unsigned long lastInterrupt = 0;
    unsigned long now = millis();
    if (now - lastInterrupt > 2) {
        rightEncoderCount += (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) ? 1 : -1;
        lastInterrupt = now;
    }
}

void updateDisplay(const char* status) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("Status: ");
    display.println(status);
    display.print("Yaw: ");
    display.print(mpu.getAngleZ(), 1);
    display.print("\nL: ");
    display.print(leftEncoderCount);
    display.print(" R: ");
    display.println(rightEncoderCount);
    display.print("l: ");
    display.print(lspeed);
    display.print(" ");
    display.println(rspeed);
    display.display();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (abs(leftSpeed) < 15) leftSpeed = 0;
    if (abs(rightSpeed) < 15) rightSpeed = 0;
    
    leftSpeed = constrain(leftSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    digitalWrite(LEFT_MOTOR_IN1, leftSpeed > 0);
    digitalWrite(LEFT_MOTOR_IN2, leftSpeed < 0);
    digitalWrite(RIGHT_MOTOR_IN1, rightSpeed > 0);
    digitalWrite(RIGHT_MOTOR_IN2, rightSpeed < 0);
    
    analogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));
    analogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
}

void brakeMotors() {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_PWM, 0);
    analogWrite(RIGHT_MOTOR_PWM, 0);
}

void setup() {
    delay(3000);
    Serial.begin(115200);
    display.setRotation(2);
    Wire.begin();
    mpu.begin();
    mpu.calcOffsets(true);
    pinMode(PB12, OUTPUT);
    digitalWrite(PB12, HIGH);
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Display failed! Restarting...");
        delay(5000);
        NVIC_SystemReset();
    }
    display.clearDisplay();
    display.display();
    
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
    
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
    
    updateDisplay("System Ready");
}

void moveDistancePID(float distanceCm) {
    double targetCounts = (distanceCm / (3.14 * WHEEL_DIAMETER_CM)) * COUNTS_PER_REVOLUTION;
    double slowdownStart = targetCounts * SLOWDOWN_FACTOR;
    bool inSlowdownZone = false;
    long prevErrorLeft = 0, prevErrorRight = 0;
    
    updateDisplay("Moving...");
    
    while (true) {
        mpu.update();
        double errorLeft = targetCounts - leftEncoderCount;
        double errorRight = targetCounts - rightEncoderCount;
        double yawCorrection = KP_YAW * (targetYaw - mpu.getAngleZ());
        
        // if (!inSlowdownZone && (abs(errorLeft) < slowdownStart || abs(errorRight) < slowdownStart)) {
        //     inSlowdownZone = true;
        //     updateDisplay("fuck");
            
        // }
        // if(flag){
        //       flag=!flag;
        //       updateDisplay("Slowing downT");
        //     }else{
        //       flag=!flag;
        //       updateDisplay("Slowing downF");

        //     }
        
        double speedFactor = constrain(max(abs(errorLeft), abs(errorRight)) / slowdownStart, 0.4, 1.0);
        int leftSpeed = constrain(KP_DIST_LEFT * errorLeft + KD_DIST_LEFT * (errorLeft - prevErrorLeft), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) * speedFactor - yawCorrection;
        int rightSpeed = constrain(KP_DIST_RIGHT * errorRight + KD_DIST_RIGHT * (errorRight - prevErrorRight), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) * speedFactor + yawCorrection;
        lspeed=errorLeft;
        rspeed=errorRight;
        setMotorSpeeds(leftSpeed, rightSpeed);
        // if((abs(leftSpeed) + abs(rightSpeed))< 35){
          updateDisplay("notStopped");
         if ((abs(errorLeft) +abs(errorRight)) < (2*STOP_THRESHOLD)) {
          updateDisplay("Stopped");
        //     delay(50);
        //     mpu.update();
            //if (abs(targetCounts - leftEncoderCount) < STOP_THRESHOLD z abs(targetCounts - rightEncoderCount) < STOP_THRESHOLD) {
                brakeMotors();
                // updateDisplay("Stopped");
                break;
            //}
         }
        // updateDisplay("Not Stopped");
        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;
        delay(5);
    }
}

void loop() {

    leftEncoderCount = 0;
    rightEncoderCount = 0;
    targetYaw = mpu.getAngleZ();
    updateDisplay("Starting move");
    moveDistancePID(26.0);
    updateDisplay("Waiting...");
    delay(3000);
}