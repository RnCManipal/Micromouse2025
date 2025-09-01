#include "movement.h"
#include <math.h>

float prevTofError = 0;
float prevDistError = 0;

double kpT = 1 , kiT = 0.0, kdT = 0.7; //rotate in place PID constants
double targetAngle = 0.0;
double tilt_error = 0, prev_tilt_error = 0, integral_tilt = 0;

// PID Constants for move forward
// const double KP_DIST_LEFT = 0.10, KD_DIST_LEFT = 0.03;
// const double KP_DIST_RIGHT = 0.10, KD_DIST_RIGHT = 0.03;


double left_dist, right_dist, front_dist;
double targetYaw;
bool flag=true;
int lspeed=0,rspeed=0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void updateDisplay(const char* status) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("Status: ");
    display.println(status);
    display.print("Yaw: ");
    display.print(readYaw(), 1);
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

void moveForward(int distanceCm, double KP_DIST_LEFT ,double KD_DIST_LEFT, double KP_DIST_RIGHT,double KD_DIST_RIGHT,double WALL_FOLLOW_KP, double WALL_FOLLOW_KD,double KP_YAW) {
    //mpu.update();
    targetYaw = initAngles[currentDir];
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    double targetCounts = (distanceCm / (3.14 * WHEEL_DIAMETER_CM)) * COUNTS_PER_ROTATION;
    double slowdownStart = targetCounts * 0.95;
    bool inSlowdownZone = false;
    long prevErrorLeft = 0, prevErrorRight = 0;

    Serial.print("Target Counts=");
    Serial.println(targetCounts);

    // Wall following constants

    const double DESIRED_WALL_DIST = 65  ; // mm
    const double WALL_DETECT_THRESHOLD = 300.0; // mm
     //0.2


    double prevwallError = 0;
    
    while (true) {
        bool leftWall = (left_dist < WALL_DETECT_THRESHOLD && left_dist>0);
        bool rightWall = (right_dist < WALL_DETECT_THRESHOLD && right_dist>0);
        //mpu.update();
        left_dist = getDistance(tofLeft);
        right_dist = getDistance(tofRight);
        front_dist = getDistance(tofCenter);
        
        double errorLeft = targetCounts - leftEncoderCount;
        double errorRight = targetCounts - rightEncoderCount;

        // Base yaw correction (original functionality)
        double yawCorrection = KP_YAW * -(targetYaw - readYaw());
    
        // Wall-following adjustment — added on top of yaw correction
        
        double wallError = 0;
        

        if (leftWall && rightWall) {
            wallError = -(DESIRED_WALL_DIST - left_dist) + (DESIRED_WALL_DIST - right_dist);
        } 
        if (leftWall && !rightWall) {
            wallError = -(DESIRED_WALL_DIST - left_dist);
        } 
        if (!leftWall && rightWall) {
            wallError = -(DESIRED_WALL_DIST - right_dist);
        } 
        if(!leftWall && !rightWall){
            wallError = 0;
        }
        yawCorrection += WALL_FOLLOW_KP * wallError + WALL_FOLLOW_KD * (wallError - prevwallError ) ;
        prevwallError=wallError;
        if(front_dist < 100 && front_dist>0){
            brakeMotors();
            break;
            
        }
        
        int leftSpeed = constrain(KP_DIST_LEFT * errorLeft + KD_DIST_LEFT * (errorLeft - prevErrorLeft), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) - yawCorrection;
        int rightSpeed = constrain(KP_DIST_RIGHT * errorRight + KD_DIST_RIGHT * (errorRight - prevErrorRight), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED) + yawCorrection;


        lspeed = errorLeft;
        rspeed = errorRight;
        setMotorSpeeds(leftSpeed, rightSpeed);

        if ((abs(errorLeft) + abs(errorRight)) < (2 * STOP_THRESHOLD)) {
            brakeMotors();
            break;
        }

        prevErrorLeft = errorLeft;
        prevErrorRight = errorRight;
        delay(5);
    }
    float yawError = wrapAngle(targetYaw - readYaw());

    if (abs(targetYaw - readYaw())>3){
        rotateToFixed(initAngles[currentDir],18);
    }

    //if(front_dist < 155 && front_dist>90){
            
            //front_dist = getDistance(tofCenter);
            //moveForward(front_dist-85);
        //}

   for(front_dist = getDistance(tofCenter);front_dist < 200 && front_dist>80;front_dist = getDistance(tofCenter)){
      digitalWrite(M1_in1, HIGH);
    digitalWrite(M1_in2, LOW);
   analogWrite(M1_PWM, 120);
    digitalWrite(M2_in1, HIGH);
       digitalWrite(M2_in2, LOW);
     analogWrite(M2_PWM, 120);
   }
   digitalWrite(M1_in1, LOW);
   digitalWrite(M1_in2, LOW);
   digitalWrite(M2_in1, LOW);
   digitalWrite(M2_in2, LOW);
}


void Motor_SetSpeed(int spdL, int spdR) {
    spdL = constrain(spdL*2.1 , -255, 255);
    spdR = constrain(spdR*2.1, -255, 255);

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

// Define the minimum PWM value as a constant or variable
const int MIN_PWM = 110; // Use 'const' if this value won't change during runtime
// int minPwmValue = 60; // Use a regular 'int' if you need to change it

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    int actualLeftSpeed = leftSpeed;
    int actualRightSpeed = rightSpeed;

    // Enforce minimum PWM for non-zero speeds
    if (actualLeftSpeed > 0 && actualLeftSpeed < MIN_PWM) {
        actualLeftSpeed = MIN_PWM;
    } else if (actualLeftSpeed < 0 && actualLeftSpeed > -MIN_PWM) {
        actualLeftSpeed = -MIN_PWM;
    } else if (actualLeftSpeed == 0) {
        actualLeftSpeed = 0; // Ensure zero remains zero
    }

    if (actualRightSpeed > 0 && actualRightSpeed < MIN_PWM) {
        actualRightSpeed = MIN_PWM;
    } else if (actualRightSpeed < 0 && actualRightSpeed > -MIN_PWM) {
        actualRightSpeed = -MIN_PWM;
    } else if (actualRightSpeed == 0) {
        actualRightSpeed = 0; // Ensure zero remains zero
    }

    actualLeftSpeed = constrain(actualLeftSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    actualRightSpeed = constrain(actualRightSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    digitalWrite(M1_in1, actualLeftSpeed > 0);
    digitalWrite(M1_in2, actualLeftSpeed < 0);
    digitalWrite(M2_in1, actualRightSpeed > 0);
    digitalWrite(M2_in2, actualRightSpeed < 0);

    analogWrite(M1_PWM, abs(actualLeftSpeed));
    analogWrite(M2_PWM, abs(actualRightSpeed));
}

float yawOffset = 0.0;  // set during setup
// --- Wrap to [-180, 180] ---
float wrapAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

// --- Read yaw relative to zeroed start ---
float readRelativeYaw() {
    return wrapAngle(readYaw() - yawOffset);
}

float initAngles[4];   // 0=N, 1=E, 2=W, 3=S
float initial;
int currentDir = 0;    // start facing "forward" (index 0)


void setFixedAngles() {
    initial = readYaw();

    initAngles[0] = wrapAngle(initial);        // forward
    initAngles[1] = wrapAngle(initial - 90);   // right
    initAngles[2] = wrapAngle(initial + 180);  // back
    initAngles[3] = wrapAngle(initial + 90);   // left

    // Print the array
    Serial.println("Init Angles:");
    for (int i = 0; i < 4; i++) {
        Serial.print("initAngles[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.println(initAngles[i]);
    }
}


// --- Simple PID ---
float computePID(float error, float kp, float kd) {
    static float prevError = 0.0;
    float derivative = error - prevError;
    prevError = error;
    return kp * error + kd * derivative;
}

// --- Rotate relative to current orientation ---
void rotateInPlace(float relativeAngle, int maxSpeed) {
    float initialYaw = readRelativeYaw();            
    float targetYaw = wrapAngle(initialYaw + relativeAngle);

    float prevYaw = initialYaw;
    unsigned long prevTime = millis();

    while (true) {
        float currentYaw = readRelativeYaw();
        float error = wrapAngle(targetYaw - currentYaw);

        // --- compute angular velocity ---
        unsigned long now = millis();
        float dt = (now - prevTime) / 1000.0; // seconds
        float yawRate = wrapAngle(currentYaw - prevYaw) / dt;

        prevYaw = currentYaw;
        prevTime = now;
        // --------------------------------

        float output = computePID(error, kpT, kdT);
        int speed = constrain(abs(output), 20, maxSpeed);
        int direction = (error > 0) ? 1 : -1;

        Motor_SetSpeed(-direction * speed, direction * speed);
        // Serial.print("Error: "); Serial.print(error);
        // Serial.print("  YawRate: "); Serial.println(yawRate);

        // Stop condition: close enough AND slow enough
        if (abs(error) < 0.5 && abs(yawRate) < 75.0) { // 5°/s threshold
            break;
        }

        delay(10);
    }

    brakeMotors();
}



// Rotate to one of the predefined orientations
void rotateToFixed(float targetYaw, int maxSpeed) {
    float currentYaw = readYaw();
    float error = wrapAngle(targetYaw - currentYaw);
    rotateInPlace(error, maxSpeed);
}

void TurnLeft() {
    Serial.println("turning left");
    currentDir = (currentDir + 3) % 4;   // move left in index space
    rotateToFixed(initAngles[currentDir], 255);
}

void TurnRight() {
    Serial.println("turning right");
    currentDir = (currentDir + 1) % 4;   // move right
    rotateToFixed(initAngles[currentDir], 255);
}

void Turn180() {
    Serial.println("turning back");    
    currentDir = (currentDir + 2) % 4;   // flip
    rotateToFixed(initAngles[currentDir], 255);
}


void brakeMotors() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}



void brake() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
}
