#include "movement.h"

float prevTofError = 0;


float prevDistError = 0;

double kpT = 12 , kiT = 0.0, kdT = 0.5; //rotate in place PID constants
double targetAngle = 0.0;
double tilt_error = 0, prev_tilt_error = 0, integral_tilt = 0;

// PID Constants for move forward
const double KP_DIST_LEFT = 0.07, KD_DIST_LEFT = 0.03;
const double KP_DIST_RIGHT = 0.1, KD_DIST_RIGHT = 0.03;
const double KP_YAW = 0.3, KI_YAW = 0.0, KD_YAW = 0.35;
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

void moveForward(int distanceCm, double KP_DIST_LEFT ,double KD_DIST_LEFT, double KP_DIST_RIGHT,double KD_DIST_RIGHT) {
    mpu.update();
    targetYaw = mpu.getAngleZ();
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    double targetCounts = (distanceCm / (3.14 * WHEEL_DIAMETER_CM)) * COUNTS_PER_ROTATION;
    double slowdownStart = targetCounts * 0.95;
    bool inSlowdownZone = false;
    long prevErrorLeft = 0, prevErrorRight = 0;

    Serial.print("Target Counts=");
    Serial.println(targetCounts);

    // Wall following constants
    const double DESIRED_WALL_DIST = 80.0; // mm
    const double WALL_DETECT_THRESHOLD = 250.0; // mm
    const double WALL_FOLLOW_KP = 0.65; // tune this

    while (true) {
        mpu.update();
        left_dist = getDistance(tofLeft);
        right_dist = getDistance(tofRight);
        front_dist = getDistance(tofCenter);
        
        double errorLeft = targetCounts - leftEncoderCount;
        double errorRight = targetCounts - rightEncoderCount;

        // Base yaw correction (original functionality)
        double yawCorrection = KP_YAW * -(targetYaw - mpu.getAngleZ());
    
        // Wall-following adjustment — added on top of yaw correction
        bool leftWall = (left_dist < WALL_DETECT_THRESHOLD && left_dist>0);
        bool rightWall = (right_dist < WALL_DETECT_THRESHOLD && right_dist>0);
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
        yawCorrection += WALL_FOLLOW_KP * wallError ;
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

    if (abs(targetYaw - mpu.getAngleZ())>0){
        rotateInPlace(targetYaw - mpu.getAngleZ(), 20);
    }

    //if(front_dist < 155 && front_dist>90){
            
            //front_dist = getDistance(tofCenter);
            //moveForward(front_dist-85);
        //}

   for(front_dist = getDistance(tofCenter);front_dist < 200 && front_dist>100;front_dist = getDistance(tofCenter)){
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
    spdL = constrain(spdL * 2, -255, 255);
    spdR = constrain(spdR * 2, -255, 255);

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

float computePID(int error, float kp, float kd) {
    static float prevError = 0;
    float derivative = error - prevError;
    float correction = kp * error + kd * derivative;
    prevError = error;
    return correction;
}

void rotateInPlace(float targetAngleDegrees, int maxSpeed) {
    mpu.update();
    float initialYaw = mpu.getAngleZ();
    float targetYaw = initialYaw + targetAngleDegrees;

    while (true) {
        mpu.update();
        float currentYaw = mpu.getAngleZ();
        float error = targetYaw - currentYaw;

        float derivative = error - prev_tilt_error;
        float output = (kpT * error) + (kdT * derivative);
        prev_tilt_error = error;

        // Serial.print("Current angle: ");
        // Serial.println(currentYaw);
        // Serial.print("Current error: ");
        // Serial.println(error);

        int speed = constrain(abs(output), 20, maxSpeed * 1.48);

        //if (abs(error) < 10) {  
          //  speed /= 2;  // Reduce speed when the bot is close to the target angle
        // }

        if (abs(error) < 0.3 && abs(derivative)<0.06) {  // Ensure it's not moving fast
            break;
        }
        int direction = (error > 0) ? -1 : 1;
        Motor_SetSpeed(-direction * speed, direction * speed);
    }

    brake();
}

void brakeMotors() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}

void TurnLeft() {
    rotateInPlace(-90.0, 35);
}

void TurnRight() {
    rotateInPlace(90.0, 35);
}

void Turn180() {
    if(getDistance(tofLeft)>getDistance(tofRight)){
        rotateInPlace(-180.0, 35);
        }
    else{
        rotateInPlace(180,35);
    }
}

void brake() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
}