#include "movement.h"

float tofkp = 2, tofkd = 5;
float prevTofError = 0;

float distkp = 1.0, distkd = 1.0;
float prevDistError = 0;

double kpT = 3.0, kiT = 0.0, kdT = 8;
double targetAngle = 0.0;
double tilt_error = 0, prev_tilt_error = 0, integral_tilt = 0;

double kpL = 0.09, kdL = 1.1;
double kpR = 0.11, kdR = 1.2;

// PID Constants
const double KP_DIST_LEFT = 0.082, KD_DIST_LEFT = 0.3;
const double KP_DIST_RIGHT = 0.082, KD_DIST_RIGHT = 0.3;
const double KP_YAW = 2, KI_YAW = 0.0, KD_YAW = 0.5;

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

void moveForward(int distanceCm) {
    mpu.update();
    targetYaw=mpu.getAngleZ();
    leftEncoderCount=0;
    rightEncoderCount=0;
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
        // Serial.print("Target Counts: "); Serial.println(targetCounts);
        // Serial.print("Encoder Left: "); Serial.print(leftEncoderCount);
        // Serial.print(" Encoder Right: "); Serial.println(rightEncoderCount);
        // Serial.print("Yaw Error: "); Serial.println(targetYaw - mpu.getAngleZ());

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

void Motor_SetSpeed(int spdL, int spdR) {
    spdL = constrain(spdL * 1.48, -255, 255);
    spdR = constrain(spdR * 1.48, -255, 255);

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

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (abs(leftSpeed) < 15) leftSpeed = 0;
    if (abs(rightSpeed) < 15) rightSpeed = 0;
    
    leftSpeed = constrain(leftSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    digitalWrite(M1_in1, leftSpeed > 0);
    digitalWrite(M1_in2, leftSpeed < 0);
    digitalWrite(M2_in1, rightSpeed > 0);
    digitalWrite(M2_in2, rightSpeed < 0);
    
    analogWrite(M1_PWM, abs(leftSpeed));
    analogWrite(M2_PWM, abs(rightSpeed));
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

        Serial.print("Current angle: ");
        Serial.println(currentYaw);
        Serial.print("Current error: ");
        Serial.println(error);

        int speed = constrain(abs(output), 20, maxSpeed * 1.48);

        if (abs(error) < 10) {  
            speed /= 2;  // Reduce speed when the bot is close to the target angle
        }

        if (abs(error) < 1 && abs(derivative) < 0.5) {  // Ensure it's not moving fast
            break;
        }
        int direction = (error > 0) ? -1 : 1;
        Motor_SetSpeed(direction * speed, -direction * speed);
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
    rotateInPlace(90.0, 20);
}

void TurnRight() {
    rotateInPlace(-90.0, 20);
}

void Turn180() {
    rotateInPlace(180.0, 20);
}

void brake() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
}
