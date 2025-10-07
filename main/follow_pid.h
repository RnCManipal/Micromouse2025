#ifndef FOLLOW_PID_H_
#define FOLLOW_PID_H_

#include "movement.h"
#include "config.h"
#include "Sensors.h"
#include "floodfill.h"

#define FRONT_THRESHOLD 125
#define LEFT_THRESHOLD  200
#define RIGHT_THRESHOLD 200
#define base_speed 200

// PID constants for wall centering
#define KP_wf 1.1    // Proportional gain
#define KD_wf 5   // Derivative gain
#define WALL_TARGET_DISTANCE 40  // Target distance from left wall (mm)

void delaying(int cm){
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    float target_wf= (cm / (3.14 * WHEEL_DIAMETER_CM)) * COUNTS_PER_ROTATION ;
    while(true){
    if (leftEncoderCount<= target_wf+15 && leftEncoderCount>= target_wf-15 && rightEncoderCount<= target_wf+15 && rightEncoderCount>= target_wf-15 ){
        return;
    }
}
}

int previous_error = 0;
void movePID();
int cellbycell=25;
void leftfollow_PID() {
    Serial.println("Left Wall Follower with PID started");

    while (true) {
        int left_wall  = getDistance(tofLeft);
        int right_wall = getDistance(tofRight);
        int front_wall = getDistance(tofCenter);

        Serial.print("Left: "); Serial.print(left_wall);
        Serial.print(" Right: "); Serial.print(right_wall);
        Serial.print(" Front: "); Serial.println(front_wall);

        // Check if left wall is detected for PID control
        bool left_wall_detected = (left_wall <= 250 && left_wall != -1);
        bool right_wall_detected = (right_wall <= 250 && right_wall != -1);
        bool front_wall_detected = (front_wall <= 200 && front_wall != -1);

        if (!left_wall_detected) {
            // No left wall detected, stop the robot
            Serial.println("No left wall detected. Stopping.");
          
            //delay(110);
            brakeMotors();
            TurnLeft();
            //rotateInPlace(90,255);
            moveForward(cellbycell,KP_DIST_LEFT, KD_DIST_LEFT ,KP_DIST_RIGHT ,KD_DIST_RIGHT);
        }

        else if (left_wall_detected && !front_wall_detected){
            //movePID();
             moveForward(cellbycell,KP_DIST_LEFT, KD_DIST_LEFT ,KP_DIST_RIGHT ,KD_DIST_RIGHT);
        }
        else if(right_wall_detected && front_wall_detected){
            Turn180();            
        }
        else if(front_wall_detected){
            Serial.println("Front wall detected. Stopping and turning right.");
            //delaying(10);
            // delay(110);
            brakeMotors();
            TurnRight();
            //rotateInPlace(-90,255);
        }

    }

}



float targetYaw_wf = 0;        // Desired heading (set when function starts)
float previous_error_wf = 0;   // For derivative
float KP_yaw = 2.0;         // Tune this
float KD_yaw = 0.5;         // Tune this

void movePID() {
    // Lock current yaw as target
    targetYaw_wf = readYaw();  

    while (true) {
        // --- Front wall stop condition ---
        int front_wall = getDistance(tofCenter);
        int left_wall= getDistance(tofLeft);

        if (front_wall > 0 && front_wall < FRONT_THRESHOLD) {
            //delay(110);
            //delaying(10);
            brakeMotors();
            return;
        }
        if (left_wall> LEFT_THRESHOLD || left_wall==-1){
            //delay(110);
            //delaying(10);
            brakeMotors();
            return;
        }

        // --- Yaw PID ---
        float currentYaw_wf = readYaw();
        float error_wf = wrapAngle(targetYaw_wf - currentYaw_wf); // keep in [-180, 180]
        float derivative_wf = error_wf - previous_error_wf;
        float correction_wf = (KP_yaw * error_wf) + (KD_yaw * derivative_wf);

        // --- Motor control ---
        int left_speed  = constrain(base_speed - correction_wf, 0, 255);
        int right_speed = constrain(base_speed + correction_wf, 0, 255);

        digitalWrite(M1_in1, HIGH);
        digitalWrite(M1_in2, LOW);
        analogWrite(M1_PWM, left_speed);

        digitalWrite(M2_in1, HIGH);
        digitalWrite(M2_in2, LOW);
        analogWrite(M2_PWM, right_speed);

        // Debug
        Serial.print("leftwall: "); Serial.print(left_wall);
        Serial.print(" frontwall: "); Serial.print(front_wall);
        Serial.print(" Error: "); Serial.print(error_wf);
        Serial.print(" Correction: "); Serial.println(correction_wf);

        previous_error_wf = error_wf;
        delay(10); // small loop delay
    }
}




#endif