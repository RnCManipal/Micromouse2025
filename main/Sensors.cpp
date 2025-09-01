#include "Sensors.h"

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
MPU6050 mpu(Wire);
Adafruit_VL6180X tofLeft;
Adafruit_VL6180X tofCenter;
Adafruit_VL6180X tofRight;

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

static euler_t ypr = {0.0f, 0.0f, 0.0f};

// Helper functions for quaternions
static void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr), sqi = sq(qi), sqj = sq(qj), sqk = sq(qk);
    ypr->yaw   = atan2(2.0f*(qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0f*(qi*qk - qj*qr) / (sqi + sqi + sqj + sqk));
    ypr->roll  = atan2(2.0f*(qj*qk + qi*qr), (-sqi - sqj + sqk + sqr));
    if (degrees) {
        ypr->yaw   *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll  *= RAD_TO_DEG;
    }
}

static void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rv, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rv->real, rv->i, rv->j, rv->k, ypr, degrees);
}

static void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rv, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rv->real, rv->i, rv->j, rv->k, ypr, degrees);
}

// Set reports
void setReports(sh2_SensorId_t reportType, long report_interval) {
    if (!bno08x.enableReport(reportType, report_interval)) {
        Serial.println("Could not enable BNO report");
    }
}

// Read events and update Euler angles
void processBNOEvents() {
    if (bno08x.wasReset()) {
        Serial.println("BNO sensor reset!");
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ARVR_STABILIZED_RV:
                quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
                break;
            case SH2_GYRO_INTEGRATED_RV:
                quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
                break;
        }
    }
}

// Return the last Euler angles
euler_t getEuler() {
    return ypr;
}

float readYaw() {
    // Process new BNO events
    if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ARVR_STABILIZED_RV:
                quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
                break;
            case SH2_GYRO_INTEGRATED_RV:
                quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
                break;
        }
    }
    return ypr.yaw;  // Return latest yaw
}

void leftEncoderISR() {
    leftEncoderCount += (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? -1 : 1;
}

void rightEncoderISR() {
    rightEncoderCount += (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? -1 : 1;
}



int getDistance(Adafruit_VL6180X &sensor) {
    uint8_t distance = sensor.readRange();
    uint8_t status = sensor.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
        return distance;  // distance is already in cm
    } else {
        return -1;  // error or out of range
    }
}

bool hasBothSideWalls(int leftDist, int rightDist) {
    // Consider walls present if they're between 5cm and 30cm away
    return (leftDist >= 4 && leftDist <= 8) && (rightDist >= 4 && rightDist <= 8);
}

int waitForPress() {
  // Loop until either button is pressed
  while (true) {
    if (digitalRead(PUSH1) == LOW) {  // button1 pressed
      delay(50); // debounce
      while (digitalRead(PUSH1) == LOW); // wait release
      delay(1500);
      return 1;
      break;
    }
    if (digitalRead(PUSH2) == LOW) {  // button2 pressed
      delay(50); 
      while (digitalRead(PUSH2) == LOW); // wait release
      delay(1500);
      return 2;
      break;
      }
  }
}
