#include "config.h"
#include "Sensors.h"
#include "movement.h"
#include "floodfill.h"
#include <Adafruit_BNO08x.h>




#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif


void setup() {
    delay(2000);  // Initial delay (if needed)
    Serial.begin(115200);
    Wire.begin();
   
    if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
      Serial.println("Failed to find BNO08x chip");
      while (1) { delay(10); }
    }
    Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
    // === ToF Sensor Initialization ===
    pinMode(TOF_LEFT_XSHUT, OUTPUT);
    pinMode(TOF_CENTER_XSHUT, OUTPUT);
    pinMode(TOF_RIGHT_XSHUT, OUTPUT);
    digitalWrite(TOF_LEFT_XSHUT, LOW);
    digitalWrite(TOF_CENTER_XSHUT, LOW);
    digitalWrite(TOF_RIGHT_XSHUT, LOW);
    delay(10);

    pinMode(SWITCH, INPUT);
    // Initialize left ToF
    digitalWrite(TOF_LEFT_XSHUT, HIGH);
    delay(50);
    if (!tofLeft.begin()) {
        Serial.println("Failed to initialize Left ToF!");
    }
    tofLeft.setAddress(0x30);

    // Initialize center ToF
    digitalWrite(TOF_CENTER_XSHUT, HIGH);
    delay(50);
    if (!tofCenter.begin()) {
        Serial.println("Failed to initialize Center ToF!");
    }
    tofCenter.setAddress(0x31);

    // Initialize right ToF
    digitalWrite(TOF_RIGHT_XSHUT, HIGH);
    delay(50);
    if (!tofRight.begin()) {
        Serial.println("Failed to initialize Right ToF!");
    }
    tofRight.setAddress(0x32);

    // === MPU6050 Initialization ===
    // mpu.begin();
    // delay(500);
    // mpu.calcOffsets(true);  // Auto-calibrate
    // Wire.setClock(400000);  // Increase MPU6050 read speed

    // === Motor Control Initialization ===
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M1_in1, OUTPUT);
    pinMode(M1_in2, OUTPUT);
    pinMode(M2_in1, OUTPUT);
    pinMode(M2_in2, OUTPUT);

    pinMode(PUSH1,INPUT_PULLUP);
    pinMode(PUSH2,INPUT_PULLUP);
    // Enable Motor Driver
    pinMode(PB12, OUTPUT);
    digitalWrite(PB12, HIGH);

    // === Encoder Interrupts ===
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), rightEncoderISR, CHANGE);
    updateDisplay("System Ready");

    delay(2000); // Final stabilization delay

}

int a  = 0;

void loop() {

  

  int press= waitForPress();

  if(a == 0){
    setFixedAngles();
    a++;
    delay(3000);
  }
  switch (press) {
      case 1:   
        floodfill();
        break;

      case 2:   
        leftWallFollowerLoop();
        break;
    }
  // Serial.println(readYaw());
  // TurnRight();
  // delay(500);
  // TurnLeft();
  // delay(500);
  // Turn180();
  // delay(500);
  }
 

