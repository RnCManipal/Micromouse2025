#include "config.h"
#include "Sensors.h"
#include "movement.h"
#include "floodfill.h"
void setup() {
    delay(5000);  // Initial delay (if needed)
    Serial.begin(115200);
    Wire.begin();
    display.setRotation(2);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Display failed! Restarting...");
        delay(5000);
        NVIC_SystemReset();
    }
    display.clearDisplay();
    display.display();
    
    // === ToF Sensor Initialization ===
    pinMode(TOF_LEFT_XSHUT, OUTPUT);
    pinMode(TOF_CENTER_XSHUT, OUTPUT);
    pinMode(TOF_RIGHT_XSHUT, OUTPUT);
    digitalWrite(TOF_LEFT_XSHUT, LOW);
    digitalWrite(TOF_CENTER_XSHUT, LOW);
    digitalWrite(TOF_RIGHT_XSHUT, LOW);
    delay(10);

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
    mpu.begin();
    delay(500);
    mpu.calcOffsets(true);  // Auto-calibrate
    Wire.setClock(400000);  // Increase MPU6050 read speed

    // === Motor Control Initialization ===
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M1_in1, OUTPUT);
    pinMode(M1_in2, OUTPUT);
    pinMode(M2_in1, OUTPUT);
    pinMode(M2_in2, OUTPUT);

    // Enable Motor Driver
    pinMode(PB12, OUTPUT);
    digitalWrite(PB12, HIGH);

    // === Encoder Interrupts ===
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), rightEncoderISR, CHANGE);
    updateDisplay("System Ready");

    delay(1000); // Final stabilization delay
}


void loop() {
  floodfill();
    // moveForward(25);
    // moveForward(25);
    // delay(1000);
    // Turn180();
    // delay(1000);
}
