#include <Wire.h>
#include "Adafruit_VL6180X.h"

// Sensor instances
Adafruit_VL6180X sensorLeft = Adafruit_VL6180X();
Adafruit_VL6180X sensorCenter = Adafruit_VL6180X();
Adafruit_VL6180X sensorRight = Adafruit_VL6180X();

#define XSHUT_LEFT   PB13  // Will set to 0x30
#define XSHUT_CENTER PA9   // Will set to 0x31
#define XSHUT_RIGHT  PB15  // Default address 0x29

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(100);

  // Set XSHUT as output
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_CENTER, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Turn off all sensors
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_CENTER, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // 1. Start Left sensor and set new address (0x30)
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensorLeft.begin()) {
    Serial.println("Failed to init LEFT sensor");
    while (1);
  }
  sensorLeft.setAddress(0x30);

  // 2. Start Center sensor and set new address (0x31)
  digitalWrite(XSHUT_CENTER, HIGH);
  delay(10);
  if (!sensorCenter.begin()) {
    Serial.println("Failed to init CENTER sensor");
    while (1);
  }
  sensorCenter.setAddress(0x31);

  // 3. Start Right sensor (default address 0x29)
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensorRight.begin()) {
    Serial.println("Failed to init RIGHT sensor");
    while (1);
  }

  Serial.println("All VL6180X sensors initialized.");
}

void loop() {
  uint8_t rangeL = sensorLeft.readRange();
  uint8_t rangeC = sensorCenter.readRange();
  uint8_t rangeR = sensorRight.readRange();

  Serial.print("Left (0x30): ");
  Serial.print(rangeL);
  Serial.print(" mm\t");

  Serial.print("Center (0x31): ");
  Serial.print(rangeC);
  Serial.print(" mm\t");

  Serial.print("Right (0x29): ");
  Serial.print(rangeR);
  Serial.println(" mm");

  delay(250);
}

