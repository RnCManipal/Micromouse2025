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

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PB12, OUTPUT);
  digitalWrite(PB12, HIGH);
  // Motor Pins
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M1_in1, OUTPUT);
  pinMode(M1_in2, OUTPUT);
  pinMode(M2_in1, OUTPUT);
  pinMode(M2_in2, OUTPUT);

  // Encoder Pins
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT_PULLUP);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), rightEncoderISR, CHANGE);

  // Set motor direction forward
  digitalWrite(M1_in1, HIGH);
  digitalWrite(M1_in2, LOW);
  digitalWrite(M2_in1, HIGH);
  digitalWrite(M2_in2, LOW);

  // Set PWM speed (range 0-255)
  analogWrite(M1_PWM, 100);
  analogWrite(M2_PWM, 100);
}

void loop() {
  Serial.print("Left Encoder: ");
  Serial.print(leftEncoderCount);
  Serial.print(" | Right Encoder: ");
  Serial.println(rightEncoderCount);

  delay(100);
}

// Encoder ISR for left motor
void leftEncoderISR() {
  if (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) {
    leftEncoderCount--;
  } else {
    leftEncoderCount++;
  }
}

// Encoder ISR for right motor
void rightEncoderISR() {
  if (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) {
    rightEncoderCount--;
  } else {
    rightEncoderCount++;
  }
}
