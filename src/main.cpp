#include <Arduino.h>
#include <Pixy2.h>
#include <Servo.h>
#include <Wire.h>

constexpr uint8_t I2C_SLAVE_ADDR = 0x08;

#define ENC_LEFT_A 4
#define ENC_LEFT_B 5
#define ENC_RIGHT_A 3
#define ENC_RIGHT_B 2

#define PWM1 8
#define IN1 6
#define IN2 7
#define PWM2 18
#define IN3 20
#define IN4 19

#define LED  22
#define ECHO  15
#define TRIG  14
#define SERVO_PIN  23

#define CENTRE_ANGLE 135
#define MAX_RIGHT  CENTRE_ANGLE + 45
#define MAX_LEFT  CENTRE_ANGLE - 45

Servo steer_servo;
Pixy2 pixy;

volatile long leftCount = 0;
volatile long rightCount = 0;

volatile int i2c_speed_left = 0;
volatile int i2c_speed_right = 0;
volatile int i2c_steering_angle = CENTRE_ANGLE;
volatile bool i2c_new_command = false;

void leftEncoderISR() {
  if (digitalReadFast(ENC_LEFT_B) == digitalReadFast(ENC_LEFT_A)) {
    leftCount++;
  } else {
    leftCount--;
  }
}

void rightEncoderISR() {
  if (digitalReadFast(ENC_RIGHT_B) == digitalReadFast(ENC_RIGHT_A)) {
    rightCount++;
  } else {
    rightCount--;
  }
}

void initPixy() {
  pixy.init();
  pixy.changeProg("line");
}

void steer(int angle) {
  angle = constrain(angle, MAX_LEFT, MAX_RIGHT);
  steer_servo.write(angle);
}

void run(int speedLeft, int speedRight) {
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  if (speedRight >= 0) {
    analogWrite(PWM2, speedRight);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(PWM2, -speedRight);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  if (speedLeft >= 0) {
    analogWrite(PWM1, speedLeft);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    analogWrite(PWM1, -speedLeft);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

void onReceive(int howMany) {
  if (howMany < 3) {
    while (Wire1.available() > 0) {
      Wire1.read();
    }
    return;
  }

  const char cmd = static_cast<char>(Wire1.read());
  const uint8_t val1 = Wire1.read();
  const uint8_t val2 = Wire1.read();

  if (cmd == 's') {
    i2c_speed_left = map(val1, 0, 255, -255, 255);
    i2c_speed_right = map(val2, 0, 255, -255, 255);
    i2c_new_command = true;
  } else if (cmd == 't') {
    i2c_steering_angle = map(val1, 0, 255, MAX_LEFT, MAX_RIGHT);
    i2c_new_command = true;
  }

  while (Wire1.available() > 0) {
    Wire1.read();
  }
}

void communication_setup() {
  Wire1.begin(I2C_SLAVE_ADDR);
  Wire1.onReceive(onReceive);
}

void communication_loop() {
  if (!i2c_new_command) {
    return;
  }

  Serial.printf("Received:\n Speed Left: %d Right: %d Angle: %d\n",
                i2c_speed_left,
                i2c_speed_right,
                i2c_steering_angle);
  run(i2c_speed_left, i2c_speed_right);
  steer(i2c_steering_angle);
  i2c_new_command = false;
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  analogWriteResolution(8);
  analogWriteFrequency(PWM1, 20000);
  analogWriteFrequency(PWM2, 20000);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LED, OUTPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

  communication_setup();

  steer_servo.attach(SERVO_PIN);
  steer(CENTRE_ANGLE);

  initPixy();

  digitalWrite(LED, HIGH);
  
  run(0, 0);
}

void loop() {
  // communication_loop();
  pixy.line.getAllFeatures();
  for(uint8_t i = 0; i < pixy.line.numVectors; i++) {
    Serial.printf("Vector %d: x0=%d y0=%d x1=%d y1=%d\n", i, pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0, pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);
  }
  
  delay(500);
}