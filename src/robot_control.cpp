#include <Arduino.h>
#include <Pixy2.h>
#include <Servo.h>

#include "robot_control.h"

namespace {

constexpr uint8_t ENC_LEFT_A = 4;
constexpr uint8_t ENC_LEFT_B = 5;
constexpr uint8_t ENC_RIGHT_A = 3;
constexpr uint8_t ENC_RIGHT_B = 2;

constexpr uint8_t PWM1 = 8;
constexpr uint8_t IN1 = 6;
constexpr uint8_t IN2 = 7;
constexpr uint8_t PWM2 = 18;
constexpr uint8_t IN3 = 20;
constexpr uint8_t IN4 = 19;

constexpr uint8_t LED = 22;

constexpr uint8_t ECHO = 15;
constexpr uint8_t TRIG = 14;

constexpr uint8_t SERVO_PIN = 23;

Servo steer_servo;
Pixy2 pixy;

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

}  // namespace

volatile long leftCount = 0;
volatile long rightCount = 0;

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

void robot_control_setup() {
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

  steer_servo.attach(SERVO_PIN);
  steer(CENTRE_ANGLE);

  initPixy();

  digitalWrite(LED, HIGH);
  run(0, 0);
}
