#include <Arduino.h>
#include <Wire.h>
#include <Pixy2.h>
#include <Servo.h>

#define ENC_LEFT_A   4
#define ENC_LEFT_B   5
#define ENC_RIGHT_A  3
#define ENC_RIGHT_B  2

#define PWM1 8
#define IN1 7
#define IN2 6
#define PWM2 18
#define IN3 19
#define IN4 20

#define LED 22

#define ECHO 15
#define TRIG 14

#define SERVO_PIN 23

#define CENTRE_ANGLE 135
#define MAX_RIGHT CENTRE_ANGLE + 45
#define MAX_LEFT CENTRE_ANGLE - 45


Servo steer_servo;
Pixy2 pixy;

void initPixy(){
  pixy.init();
  pixy.changeProg("line");
}

void readMainVec(){
  pixy.line.getMainFeatures();
}

void readAllVecs(){
  pixy.line.getAllFeatures();
}

void printVecs(){
  for (int i = 0; i < pixy.line.numVectors; i++)
  {
    Serial.print("Vector ");
    Serial.print(i);
    Serial.print(": x0=");
    Serial.print(pixy.line.vectors[i].m_x0);
    Serial.print(", y0=");
    Serial.print(pixy.line.vectors[i].m_y0);
    Serial.print(", x1=");
    Serial.print(pixy.line.vectors[i].m_x1);
    Serial.print(", y1=");
    Serial.println(pixy.line.vectors[i].m_y1);
    Serial.println("------------------------------");
  }
  Serial.println("==============================");
}

void debugVision(){
  readAllVecs();
  //readMainVec();
  printVecs();
}

#define TICKS_PER_REV  360.0f
#define WHEEL_DIAMETER 0.065f     // meters

volatile long leftCount = 0;
volatile long rightCount = 0;

void leftEncoderISR() {
  if (digitalReadFast(ENC_LEFT_B) == digitalReadFast(ENC_LEFT_A))
    leftCount++;
  else
    leftCount--;
}

void rightEncoderISR() {
  if (digitalReadFast(ENC_RIGHT_B) == digitalReadFast(ENC_RIGHT_A))
    rightCount++;
  else
    rightCount--;
}

void run(int speedLeft, int speedRight)
{
  if (speedRight >= 0)
  {
    analogWrite(PWM2, speedRight);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    analogWrite(PWM2, -speedRight);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  
  if (speedLeft >= 0)
  {
    analogWrite(PWM1, speedLeft);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    analogWrite(PWM1, -speedLeft);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
}

void setup()
{
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

  initPixy();

  run(0,0);

  digitalWrite(LED, HIGH);
  steer_servo.write(MAX_LEFT);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  steer_servo.write(CENTRE_ANGLE);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  steer_servo.write(MAX_RIGHT);
  delay(500);
  digitalWrite(LED, LOW);

  Serial.begin(115200);
  Serial.println("Starting...");
}

void loop()
{
/*
  noInterrupts();
  long left = leftCount;
  long right = rightCount;
  interrupts();

  Serial.print("Left Count: ");
  Serial.print(left);
  Serial.print(" Right Count: ");
  Serial.println(right);
*/

  debugVision();
  delay(500);
}