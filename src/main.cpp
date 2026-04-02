#include <Arduino.h>
#include <Pixy2.h>
#include <Servo.h>
#include <Wire.h>

constexpr uint8_t I2C_SLAVE_ADDR = 0x08;

#define MAX_VECTORS 20
#define MIN_VECTOR_LENGTH 10

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

#define LED 22
#define ECHO 15
#define TRIG 14
#define SERVO_PIN 23

#define CENTRE_ANGLE 135
#define MAX_RIGHT CENTRE_ANGLE + 45
#define MAX_LEFT CENTRE_ANGLE - 45

Servo steer_servo;
Pixy2 pixy;

volatile long leftCount = 0;
volatile long rightCount = 0;

volatile int i2c_speed_left = 0;
volatile int i2c_speed_right = 0;
volatile int i2c_steering_angle = CENTRE_ANGLE;
volatile bool i2c_new_command = false;

void leftEncoderISR()
{
  if (digitalReadFast(ENC_LEFT_B) == digitalReadFast(ENC_LEFT_A))
  {
    leftCount++;
  }
  else
  {
    leftCount--;
  }
}

void rightEncoderISR()
{
  if (digitalReadFast(ENC_RIGHT_B) == digitalReadFast(ENC_RIGHT_A))
  {
    rightCount++;
  }
  else
  {
    rightCount--;
  }
}

void initPixy()
{
  pixy.init();
  pixy.changeProg("line");
}

void steer(int angle)
{
  angle = constrain(angle, MAX_LEFT, MAX_RIGHT);
  steer_servo.write(angle);
}

void run(int speedLeft, int speedRight)
{
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

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

void onReceive(int howMany)
{
  if (howMany < 3)
  {
    while (Wire1.available() > 0)
    {
      Wire1.read();
    }
    return;
  }

  const char cmd = static_cast<char>(Wire1.read());
  const uint8_t val1 = Wire1.read();
  const uint8_t val2 = Wire1.read();

  if (cmd == 's')
  {
    i2c_speed_left = map(val1, 0, 255, -255, 255);
    i2c_speed_right = map(val2, 0, 255, -255, 255);
    i2c_new_command = true;
  }
  else if (cmd == 't')
  {
    i2c_steering_angle = map(val1, 0, 255, MAX_LEFT, MAX_RIGHT);
    i2c_new_command = true;
  }

  while (Wire1.available() > 0)
  {
    Wire1.read();
  }
}

void communication_setup()
{
  Wire1.begin(I2C_SLAVE_ADDR);
  Wire1.onReceive(onReceive);
}

void communication_loop()
{
  if (!i2c_new_command)
  {
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

float computeAngle(const Vector &vec)
{
  float res = atan2(vec.m_y1 - vec.m_y0, vec.m_x1 - vec.m_x0) * 180.0 / PI;
  while(res < 0)
  {
    res += 360;
  }
  while(res >= 360)
  {
    res -= 360;
  }
  return res;
}

float computeLength(const Vector &vec)
{
  return sqrt(pow(vec.m_x1 - vec.m_x0, 2) + pow(vec.m_y1 - vec.m_y0, 2));
}

void sortVectorsByLength(Vector *vectors, uint8_t numVectors)
{
  for (uint8_t i = 0; i < numVectors - 1; i++)
  {
    for (uint8_t j = 0; j < numVectors - i - 1; j++)
    {
      float length1 = computeLength(vectors[j]);
      float length2 = computeLength(vectors[j + 1]);
      if (length1 < length2)
      {
        Vector temp = vectors[j];
        vectors[j] = vectors[j + 1];
        vectors[j + 1] = temp;
      }
    }
  }
}

void setup()
{
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

  // communication_setup();

  steer_servo.attach(SERVO_PIN);
  steer(CENTRE_ANGLE);

  initPixy();

  digitalWrite(LED, HIGH);

  run(150, 150);
}

void loop()
{
  // communication_loop();
  Vector vectors[MAX_VECTORS];
  uint8_t numVectors = 0;
  pixy.line.getAllFeatures();
  Serial.printf("Detected %d vectors\n", pixy.line.numVectors);
  for (uint8_t i = 0; i < pixy.line.numVectors; i++)
  {
    float angle = computeAngle(pixy.line.vectors[i]);
    float length = computeLength(pixy.line.vectors[i]);
    if (numVectors < MAX_VECTORS && length > MIN_VECTOR_LENGTH)
    {
      vectors[numVectors] = pixy.line.vectors[i];
      numVectors++;
    }
    Serial.printf("Vector %d: Angle=%.2f, Length=%.2f\n", i, angle, length);
  }
  
  sortVectorsByLength(vectors, numVectors);
  for(int i = 0; i < numVectors; i++)
  {
    float angle = computeAngle(vectors[i]);
    float length = computeLength(vectors[i]);
    Serial.printf("Sorted Vector %d: Angle=%.2f, Length=%.2f\n", i, angle, length);
  }

  if(numVectors >= 2)
  {
    float angle1 = computeAngle(vectors[0]);
    float angle2 = computeAngle(vectors[1]);
    float avr_angle = (angle1 + angle2) / 2.0;
    Serial.printf("Angle1: %.2f, Angle2: %.2f, Average Angle: %.2f\n", angle1, angle2, avr_angle);
    steer(map(avr_angle, 200, 360, MAX_LEFT, MAX_RIGHT));
  }else{
    Serial.println("Not enough vectors detected, using the best one");
    steer(map(computeAngle(vectors[0]), 200, 360, MAX_LEFT, MAX_RIGHT));
  }
  Serial.println("---------------------------------------------------------------");

/*
Raisonnment Zabkawoui:
all vecs have weights
weight = max y of the vector
Somme [ vectors * weight ]  / somme of weights = Vecteur de raisonnement
steering angle = angle of the vector de raisonnement
*/

  //delay(2000);
}