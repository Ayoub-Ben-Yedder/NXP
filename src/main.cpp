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

constexpr float PIXY_FORWARD_ANGLE = 270.0;
constexpr float PIXY_STEER_DEADBAND = 10.0;
constexpr uint8_t STEERING_DELAY_CYCLES = 2;
constexpr float STEERING_QUEUE_THRESHOLD = 2.0;
constexpr float STEERING_CENTER_BIAS = 5.0;
constexpr float STEERING_SOFT_LIMIT_START = 25.0;
constexpr float STEERING_SOFT_LIMIT_FACTOR = 0.5;

Servo steer_servo;
Pixy2 pixy;
static float lastSteeringAngle = CENTRE_ANGLE;
static float pendingSteeringAngle = CENTRE_ANGLE;
static uint8_t steeringDelayCounter = 0;

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

float normalizeAngle(float angle)
{
  while (angle < 0.0)
  {
    angle += 360.0;
  }
  while (angle >= 360.0)
  {
    angle -= 360.0;
  }
  return angle;
}

bool isHorizontalAngle(float angle)
{
  angle = normalizeAngle(angle);
  return (angle <= PIXY_STEER_DEADBAND || angle >= 360.0 - PIXY_STEER_DEADBAND) || (fabs(angle - 180.0) <= PIXY_STEER_DEADBAND);
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float angleDelta(float from, float to)
{
  float delta = normalizeAngle(to) - normalizeAngle(from);
  while (delta <= -180.0)
  {
    delta += 360.0;
  }
  while (delta > 180.0)
  {
    delta -= 360.0;
  }
  return delta;
}

float computeSteeringAngle(const Vector *vectors, uint8_t numVectors)
{
  if (numVectors == 0)
  {
    return lastSteeringAngle;
  }

  float weightedDelta = 0.0;
  float totalWeight = 0.0;

  for (uint8_t i = 0; i < numVectors; i++)
  {
    float angle = computeAngle(vectors[i]);
    float length = computeLength(vectors[i]);
    float delta = angleDelta(PIXY_FORWARD_ANGLE, angle);

    if (fabs(delta) > 90.0)
    {
      delta = angleDelta(PIXY_FORWARD_ANGLE, normalizeAngle(angle + 180.0));
    }

    float weight = max(length, 1.0f);
    weightedDelta += delta * weight;
    totalWeight += weight;
  }

  float avgDelta = totalWeight > 0.0 ? weightedDelta / totalWeight : 0.0;
  avgDelta = constrain(avgDelta, -45.0, 45.0);
  return mapFloat(avgDelta, -45.0, 45.0, MAX_LEFT, MAX_RIGHT);
}

float biasSteeringTowardCenter(float steeringAngle)
{
  if (steeringAngle > CENTRE_ANGLE)
  {
    return max(CENTRE_ANGLE, steeringAngle - STEERING_CENTER_BIAS);
  }
  if (steeringAngle < CENTRE_ANGLE)
  {
    return min(CENTRE_ANGLE, steeringAngle + STEERING_CENTER_BIAS);
  }
  return steeringAngle;
}

float softenExtremeSteering(float steeringAngle)
{
  float delta = steeringAngle - CENTRE_ANGLE;
  float absDelta = fabs(delta);
  if (absDelta <= STEERING_SOFT_LIMIT_START)
  {
    return steeringAngle;
  }

  float sign = delta > 0 ? 1.0f : -1.0f;
  float extra = absDelta - STEERING_SOFT_LIMIT_START;
  float softenedExtra = extra * STEERING_SOFT_LIMIT_FACTOR;
  return CENTRE_ANGLE + sign * (STEERING_SOFT_LIMIT_START + softenedExtra);
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
  Serial.println("Serial monitor ready at 115200");

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
    int x0 = pixy.line.vectors[i].m_x0;
    int y0 = pixy.line.vectors[i].m_y0;
    int x1 = pixy.line.vectors[i].m_x1;
    int y1 = pixy.line.vectors[i].m_y1;
    float avgY = (y0 + y1) / 2.0f;

    if (numVectors < MAX_VECTORS && length > MIN_VECTOR_LENGTH && !isHorizontalAngle(angle) && !((avgY >= 0 && avgY <= 2) || (avgY >= 48 && avgY <= 50)))
    {
      vectors[numVectors] = pixy.line.vectors[i];
      numVectors++;
    }
    else if (isHorizontalAngle(angle))
    {
      Serial.printf("Skipping horizontal vector %d: Angle=%.2f, Length=%.2f, x0=%d, y0=%d, x1=%d, y1=%d\n", i, angle, length, x0, y0, x1, y1);
    }
    else if ((avgY >= 0 && avgY <= 2) || (avgY >= 48 && avgY <= 50))
    {
      Serial.printf("Ignoring vector in y-range %d: Angle=%.2f, Length=%.2f, x0=%d, y0=%d, x1=%d, y1=%d\n", i, angle, length, x0, y0, x1, y1);
    }
    else
    {
      Serial.printf("Ignoring short vector %d: Angle=%.2f, Length=%.2f, x0=%d, y0=%d, x1=%d, y1=%d\n", i, angle, length, x0, y0, x1, y1);
    }
    Serial.printf("Vector %d: Angle=%.2f, Length=%.2f, x0=%d, y0=%d, x1=%d, y1=%d\n", i, angle, length, x0, y0, x1, y1);
  }
  
  sortVectorsByLength(vectors, numVectors);
  for(int i = 0; i < numVectors; i++)
  {
    float angle = computeAngle(vectors[i]);
    float length = computeLength(vectors[i]);
    int x0 = vectors[i].m_x0;
    int y0 = vectors[i].m_y0;
    int x1 = vectors[i].m_x1;
    int y1 = vectors[i].m_y1;
    Serial.printf("Sorted Vector %d: Angle=%.2f, Length=%.2f, x0=%d, y0=%d, x1=%d, y1=%d\n", i, angle, length, x0, y0, x1, y1);
  }

  float targetSteering = lastSteeringAngle;
  float mainVectorCenterX = -1.0f;

  if (numVectors >= 2)
  {
    float angle1 = computeAngle(vectors[0]);
    float angle2 = computeAngle(vectors[1]);
    if ((abs(angle1 - angle2) > 70 && abs(angle1 - angle2) < 110) || (abs(angle1 - angle2) > 250 && abs(angle1 - angle2) < 290))
    {
      // kana fama angle 90 binet les deux vecteurs, bech nwaslou l goddam (most likely fama intersection)
      targetSteering = CENTRE_ANGLE;
    }
    else
    {
      targetSteering = computeSteeringAngle(vectors, numVectors);
    }
  }
  else if (numVectors == 1)
  {
    Serial.println("Not enough vectors detected, using the best one");
    targetSteering = computeSteeringAngle(vectors, numVectors);
  }
  else
  {
    Serial.println("No valid vectors, keeping previous steering angle");
    targetSteering = lastSteeringAngle;
  }

  if (numVectors > 0)
  {
    mainVectorCenterX = (vectors[0].m_x0 + vectors[0].m_x1) / 2.0f;
    if (fabs(targetSteering - CENTRE_ANGLE) <= 10.0f && mainVectorCenterX >= 20.0f && mainVectorCenterX <= 60.0f)
    {
      float correction = 0.0f;
      if (mainVectorCenterX >= 20.0f && mainVectorCenterX < 40.0f)
      {
        // Small turn to the right
        correction = 15.0f;
        Serial.printf("Small turn RIGHT: mainVectorCenterX=%.1f\n", mainVectorCenterX);
      }
      else if (mainVectorCenterX >= 40.0f && mainVectorCenterX <= 60.0f)
      {
        // Small turn to the left
        correction = -15.0f;
        Serial.printf("Small turn LEFT: mainVectorCenterX=%.1f\n", mainVectorCenterX);
      }
      
      targetSteering = constrain(targetSteering + correction, MAX_LEFT, MAX_RIGHT);
      Serial.printf("Center-steer correction: mainVectorCenterX=%.1f, correction=%.1f -> %.1f\n",
                    mainVectorCenterX,
                    correction,
                    targetSteering);
    }
  }

  targetSteering = biasSteeringTowardCenter(targetSteering);
  targetSteering = softenExtremeSteering(targetSteering);

  if (fabs(targetSteering - pendingSteeringAngle) > STEERING_QUEUE_THRESHOLD)
  {
    pendingSteeringAngle = targetSteering;
    steeringDelayCounter = STEERING_DELAY_CYCLES;
    Serial.printf("Queued new steering target %0.2f (delay %d)\n", pendingSteeringAngle, steeringDelayCounter);
  }

  if (steeringDelayCounter > 0)
  {
    steeringDelayCounter--;
    Serial.printf("Delaying steering response, %d cycles left\n", steeringDelayCounter);
  }

  if (steeringDelayCounter == 0 && pendingSteeringAngle != lastSteeringAngle)
  {
    Serial.printf("Applying delayed steering %0.2f\n", pendingSteeringAngle);
    steer(pendingSteeringAngle);
    lastSteeringAngle = pendingSteeringAngle;
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