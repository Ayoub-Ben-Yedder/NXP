#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>
#include <Encoder.h>

// 0 not connected
// 1 not connected
#define ENC_LEFT_A   2
#define ENC_LEFT_B   3
#define ENC_RIGHT_A  4
#define ENC_RIGHT_B  5

#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9

/*
SPI PINS, lel Camera Pixy
10 CS
11 MOSI
12 MISO
13 SCK
*/

#define LED 14

// 15 not connected

/*
I2C Pins, lel Communication m3a allaho a3lem balek nest7a9oulhom
SCL1 16
SDA1 17
*/

// 18 not connected
// 

#define STEER_SERVO  19

/*
Serial5 Pins, lel Communication m3a el ESP32
TX5 20
RX5 21
*/

#define ECHO 22
#define TRIG 23




#define CENTER_ANGLE    90
#define MAX_LEFT_ANGLE  0 
#define MAX_RIGHT_ANGLE 180


#define TICKS_PER_REV  360.0f
#define WHEEL_DIAMETER 0.065f     // meters

Servo steeringServo;

Pixy2 pixy;

Encoder leftEncoder(ENC_LEFT_A, ENC_LEFT_B);
Encoder rightEncoder(ENC_RIGHT_A, ENC_RIGHT_B);

long lastLeftCount = 0;
long lastRightCount = 0;

unsigned long lastSpeedTime = 0;

float leftSpeed = 0; // ticks/sec
float rightSpeed = 0;

// ===== BEGIN PI SPEED CONTROL =====
float targetLeftSpeed = 0; // m/s
float targetRightSpeed = 0;

float leftPWM = 0;
float rightPWM = 0;

float Kp = 3.0;
float Ki = 0.0;

float leftIntegral = 0;
float rightIntegral = 0;

unsigned long lastControlTime = 0;
// ===== END PI SPEED CONTROL =====

bool teleopMode = false;

static const uint8_t kStartByte = 0xAA;
static const uint8_t kTypeInfo = 0x00;
static const uint8_t kTypeSet = 0x01;

static const uint8_t kIdSpeed = 0x00;
static const uint8_t kIdSteer = 0x01;
static const uint8_t kIdAuto = 0x02;
static const uint8_t kIdVec = 0x03;

static const uint8_t kMaxDataLen = 8;

enum RxState : uint8_t {
  RX_WAIT_START = 0,
  RX_READ_TYPE,
  RX_READ_ID,
  RX_READ_LEN,
  RX_READ_DATA,
  RX_READ_CRC
};

static RxState rxState = RX_WAIT_START;
static uint8_t rxType = 0;
static uint8_t rxId = 0;
static uint8_t rxLen = 0;
static uint8_t rxData[kMaxDataLen];
static uint8_t rxIndex = 0;
static uint8_t rxCrc = 0;

static uint8_t currentSteerAngle = CENTER_ANGLE;

static uint8_t lastInfoSpeedR = 0;
static uint8_t lastInfoSpeedL = 0;
static uint8_t lastInfoSteer = CENTER_ANGLE;
static uint8_t lastInfoAuto = 0xFF;
static uint16_t lastVec[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

float ticksToMetersPerSecond(float ticksPerSecond)
{
  float revPerSec = ticksPerSecond / TICKS_PER_REV;
  float wheelCirc = PI * WHEEL_DIAMETER;
  return revPerSec * wheelCirc;
}

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

void debugSpeed(){
  Serial.print("L m/s: ");
  Serial.print(ticksToMetersPerSecond(leftSpeed), 3);
  Serial.print("  PWM: ");
  Serial.print(leftPWM);

  Serial.print(" | R m/s: ");
  Serial.print(ticksToMetersPerSecond(rightSpeed), 3);
  Serial.print("  PWM: ");
  Serial.println(rightPWM);
}

void steer(int angle){
  angle = constrain(angle, MAX_LEFT_ANGLE, MAX_RIGHT_ANGLE);
  steeringServo.write(angle);
  currentSteerAngle = static_cast<uint8_t>(angle);
}

void run(int speedLeft, int speedRight)
{
  if (speedRight >= 0)
  {
    analogWrite(IN3, speedRight);
    analogWrite(IN4, 0);
  }
  else
  {
    analogWrite(IN3, 0);
    analogWrite(IN4, -speedRight);
  }

  if (speedLeft >= 0)
  {
    analogWrite(IN1, speedLeft);
    analogWrite(IN2, 0);
  }
  else
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, -speedLeft);
  }
}

long readDistance()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000); // timeout 30ms
  long distance = duration * 0.034 / 2;       // cm

  return distance;
}


void updateSpeed()
{
  unsigned long now = millis();
  unsigned long dt = now - lastSpeedTime;

  if (dt >= 50)   // 20 Hz update
  {
    long leftCount = leftEncoder.read();
    long rightCount = rightEncoder.read();

    long dLeft = leftCount - lastLeftCount;
    long dRight = rightCount - lastRightCount;

    leftSpeed  = (dLeft  * 1000.0f) / dt;  // ticks/sec
    rightSpeed = (dRight * 1000.0f) / dt;

    lastLeftCount = leftCount;
    lastRightCount = rightCount;
    lastSpeedTime = now;
  }
}

void setSpeed(float left_mps, float right_mps)
{
  targetLeftSpeed = left_mps;
  targetRightSpeed = right_mps;
}

void updatePI()
{
  
  unsigned long now = millis();
  float dt = (now - lastControlTime) / 1000.0f;

  if (dt < 0.02f) return;   // 50 Hz

  // Current speed (m/s)
  float currentLeft  = ticksToMetersPerSecond(leftSpeed);
  float currentRight = ticksToMetersPerSecond(rightSpeed);

  // Error
  float leftError  = targetLeftSpeed  - currentLeft;
  float rightError = targetRightSpeed - currentRight;

  // Integrate
  leftIntegral  += leftError * dt;
  rightIntegral += rightError * dt;

  // ---- Reset integrator if target speed is zero ----
  if (targetLeftSpeed == 0) leftIntegral = 0;
  if (targetRightSpeed == 0) rightIntegral = 0;

  // Anti-windup (VERY IMPORTANT)
  leftIntegral  = constrain(leftIntegral,  -20, 20);
  rightIntegral = constrain(rightIntegral, -20, 20);

  // PI output (incremental form for stability)
  leftPWM  += Kp * leftError  + Ki * leftIntegral;
  rightPWM += Kp * rightError + Ki * rightIntegral;

  // Constrain PWM
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  run((int)leftPWM, (int)rightPWM);

  lastControlTime = now;
}

static uint8_t crc8(uint8_t type, uint8_t id, uint8_t len, const uint8_t *data) {
  uint8_t crc = type ^ id ^ len;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
  }
  return crc;
}

static void sendFrame(uint8_t type, uint8_t id, const uint8_t *data, uint8_t len) {
  uint8_t crc = crc8(type, id, len, data);
  Serial1.write(kStartByte);
  Serial1.write(type);
  Serial1.write(id);
  Serial1.write(len);
  for (uint8_t i = 0; i < len; ++i) {
    Serial1.write(data[i]);
  }
  Serial1.write(crc);
}

static uint8_t clampPwmToByte(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return static_cast<uint8_t>(value);
}

static void sendInfoSpeed() {
  uint8_t data[2];
  data[0] = clampPwmToByte(static_cast<int>(rightPWM));
  data[1] = clampPwmToByte(static_cast<int>(leftPWM));
  sendFrame(kTypeInfo, kIdSpeed, data, 2);
  lastInfoSpeedR = data[0];
  lastInfoSpeedL = data[1];
}

static void sendInfoSteer() {
  uint8_t data[1] = { currentSteerAngle };
  sendFrame(kTypeInfo, kIdSteer, data, 1);
  lastInfoSteer = currentSteerAngle;
}

static void sendInfoAuto() {
  uint8_t data[1] = { static_cast<uint8_t>(teleopMode ? 0x00 : 0x01) };
  sendFrame(kTypeInfo, kIdAuto, data, 1);
  lastInfoAuto = data[0];
}

static void sendInfoVec(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t data[8];
  data[0] = static_cast<uint8_t>(x0 & 0xFF);
  data[1] = static_cast<uint8_t>((x0 >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>(y0 & 0xFF);
  data[3] = static_cast<uint8_t>((y0 >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(x1 & 0xFF);
  data[5] = static_cast<uint8_t>((x1 >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>(y1 & 0xFF);
  data[7] = static_cast<uint8_t>((y1 >> 8) & 0xFF);
  sendFrame(kTypeInfo, kIdVec, data, 8);
  lastVec[0] = x0;
  lastVec[1] = y0;
  lastVec[2] = x1;
  lastVec[3] = y1;
}

static void handleSetFrame(uint8_t id, const uint8_t *data, uint8_t len) {
  if (id == kIdSpeed && len == 2) {
    uint8_t pwmR = data[0];
    uint8_t pwmL = data[1];
    rightPWM = pwmR;
    leftPWM = pwmL;
    run(static_cast<int>(leftPWM), static_cast<int>(rightPWM));
  } else if (id == kIdSteer && len == 1) {
    steer(static_cast<int>(data[0]));
  } else if (id == kIdAuto && len == 1) {
    teleopMode = (data[0] == 0x00);
    leftIntegral = 0;
    rightIntegral = 0;
    if (teleopMode) {
      run(0, 0);
    }
  }
}

static void processRxByte(uint8_t byteIn) {
  switch (rxState) {
    case RX_WAIT_START:
      if (byteIn == kStartByte) {
        rxState = RX_READ_TYPE;
      }
      break;
    case RX_READ_TYPE:
      rxType = byteIn;
      rxCrc = byteIn;
      rxState = RX_READ_ID;
      break;
    case RX_READ_ID:
      rxId = byteIn;
      rxCrc ^= byteIn;
      rxState = RX_READ_LEN;
      break;
    case RX_READ_LEN:
      rxLen = byteIn;
      rxCrc ^= byteIn;
      if (rxLen > kMaxDataLen) {
        rxState = RX_WAIT_START;
      } else if (rxLen == 0) {
        rxState = RX_READ_CRC;
      } else {
        rxIndex = 0;
        rxState = RX_READ_DATA;
      }
      break;
    case RX_READ_DATA:
      rxData[rxIndex++] = byteIn;
      rxCrc ^= byteIn;
      if (rxIndex >= rxLen) {
        rxState = RX_READ_CRC;
      }
      break;
    case RX_READ_CRC:
      if (rxCrc == byteIn && rxType == kTypeSet) {
        handleSetFrame(rxId, rxData, rxLen);
      }
      rxState = RX_WAIT_START;
      break;
    default:
      rxState = RX_WAIT_START;
      break;
  }
}

static void pollSerial() {
  while (Serial1.available()) {
    uint8_t byteIn = static_cast<uint8_t>(Serial1.read());
    processRxByte(byteIn);
  }
}

static void sendInfoIfChanged() {
  uint8_t speedR = clampPwmToByte(static_cast<int>(rightPWM));
  uint8_t speedL = clampPwmToByte(static_cast<int>(leftPWM));
  if (speedR != lastInfoSpeedR || speedL != lastInfoSpeedL) {
    sendInfoSpeed();
  }

  if (currentSteerAngle != lastInfoSteer) {
    sendInfoSteer();
  }

  uint8_t autoValue = static_cast<uint8_t>(teleopMode ? 0x00 : 0x01);
  if (autoValue != lastInfoAuto) {
    sendInfoAuto();
  }

  readMainVec();
  if (pixy.line.numVectors > 0) {
    uint16_t x0 = pixy.line.vectors[0].m_x0;
    uint16_t y0 = pixy.line.vectors[0].m_y0;
    uint16_t x1 = pixy.line.vectors[0].m_x1;
    uint16_t y1 = pixy.line.vectors[0].m_y1;
    if (x0 != lastVec[0] || y0 != lastVec[1] || x1 != lastVec[2] || y1 != lastVec[3]) {
      sendInfoVec(x0, y0, x1, y1);
    }
  }
}

void setup()
{
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  run(0, 0);

  steeringServo.attach(STEER_SERVO);
  steeringServo.write(CENTER_ANGLE);

  initPixy();

  Serial.begin(921600);
  Serial1.begin(921600);

  sendInfoSpeed();
  sendInfoSteer();
  sendInfoAuto();
  readMainVec();
  if (pixy.line.numVectors > 0) {
    sendInfoVec(pixy.line.vectors[0].m_x0,
                pixy.line.vectors[0].m_y0,
                pixy.line.vectors[0].m_x1,
                pixy.line.vectors[0].m_y1);
  }
}

void loop()
{
  pollSerial();

  updateSpeed();

  if (!teleopMode) {
    updatePI();
  }

  static unsigned long lastInfoCheck = 0;
  if (millis() - lastInfoCheck > 50) {
    sendInfoIfChanged();
    lastInfoCheck = millis();
  }
}