#include <Arduino.h>
#include <Servo.h>
#include <Pixy2.h>
#include <Encoder.h>

#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12

#define TRIG 14
#define ECHO 15

#define STEER_SERVO     23

#define CENTER_ANGLE    90
#define MAX_LEFT_ANGLE  0 
#define MAX_RIGHT_ANGLE 180

#define ENC_LEFT_A   4
#define ENC_LEFT_B   5
#define ENC_RIGHT_A  6
#define ENC_RIGHT_B  7

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

float ticksToMetersPerSecond(float ticksPerSecond)
{
  float revPerSec = ticksPerSecond / TICKS_PER_REV;
  float wheelCirc = PI * WHEEL_DIAMETER;
  return revPerSec * wheelCirc;
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

void sendDebugSpeed() {
  Serial1.print("SPD,");
  Serial1.print(ticksToMetersPerSecond(leftSpeed), 3);
  Serial1.print(",");
  Serial1.print(leftPWM);
  Serial1.print(",");
  Serial1.print(ticksToMetersPerSecond(rightSpeed), 3);
  Serial1.print(",");
  Serial1.println(rightPWM);
}

void sendDebugVectors() {
  readAllVecs();
  for (int i = 0; i < pixy.line.numVectors; i++) {
    Serial1.print("VEC,");
    Serial1.print(i);
    Serial1.print(",");
    Serial1.print(pixy.line.vectors[i].m_x0);
    Serial1.print(",");
    Serial1.print(pixy.line.vectors[i].m_y0);
    Serial1.print(",");
    Serial1.print(pixy.line.vectors[i].m_x1);
    Serial1.print(",");
    Serial1.println(pixy.line.vectors[i].m_y1);
  }
}

void sendDebugInfo(){
  sendDebugSpeed(); 
  sendDebugVectors();
}

void handleTeleop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim(); // remove \r or spaces

    if (cmd.startsWith("STOP")) {  // emergency stop
      run(0, 0);
      steer(CENTER_ANGLE);
    } else if (cmd.startsWith("SPD")) {  
      // Example: SPD,120,130
      int comma1 = cmd.indexOf(',');
      int comma2 = cmd.indexOf(',', comma1 + 1);
      if (comma1 > 0 && comma2 > comma1) {
        int leftPWM = cmd.substring(comma1 + 1, comma2).toInt();
        int rightPWM = cmd.substring(comma2 + 1).toInt();
        run(leftPWM, rightPWM);
      }
    } else if (cmd.startsWith("STEER")) {
      // Example: STEER,90
      int comma = cmd.indexOf(',');
      if (comma > 0) {
        int angle = cmd.substring(comma + 1).toInt();
        steer(angle);
      }
    } else if (cmd.startsWith("AUTO")){
      teleopMode = false;
      leftIntegral = 0; rightIntegral = 0;
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
}

void loop()
{
  // Check if ESP32 wants teleop mode
  if (Serial1.available()) {
    String startCmd = Serial1.readStringUntil('\n');
    startCmd.trim();
    if (startCmd == "TELEOP") {
      teleopMode = true;
      run(0,0);  // stop autonomous
      leftIntegral = 0; rightIntegral = 0;
    }
  }

  if (teleopMode) {
    handleTeleop();  // Teensy executes commands from ESP32
  } else {
    updateSpeed();
    updatePI();
    
    // Optionally send debug info periodically
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 100) {  // 10 Hz
      sendDebugInfo();
      lastDebug = millis();
    }
  }
}