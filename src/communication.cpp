#include <Arduino.h>
#include <Wire.h>

#include "communication.h"
#include "robot_control.h"

volatile int i2c_speed_left = 0;
volatile int i2c_speed_right = 0;
volatile int i2c_steering_angle = CENTRE_ANGLE;
volatile bool i2c_new_command = false;

void onReceive(int howMany) {
  if (Wire1.available() >= 3) {
    char cmd = Wire1.read();
    byte val1 = Wire1.read();
    byte val2 = Wire1.read();

    if (cmd == 's') { // Speed command
      i2c_speed_left = map(val1, 0, 255, -255, 255);
      i2c_speed_right = map(val2, 0, 255, -255, 255);
      i2c_new_command = true;
    } else if (cmd == 't') { // Steering command
      i2c_steering_angle = map(val1, 0, 255, MAX_LEFT, MAX_RIGHT);
      i2c_new_command = true;
    }
  }
  // Discard extra bytes
  while (Wire1.available() > 0) {
    Wire1.read();
  }
}

void communication_setup(){
  Wire1.begin(I2C_SLAVE_ADDR);
  Wire1.onReceive(onReceive);
}

void communication_loop(){
  if (i2c_new_command) {
    Serial.printf("Received:\n Speed  Left: %d Right: %d Angle: %d\n", i2c_speed_left, i2c_speed_right, i2c_steering_angle);
    run(i2c_speed_left, i2c_speed_right);
    steer(i2c_steering_angle);
    i2c_new_command = false;
  }
}

