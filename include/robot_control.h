#pragma once

#include <Arduino.h>

constexpr uint8_t I2C_SLAVE_ADDR = 0x08;

constexpr int CENTRE_ANGLE = 135;
constexpr int MAX_RIGHT = CENTRE_ANGLE + 45;
constexpr int MAX_LEFT = CENTRE_ANGLE - 45;

extern volatile long leftCount;
extern volatile long rightCount;

void robot_control_setup();
void initPixy();
void steer(int angle);
void run(int speedLeft, int speedRight);
