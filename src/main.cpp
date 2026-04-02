#include <Arduino.h>

#include "communication.h"
#include "robot_control.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  robot_control_setup();
  communication_setup();
}

void loop()
{
  communication_loop();
}