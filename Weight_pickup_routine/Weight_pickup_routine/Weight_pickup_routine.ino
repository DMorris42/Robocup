/*
This code controls the rotation of the pinion stepper motor
using the rack_pinion.h library
*/

#include "rack_pinion.h"
#include "arm.h"

int Armdirpin = 31;
int Armsteppin = 30;
int RPdirpin = 35; 
int RPsteppin = 34;
static volatile int time = 0;
static volatile int prev_time = 0;

RP rack_pinion(RPdirpin, RPsteppin);
Arm arm(Armdirpin, Armsteppin);

void setup()
{
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  delay(300);
}
void loop()
{
  time = millis();
  arm.move_arm(300);
  delay(200);
  arm.arm_dir_up();
  arm.move_arm(400);
  rack_pinion.move_RP(950);
  arm.arm_dir_down();
  arm.move_arm(300);
  delay(1500);
  arm.arm_dir_up();
  arm.move_arm(300);
  rack_pinion.RP_dir_backward();
  rack_pinion.move_RP(1000);
  while (1) {
    delay(1000);
  }
}
