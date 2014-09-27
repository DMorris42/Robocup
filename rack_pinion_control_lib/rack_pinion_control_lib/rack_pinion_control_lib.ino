/*
This code controls the rotation of the pinion stepper motor
using the rack_pinion.h library
*/

#include "rack_pinion.h"

int RPdirpin = 35; 
int RPsteppin = 34;
static volatile int time = 0;
static volatile int prev_time = 0;

//static TRAY_STATE tray_dir = unknown;

RP rack_pinion(RPdirpin, RPsteppin);

void setup()
{
  rack_pinion.RP_dir_forward();
  delay(300);
}
void loop()
{
  time = millis();
  //delay(800);
  //RP_dir_forward();
  rack_pinion.move_RP(1);
  /*if ((time - prev_time) >= 600) {
    delay(1000);
    RP_dir_backward();
    move_RP(400);
    delay(500);
    time = millis();
    prev_time = time;
  }*/
  if ((time - prev_time) >= 800) {
    if (rack_pinion.get_current_dir() == forwards) {
      rack_pinion.RP_dir_backward();
    }
    else {
    rack_pinion.RP_dir_forward();
    }
    time = millis();
    prev_time = time;
  }
}
  
