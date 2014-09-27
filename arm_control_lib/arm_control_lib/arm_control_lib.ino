/*
This code controls 
the rotation of the arm stepper motor using a library
*/
#include "arm.h"

// Currently on stepper port 3 (pins closest to motor, far side of board)
int M3dirpin = 31; //35?
int M3steppin = 30;  //34?
static volatile int time = 0;
static volatile int prev_time = 0;

Arm arm(M3dirpin, M3steppin);

void setup()
{
  delay(300);
}
void loop()
{
  time = millis();
  /*delay(800);
  arm_dir_down();
  move_arm(90);*/
  //delay(800);
  arm.arm_dir_up();
  arm.move_arm(1);
  if ((time - prev_time) >= 800) {
    /*arm_dir_down();
    move_arm(200);*/
    time = millis();
    prev_time = time;
    while(1) {
    //Serial.println("Done");
    delay(1000);
  }
  }
  /*arm_dir_up();
  move_arm(1);
  if (time >= 1500) {
    arm_dir_down();
    while (time < 3000) {
      time = millis();
      move_arm(1);
    }
  }
  while (1) {
  }*/
}
  
