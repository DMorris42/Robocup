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

int limit_switch_pin = A8;
int limit_switch_reading = LOW;

int magnet_control_pin = A6;

void setup()
{
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  pinMode(magnet_control_pin, OUTPUT);
  pinMode(limit_switch_pin, INPUT);
  delay(300);
}
void loop()
{
  pick_up();
  delay(2000);
}

void pick_up(void) {
  arm.arm_dir_down();
  rack_pinion.RP_dir_forward();
  arm.move_arm(600);
  digitalWrite(magnet_control_pin, HIGH);
  delay(200);
  
  arm.arm_dir_up();
  arm.move_arm(320);
  rack_pinion.move_RP(950);
  delay(50);
  limit_switch_reading = digitalRead(limit_switch_pin);
  while (limit_switch_reading == LOW) {
    rack_pinion.RP_dir_backward();
    rack_pinion.move_RP(100);
    rack_pinion.RP_dir_forward();
    rack_pinion.move_RP(950);
    delay(50);
    limit_switch_reading = digitalRead(limit_switch_pin);
  }
  arm.arm_dir_down();
  arm.move_arm(90);
  digitalWrite(magnet_control_pin, LOW);
  
  arm.arm_dir_up();
  arm.move_arm(500);
  rack_pinion.RP_dir_backward();
  rack_pinion.move_RP(1000);
  
  delay(50);
  limit_switch_reading = digitalRead(limit_switch_pin);
  while (limit_switch_reading == HIGH) {
    rack_pinion.RP_dir_backward();
    rack_pinion.move_RP(600);
    delay(50);
    limit_switch_reading = digitalRead(limit_switch_pin);
    if (limit_switch_reading == HIGH) {
      rack_pinion.RP_dir_forward();
      rack_pinion.move_RP(100);
      delay(50);
    }
    limit_switch_reading = digitalRead(limit_switch_pin);
  }
  morse_done();
}

void morse_done(void) {
  dash();
  dot();
  dot();
  dash();
  dash();
  dash();
  dash();
  dot();
  dot();
}

void dash(void) {
  digitalWrite(magnet_control_pin, HIGH);
  delay(600);
  digitalWrite(magnet_control_pin, LOW);
  delay(600);
}

void dot(void) {
  digitalWrite(magnet_control_pin, HIGH);
  delay(200);
  digitalWrite(magnet_control_pin, LOW);
  delay(200);
}
