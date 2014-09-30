/* Routine to collect a weight */

#include "rack_pinion.h"
#include "arm.h"

int limit_switch_pin = A8;
int limit_switch_reading = LOW;

int magnet_control_pin = A6;

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
  pinMode(magnet_control_pin, OUTPUT);
  pinMode(limit_switch_pin, INPUT);
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  delay(300);
}

void loop()
{
  time = millis();
  collect_weight();
  delay(3000);
}

void collect_weight(void) {
  time = millis();
  arm.arm_dir_down();
  rack_pinion.RP_dir_forward();
  arm.move_arm(300);
  digitalWrite(magnet_control_pin, HIGH);
  delay(200);
  time = millis();
  
  arm.arm_dir_up();
  arm.move_arm(300);
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
  arm.move_arm(50);
  digitalWrite(magnet_control_pin, LOW);
  time = millis();
  
  arm.arm_dir_up();
  arm.move_arm(200);
  rack_pinion.RP_dir_backward();
  rack_pinion.move_RP(1000);
  
  delay(50);
  limit_switch_reading = digitalRead(limit_switch_pin);
  while (limit_switch_reading == HIGH) {
    rack_pinion.RP_dir_backward();
    rack_pinion.move_RP(200);
    delay(50);
    limit_switch_reading = digitalRead(limit_switch_pin);
  }
  
  time = millis();
  
  arm.arm_dir_down();
  arm.move_arm(100);
}
