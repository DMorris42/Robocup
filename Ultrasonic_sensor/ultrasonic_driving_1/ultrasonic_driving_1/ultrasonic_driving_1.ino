/* Code for using the ultrasonic sensor while driving
   Sensor has a minimum range of 30 cm!
*/

#include <Servo.h>

int pin = 8; //Attached to A5
float distance = 0;
Servo motor_left;
Servo motor_right;
int motor_left_speed = 90;
int motor_right_speed = 90;

void setup() {
  motor_left.attach(3);
  motor_right.attach(2);
  motor_left.write(motor_left_speed);
  motor_right.write(motor_right_speed);
  drive();
}

void loop() {
  distance = read_ul_sensor_range();
  if (distance <= 40) {
    turn_left();
  }
  else {
    drive();
  }
  delay(100);
}

float read_ul_sensor_range(void) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  //Min val ~ 56 (dist. <= 30cm), ~58 at 30cm
  //ADC value increases by ~20 per 10 cm (from rough testing; check with Julian)
  if (tmp <= 58) {
    range = 30.0;
  }
  else {
    //Works; slight underestimate though
    range = 30.0 + (float(tmp) - 56.0)/2.0;
  }
  return range;
}

void turn_left(void) {
  motor_left.write(125);
  motor_right.write(55);
  delay(2000);
}

void turn_right(void) {
  motor_left.write(55);
  motor_right.write(125);
  delay(2000);
}

void reverse(void) {
  motor_left.write(125);
  motor_right.write(125);
  delay(1000);
}

void drive(void) {
  motor_left.write(55);
  motor_right.write(55);
}    
