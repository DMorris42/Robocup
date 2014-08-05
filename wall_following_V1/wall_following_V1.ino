/* Driving with panning GP2D12 and ultrasonic sensors to follow walls
*/

#include <Servo.h>

#define WALL_LEFT 0;
#define WALL_RIGHT 1;
#define WALL_UNKNOWN 2;

float USdistance = 0;
float IR1distance = 0;
float IR2distance = 0;
Servo motor_left;
Servo motor_right;
int motor_left_speed = 90;
int motor_right_speed = 90;
int USpin = 8;
int GP2D12pin1 = 10;
int GP2D12pin2 = 0x0; //NEEDS TO BE UPDATED
int US_sensing_dist = 31;
int IR_sensing_dist = 15;
unsigned int tolerance = 5;
unsigned char wall_dir = WALL_UNKNOWN;
unsigned char wall_dir_prev = WALL_UNKNOWN;

void setup() {
  motor_left.attach(2);
  motor_right.attach(3);
  motor_left.write(motor_left_speed);
  motor_right.write(motor_right_speed);
  drive();
}

void loop() {
  USdistance = read_ul_sensor_range();
  IR1distance = read_gp2d12_range(GP2D12pin1);
  IR2distance = read_gp2d12_range(GP2D12pin2);
  if (IR1distance <= IR_sensing_dist) {
    wall_dir = WALL_LEFT;
  }
  else if (IR2distance <= IR_sensing_dist) {
    wall_dir = WALL_RIGHT;
  }
  else {
    wall_dir = WALL_UNKNOWN;
  }
}

float read_ul_sensor_range(void) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(USpin);
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

float read_gp2d12_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (6787.0 /((float)tmp - 3.0)) - 4.0;
  if (tmp < 3) {
    range =  -1; // Error value
  }
  return range;
}

void turn_left(void) {
  motor_left.write(55);
  motor_right.write(125);
}

void turn_right(void) {
  motor_left.write(125);
  motor_right.write(55);
}

void reverse(void) {
  motor_left.write(55);
  motor_right.write(55);
}

void drive(void) {
  motor_left.write(125);
  motor_right.write(125);
} 
