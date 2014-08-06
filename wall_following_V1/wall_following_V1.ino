/* Driving with panning GP2D12 and ultrasonic sensors to follow walls
*/

#include <Servo.h>

#define DRIVE_SPEED 130
#define STOP_SPEED 90
#define K_P 3

float USdistance = 0;
float IRLeftdistance = 0;
float IRRightdistance = 0;

Servo motor_left;
Servo motor_right;

int motor_left_speed = 90;
int motor_right_speed = 90;

static byte USpin = 8;
static byte LeftGP2D12pin = 10;
static byte RightGP2D12pin = 6;

int US_sensing_dist = 35;
int IR_sensing_dist = 35;
static int target_distance = 20;

boolean wall_left = false;
boolean wall_right = false;
boolean wall_foward = false;

static long polling_time = 0;


void setup() {
  motor_left.attach(3);
  motor_right.attach(2);
  drive();
}

void loop() {
  if (polling_time >= 800) {
    polling_time = 0;
    poll_sensors();
  }
  if (!wall_foward) {
    speed_control();
  }
  if (wall_foward && wall_left) {  
    turn_right();
  }
  if (wall_foward && wall_right) {
    turn_left();
  }
  if (!wall_foward && !wall_left && !wall_right) {
    drive();
  }
  polling_time++;
}


void speed_control(void) {
 if (wall_right && !wall_left) {
   motor_left.write(DRIVE_SPEED + K_P * (IRRightdistance - target_distance));
 }
 if (wall_left && !wall_right) {
   motor_right.write(DRIVE_SPEED + K_P * (IRLeftdistance - target_distance));
 }
}

void poll_sensors(void) {
  wall_left = false;
  wall_right = false;
  wall_foward = false;
  USdistance = read_ul_sensor_range(USpin);
  IRLeftdistance = read_gp2d12_range(LeftGP2D12pin);
  IRRightdistance = read_gp2d12_range(RightGP2D12pin);
  if (USdistance <= US_sensing_dist) {
    wall_foward = true;
  }
  if ((IRLeftdistance != -1) && (IRLeftdistance <= IR_sensing_dist)) {
    wall_left = true;
  }
  if ((IRRightdistance != -1) && (IRRightdistance <= IR_sensing_dist)) {
    wall_right = true;
  }
}


float read_ul_sensor_range(byte pin) {
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
  motor_left.write(DRIVE_SPEED);
  motor_right.write(DRIVE_SPEED);
} 
