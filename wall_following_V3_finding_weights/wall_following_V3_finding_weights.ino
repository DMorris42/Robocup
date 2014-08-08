/* Driving with panning GP2D12 and ultrasonic sensors to follow walls
*/

#include <Servo.h>

#define DRIVE_SPEED 130
#define STOP_SPEED 90
#define K_P 5

float USdistance = 0;
float IRLeftdistance = 0;
float IRRightdistance = 0;
float IRLongdistance = 0;
float IRShortdistance = 0;

Servo motor_left;
Servo motor_right;

int motor_left_speed = 90;
int motor_right_speed = 90;

//static byte USpin = 8;
static byte IR_long_pin = 8;
static byte LeftGP2D12pin = 10;
static byte RightGP2D12pin = 6;
static byte GP2D120pin = 4;

int US_sensing_dist = 35;
int IR_sensing_dist = 35;
static int target_distance = 20;
static int IR_long_sensing_dist = 35;
static int IR_weight_sensing_dist = 28;
static int IR_weight_found_dist = 25;
static int tolerance = 20; //Can't pickup close to wall

boolean wall_left = false;
boolean wall_right = false;
boolean wall_foward = false;
boolean short_sensed = false;
boolean weight_sensed = false;
boolean weight_found = false;
boolean wall_found = false;

static long polling_time = 0;


void setup() {
  motor_left.attach(3);
  motor_right.attach(2);
  drive();
  Serial.begin(9600);
}

void loop() {
  if (polling_time >= 800) {
    polling_time = 0;
    poll_sensors();
  }
  if (weight_sensed) {
    //Serial.println("Weight sensed");
    motor_stop();
    drive();
    while (!weight_found && !wall_found) {
      weight_polling();
    }
    //Serial.println(weight_found);
    //Serial.println(wall_found);
    while(weight_found && !wall_found) {
      //Serial.println("Weight found");
      motor_stop();
    }
    turn_right();
    delay(1000);
    weight_sensed = false;
    weight_found = false;
    wall_found = false;
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
  if (wall_foward && !wall_left && !wall_right) {
    reverse();
    delay(500);
    turn_left();
    delay(700);
    poll_sensors();
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

void weight_polling(void) {
  IRShortdistance = read_gp2d120_range(GP2D120pin);
  IRLongdistance = read_IR_long_range(IR_long_pin);
  if ((IRShortdistance != -1) && (IRShortdistance <= IR_weight_found_dist) && (IRLongdistance <= IR_weight_found_dist)) {
    wall_found = true;
    //Serial.println("Wall found");
  }
  else if ((IRShortdistance != -1) && (IRShortdistance <= IR_weight_found_dist) && (IRLongdistance > (IR_weight_found_dist + tolerance))) {
    weight_found = true;
    //Serial.println("Weight found");
  }
}

void poll_sensors(void) {
  wall_left = false;
  wall_right = false;
  wall_foward = false;
  short_sensed = false;
  weight_sensed = false;
  //USdistance = read_ul_sensor_range(USpin);
  IRLongdistance = read_IR_long_range(IR_long_pin);
  IRLeftdistance = read_gp2d12_range(LeftGP2D12pin);
  IRRightdistance = read_gp2d12_range(RightGP2D12pin);
  IRShortdistance = read_gp2d120_range(GP2D120pin);
  /*if (USdistance <= US_sensing_dist) {
    wall_foward = true;
  }*/
  if ((IRLongdistance != -1) && (IRLongdistance <= IR_long_sensing_dist)) {
    wall_foward = true;
  }
  if ((IRLeftdistance != -1) && (IRLeftdistance <= IR_sensing_dist)) {
    wall_left = true;
  }
  if ((IRRightdistance != -1) && (IRRightdistance <= IR_sensing_dist)) {
    wall_right = true;
  }
  if ((IRShortdistance != -1) && (IRShortdistance <= IR_weight_sensing_dist)) {
    short_sensed = true;
  }
  if (short_sensed && (IRLongdistance > IR_weight_sensing_dist) && (IRLongdistance != -1)) {
    weight_sensed = true;
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

float read_gp2d120_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (2914.0 /((float)tmp + 3.0)) - 1.0;
  if (tmp < 6) {
    range =  -1; // Error value
  }
  return range;
}

float read_IR_long_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (9462.0 /((float)tmp - 16.92));
  if (tmp <= 16.92) {
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

void motor_stop(void) {
  motor_left.write(STOP_SPEED);
  motor_right.write(STOP_SPEED);
}
