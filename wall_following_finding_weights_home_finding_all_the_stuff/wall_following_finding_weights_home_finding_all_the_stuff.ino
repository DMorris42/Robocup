/* Driving with panning GP2D12 and ultrasonic sensors to follow walls
*/

#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define DRIVE_SPEED 130
#define STOP_SPEED 90
#define K_P 10
// set to false if using a common cathode LED
#define commonAnode true

static const char BLUE_HOME = 1;
static const char GREEN_HOME = 2;
static const char BLACK_AREA = 3;
static const char UNKNOWN_AREA = 0;

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

static volatile long polling_time = 0;
static volatile long colour_polling_time = 0;
static volatile long last_colour_polling_time = 0;

// our RGB -> eye-recognized gamma color
byte gammatable[256];

static uint16_t red = 0;
static uint16_t green = 0;
static uint16_t blue = 0;

static const uint16_t BLUE_R_LOW = 885;
static const uint16_t BLUE_R_HIGH= 893;
static const uint16_t BLUE_G_LOW = 994;
static const uint16_t BLUE_G_HIGH = 995;
static const uint16_t BLUE_B_LOW = 775;
static const uint16_t BLUE_B_HIGH = 778;

static const uint16_t GREEN_R_LOW = 902;
static const uint16_t GREEN_R_HIGH = 903;
static const uint16_t GREEN_G_LOW = 942;
static const uint16_t GREEN_G_HIGH = 943;
static const uint16_t GREEN_B_LOW = 566;
static const uint16_t GREEN_B_HIGH = 567;

static const uint16_t BLACK_R_LOW = 807;
static const uint16_t BLACK_R_HIGH = 811;
static const uint16_t BLACK_G_LOW = 805;
static const uint16_t BLACK_G_HIGH = 807;
static const uint16_t BLACK_B_LOW = 512;
static const uint16_t BLACK_B_HIGH = 516;

static const uint16_t COLOUR_TOLERANCE = 30;

static int location = UNKNOWN_AREA;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

void setup() {
  motor_left.attach(3);
  motor_right.attach(2);
  drive();
  Serial.begin(9600);
}

void loop() {
  if (colour_polling_time >= 700) {
    colour_polling_time = 0;
    read_colour_sensor();
    check_home();
    if ((location == BLUE_HOME) || (location == GREEN_HOME) && (last_colour_polling_time >= 1100)) {
      turn_right();
      last_colour_polling_time = 0;
      delay(16000);
    }
  }
  if (polling_time >= 400) {
    polling_time = 0;
    poll_sensors();
  }
  if (weight_sensed) {
    motor_stop();
    drive();
    while (!weight_found && !wall_found) {
      weight_polling();
    }
    while(weight_found && !wall_found) {
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
  colour_polling_time++;
  last_colour_polling_time++;
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
  }
  else if ((IRShortdistance != -1) && (IRShortdistance <= IR_weight_found_dist) && (IRLongdistance > (IR_weight_found_dist + tolerance))) {
    weight_found = true;
  }
}

void read_colour_sensor(void) {
  red, green, blue = 0;
  uint16_t clear;
  tcs.setInterrupt(false);      // turn on LED
  delay(52);  // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);  // turn off LED
}

void check_home(void) {
  if((red >= (BLUE_R_LOW - COLOUR_TOLERANCE)) && (red <= (BLUE_R_HIGH + COLOUR_TOLERANCE)) && ((blue >= (BLUE_B_LOW - COLOUR_TOLERANCE)) && (blue <= (BLUE_B_HIGH + COLOUR_TOLERANCE))) && ((green >= (BLUE_G_LOW - COLOUR_TOLERANCE)) && (green <= (BLUE_G_HIGH + COLOUR_TOLERANCE)))) {
    location = BLUE_HOME;
  }
  else if((red >= (GREEN_R_LOW - COLOUR_TOLERANCE)) && (red <= (GREEN_R_HIGH + COLOUR_TOLERANCE)) && ((blue >= (GREEN_B_LOW - COLOUR_TOLERANCE)) && (blue <= (GREEN_B_HIGH + COLOUR_TOLERANCE))) && ((green >= (GREEN_G_LOW - COLOUR_TOLERANCE)) && (green <= (GREEN_G_HIGH + COLOUR_TOLERANCE)))) {
    location = GREEN_HOME;
  }
  else if((red >= BLACK_R_LOW) && (red <= BLACK_R_HIGH) && ((blue >= BLACK_B_LOW) && (blue <= BLACK_B_HIGH)) && ((green >= BLACK_G_LOW) && (green <= BLACK_G_HIGH))) {
    location = BLACK_AREA;
  }
  else {
    location = UNKNOWN_AREA;
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
