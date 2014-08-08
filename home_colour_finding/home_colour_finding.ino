#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>

#define DRIVE_SPEED 135
#define STOP_SPEED 90
#define K_P 2
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

Servo motor_left;
Servo motor_right;

int motor_left_speed = 90;
int motor_right_speed = 90;

//static byte USpin = 8;
static byte IR_long_pin = 8;
static byte LeftGP2D12pin = 10;
static byte RightGP2D12pin = 6;

int US_sensing_dist = 40;
int IR_sensing_dist = 40;
static int target_distance = 25;
static int IR_long_sensing_dist = 40;

boolean wall_left = false;
boolean wall_right = false;
boolean wall_foward = false;

static volatile long polling_time = 0;
static volatile long colour_polling_time = 0;

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
    /*pinMode(DIGITAL_OUT_POWER, OUTPUT); 
  digitalWrite(DIGITAL_OUT_POWER, 1);//IO Power should be off by default*/
  Serial.begin(9600);
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
      
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;      
    }
  }
  motor_left.attach(3);
  motor_right.attach(2);
  drive();
}

void loop() {
  if (colour_polling_time >= 3500) {
    colour_polling_time = 0;
    read_colour_sensor();
    check_home();
    if ((location == BLUE_HOME) || (location == GREEN_HOME)) {
      turn_right();
      delay(10000);
    }
  }
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
  /*if (wall_foward && !wall_left && !wall_right) {
    reverse();
    turn_left();
  }*/
  if (!wall_foward && !wall_left && !wall_right) {
    drive();
  }
  polling_time++;
  colour_polling_time++;
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
  //USdistance = read_ul_sensor_range(USpin);
  IRLongdistance = read_IR_long_range(IR_long_pin);
  IRLeftdistance = read_gp2d12_range(LeftGP2D12pin);
  IRRightdistance = read_gp2d12_range(RightGP2D12pin);
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
  motor_left.write(40);
  motor_right.write(140);
}

void turn_right(void) {
  motor_left.write(140);
  motor_right.write(40);
}

void reverse(void) {
  motor_left.write(45);
  motor_right.write(45);
}

void drive(void) {
  motor_left.write(DRIVE_SPEED);
  motor_right.write(DRIVE_SPEED);
} 
