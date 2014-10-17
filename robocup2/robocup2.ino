#include "rack_pinion.h"
#include "arm.h"
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

// COLOUR SENSOR
#define commonAnode true
byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);

typedef enum ColourPos {
  BLUE_HOME,
  GREEN_HOME,
  BLACK_AREA,
  AREA_UNKNOWN
};

static uint16_t red = 0;
static uint16_t green = 0;
static uint16_t blue = 0;
static uint16_t colour_sum = 0;
#define BLACK_SUM 1416
static uint16_t BLACK_RED = 0;
static uint16_t BLACK_GREEN = 0;
static uint16_t BLACK_BLUE = 0;
static uint16_t COLOUR_SUM = 0;
static const uint16_t COLOUR_TOLERANCE = 80;
static ColourPos location = AREA_UNKNOWN;

#define DRIVE_SPEED 155
#define STOP_SPEED 90
#define REVERSE_SPEED 25
#define TURN_WHEEL_SLOW 25
#define TURN_WHEEL_FAST 155
#define TURN_WHEEL_SLOWEST 55
#define TURN_WHEEL_SLOWER 125
#define DRIVE_SPEED_SLOW 120

typedef enum Direction{
  forward,
  motor_backwards,
  left,
  right,
  stopped
};
Servo motor_left;
Servo motor_right;
int motor_left_speed = 90;
int motor_right_speed = 90;
Direction robot_dir = stopped;

int DAV_LED_PIN1 = 38;
int DAV_LED_PIN2 = 39;
int DAV_LED_PIN3 = 40;
int DAV_LED_PIN4 = 41;
int DAN_LED_PINS = A4;

int upper_IR_dist = 0;
int lower_IR_dist = 0;
int upper_IR_conv_dist = 0;
int difference_US_dist = 0;
int difference_IR_dist = 0;
int tolerance_upper_lower = 10;
unsigned int min_consec_weight_detects = 6;
bool weight_located = false;
bool homing_state = false;
bool start_of_homing = false;
bool collection_state = false;
bool new_weight = false;
bool wall_state = false;

const int numSamples = 10;

int LowerIRdata[numSamples] = {0};
int LowerIRdataIndex = 0;
int LowerIRdist = 0;
int prev_lower_IR_dist = 0;

int GP2D12Leftdata[numSamples] = {0};
int GP2D12LeftdataIndex = 0;
int GP2D12Leftdist = 0;

int GP2D12Rightdata[numSamples] = {0};
int GP2D12RightdataIndex = 0;
int GP2D12Rightdist = 0;

int LongIRdata[numSamples] = {0};
int LongIRdataIndex = 0;
int LongIRdist = 0;

int ULdata[numSamples] = {0};
int ULdataIndex = 0;
int ULdist = 0;

int bottomIRpin = 1;
int GP2D12Leftpin = 10;
int GP2D12Rightpin = 0;
int topLongIRpin = 2;
int ULpin = 3;

static unsigned int bottomIRMinRange = 7;
static unsigned int bottomIRMaxRange = 70;

int Armdirpin = 33;
int Armsteppin = 32;
int RPdirpin = 35; 
int RPsteppin = 34;
  
RP rack_pinion(RPdirpin, RPsteppin);
Arm arm(Armdirpin, Armsteppin);

int limit_switch_pin = A8;
int limit_switch_reading = LOW;

int magnet_control_pin = A6;

int left_IR_dist = 0;
int right_IR_dist = 0;
int front_US = 0;
bool left_obstacle_cruising = false;
bool right_obstacle_cruising = false;
bool front_obstacle_cruising = false;
bool left_obstacle = false;
bool right_obstacle = false;
bool front_obstacle_US = false;
bool weight_found = false;
bool pickup_state = false;
bool refind_weight = false;
bool from_scan = false;
int last_turn = 0;
int last_turn_slow = 0;
int side_dist_difference = 0;
int consecutive_weight_detects = 0;

unsigned int time = 0;
unsigned int prev_weight_time = 0;
unsigned int prev_check_time = 0;
unsigned int prev_navigate_time = 0;
unsigned int prev_scan_time = 0;
unsigned int weight_loss_time = 0;
unsigned int tray_timeout = 0;
unsigned int turning_time = 0;

unsigned int tray_dir = 0;


void setup() {
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  pinMode(magnet_control_pin, OUTPUT);
  pinMode(limit_switch_pin, INPUT);
  pinMode(DAV_LED_PIN1, OUTPUT);
  pinMode(DAV_LED_PIN2, OUTPUT);
  pinMode(DAV_LED_PIN3, OUTPUT);
  pinMode(DAV_LED_PIN4, OUTPUT);
  pinMode(DAN_LED_PINS, OUTPUT);
  motor_left.attach(3);
  motor_right.attach(2);
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
  read_colour_sensor();
  IR_check();
  motor_drive();
  time = millis();
  prev_scan_time = time;
}

void loop() {
  time = millis();
  if ((time - prev_navigate_time) >= 100) {
    navigate();
    time = millis();
    prev_navigate_time = time;
  }
  check_weight();
  if (consecutive_weight_detects >= min_consec_weight_detects) {
    weight_found = true;
    digitalWrite(DAN_LED_PINS, HIGH);
    motor_stop();
    delay(10);
    consecutive_weight_detects = 0;
  }
  else if (((time - prev_check_time) >= 9000) && (!weight_found)) {
    scanning();
    time = millis();
    prev_check_time = time;
  }
  if (weight_found) {
    weight_found = false;
    homing();
  }
  limit_switch_reading = digitalRead(limit_switch_pin);  
  if ((limit_switch_reading == HIGH) && (tray_dir == 1)) {
    rack_pinion.RP_dir_backward();
    rack_pinion.move_RP(600);
  }
  else if ((limit_switch_reading == HIGH) && (tray_dir = 0)) {
    rack_pinion.RP_dir_forward();
    rack_pinion.move_RP(100);
  }
}

void scanning () {
  weight_found = false; //by default weight is not found yet by scanning
  
  IR_check();
  left_obstacle = ((left_IR_dist < 20)&&(left_IR_dist > 0));
  right_obstacle = ((right_IR_dist < 20)&&(right_IR_dist > 0));


  if (left_obstacle){ //if wall on left scan right
    scan_right();
  }
  else if (right_obstacle){ //if wall on right scan left
    scan_left();
  }
  else { //by default turn right
    scan_right();
  }
}

void scan_left(){
  reverse(); //allow for weights right infront of the robot to be detected during scanning
  delay(500);
  turn_left_slow();
  time = millis();
  turning_time = time;
  while (((time - turning_time) <= 3000) && (!weight_found)){
    time = millis();
    check_weight();
    if (consecutive_weight_detects >= min_consec_weight_detects) {
      weight_found = true;
      digitalWrite(DAN_LED_PINS, HIGH);
      motor_stop();
      delay(10);
      consecutive_weight_detects = 0;
      from_scan = true;
    }
  }
}

void scan_right(){
  reverse();
  delay(500);
  turn_right_slow();
  time = millis();
  turning_time = time;
  while (((time - turning_time) <= 3000) && (!weight_found)){
    time = millis();
    check_weight();
    if (consecutive_weight_detects >= min_consec_weight_detects) {
      weight_found = true;
      digitalWrite(DAN_LED_PINS, HIGH);
      motor_stop();
      delay(10);
      consecutive_weight_detects = 0;
      from_scan = true;
    }
  }    
}

void homing() {
  IR_check();
  time = millis();
  prev_weight_time = time;
  homing_state = true;
  while (!((lower_IR_dist < 10) && (lower_IR_dist > -1)) && homing_state) {
    IR_check();
    time = millis();
    //motor_drive();
    if (((time - prev_weight_time) >= 3000) && (difference_US_dist < tolerance_upper_lower)) { 
      homing_state = false;
      pickup_state = false;
    }
    if ((difference_IR_dist >= tolerance_upper_lower) && (difference_US_dist >= tolerance_upper_lower)){
      motor_drive();
      refind_weight = false;
      pickup_state = true;
    }
    else {
      refind_weight = true;
      pickup_state = false;
      time = millis();
      weight_loss_time = time;
    }
    if (refind_weight){
      time = millis();
      if ((time - weight_loss_time) < 1500){
        motor_stop();
        turn_left_slow();
      }
      else {
        motor_stop();
        turn_right_slow();
      }
    }
    
    /*if (lower_IR_dist == -1) {
      motor_stop();
      delay(50);
      turn_right_slow();
      delay(100);
      motor_stop();
      IR_check();
      if (lower_IR_dist == -1) {
        turn_left_slow();
        delay(200);
        motor_stop();
        IR_check();
        if (lower_IR_dist == -1) {
          homing_state = false;
          motor_drive();
        }
        else if ((difference_IR_dist >=tolerance_upper_lower) && (difference_US_dist >= tolerance_upper_lower)) {
          motor_drive();
        }
        else {
          homing_state = false;
          motor_drive();
        }
      }
      else if ((difference_IR_dist >=tolerance_upper_lower) && (difference_US_dist >= tolerance_upper_lower)) {
        motor_drive();
      }
      else {
        motor_drive();
      }*/
  }
  digitalWrite(DAN_LED_PINS, LOW);
  if (pickup_state) { //make this process more smart to ensure pickup of weights on sides
    if (from_scan){
      from_scan = false;
      motor_drive_slow();
      delay(500);
      turn_right();
      delay(75);
      motor_stop();
      pick_up();
      motor_drive();
    }
   else {
      motor_drive_slow();
      delay(400);
      turn_right();
      delay(55);
      motor_stop();
      pick_up();
      motor_drive();
   }
  }
}

void check_weight() {
  IR_check();
  if ((difference_IR_dist >=tolerance_upper_lower) && (difference_US_dist >= tolerance_upper_lower)) { //&& lower_IR_dist != -1) {
    consecutive_weight_detects++;
  }
  else {
    consecutive_weight_detects = 0;
  }
}

void navigate() {
  IR_check();
  left_obstacle_cruising = ((left_IR_dist < 12)&&(left_IR_dist > 0));
  right_obstacle_cruising = ((right_IR_dist < 12)&&(right_IR_dist > 0));
  front_obstacle_cruising = ((upper_IR_dist < 50)&&(upper_IR_dist > 0));
  left_obstacle = ((left_IR_dist < 20)&&(left_IR_dist > 0));
  right_obstacle = ((right_IR_dist < 20)&&(right_IR_dist > 0));
  front_obstacle_US = ((front_US < 40)&&(front_US > 0));
  
  if ((front_obstacle_cruising)||(front_obstacle_US)) { //for front wall
      //moving direction
      //if object on one side, move away from the closer side
    if (left_obstacle && right_obstacle){
      while (left_obstacle && right_obstacle){ //until only one side has a wall, or neither side has a wall
        reverse();
        IR_check();
        left_obstacle = ((left_IR_dist < 20)&&(left_IR_dist > 0));
        right_obstacle = ((right_IR_dist < 20)&&(right_IR_dist > 0));
      }
      if (left_obstacle){ //determined by side sensor distance detection being less than a set value - modify for later code
        turn_right();
        //delay (100); //90_degreesfor now //might change angle for later code if need be
      }
      else if (right_obstacle){
        turn_left();
       // delay (100); //90_degrees
      }
    }
    else if (right_obstacle){
      turn_left(); //until (front_upper_dist > 15);
      last_turn = 0; //keep record of the last direction of turning
    }
      //if object on both sides, move backwards until only one side (or no sides) detects an object then spin 90 degress away from the present object side (if no objects on both sides determine the direction of turn based on the overall wanted direction of path)
    else if (left_obstacle){ //determined by side sensor distance detection being less than a set value - modify for later code
      turn_right(); //until (front_upper_dist > 15) wall follow for now(in the final code consider turning towards wanted path direction)
      last_turn = 1; //keep record of the last direction of turning
    }
      
      //neither sides have a wall
    else { //if there is no objects on both sides, turn in the direction that points in the original wanted direction - store the wanted direction and the direction the robot has turned so far (FOR NOW WE USE CLOCKWISE)
      if (last_turn == 1) { //if last turning direction was clockwise
        turn_right(); //until (front_upper_dist > 15)
      }
      else { //if last turning direction was anticlockwise
        turn_left();
      }
    }
  }
  else { //for cruising past sides walls
    if (left_obstacle_cruising && right_obstacle_cruising){ //if the robot has walls on both sides very close to it, allign the robot and move forwards
      side_dist_difference = left_IR_dist - right_IR_dist;
      if (side_dist_difference > 1){ //if the robot is closer to the right wall
        turn_left(); //turn away from the right wall
      }
      else if (side_dist_difference < -1) { //if the robot is closer to the left wall
        turn_right(); //turn away from the left wall
      }
      else { //if the robot is placed in the middle
        motor_drive();
      }
    }
    else if (left_obstacle_cruising){ //if distance below 12cm move away til it is 12cm (minimum sensor distance 10cm)
      turn_right(); //until (left_sensor_dist > 12) wall follow for now(in the final code consider turning towards wanted path direction)
      //delay(100); //just to avoid blindspot for functional assessment 2. Take this out later
    }
    else if (right_obstacle_cruising){
      turn_left(); //until (right_sensor_dist > 12) wall follow for now(in the final code consider turning towards wanted path direction)
      //delay(100); //just to avoid blindspot for functional assessment 2. Take this out later
    }
    else {
      motor_drive();
    }
  }
  time = millis();
  if (left_obstacle || right_obstacle) {
    prev_scan_time = time;
  }
}

void motor_drive_slow(void) {
  motor_left.write(DRIVE_SPEED_SLOW);
  motor_right.write(DRIVE_SPEED_SLOW);
}
  

void turn_left(void) {
  motor_left.write(TURN_WHEEL_SLOW);
  motor_right.write(TURN_WHEEL_FAST);
  robot_dir = left;
}

void turn_right(void) {
  motor_left.write(TURN_WHEEL_FAST);
  motor_right.write(TURN_WHEEL_SLOW);
  robot_dir = right;
}

void reverse(void) {
  motor_left.write(REVERSE_SPEED);
  motor_right.write(REVERSE_SPEED);
  robot_dir = motor_backwards;
}

void motor_drive(void) {
  motor_left.write(DRIVE_SPEED);
  motor_right.write(DRIVE_SPEED);
  robot_dir = forward;
} 

void motor_stop(void) {
  motor_left.write(STOP_SPEED);
  motor_right.write(STOP_SPEED);
  robot_dir = stopped;
}

void turn_left_slow(void) {
  motor_left.write(TURN_WHEEL_SLOWEST);
  motor_right.write(TURN_WHEEL_SLOWER);
  robot_dir = left;
}

void turn_right_slow(void) {
  motor_left.write(TURN_WHEEL_SLOWER);
  motor_right.write(TURN_WHEEL_SLOWEST);
  robot_dir = right;
}

void IR_check(void){
  upper_IR_dist = read_long_IR (topLongIRpin);
  lower_IR_dist = read_lower_IR (bottomIRpin);
  left_IR_dist = read_left_IR(GP2D12Leftpin);
  right_IR_dist = read_right_IR(GP2D12Rightpin);
  front_US = read_US(ULpin);
  if (upper_IR_dist != -1) {
    upper_IR_dist -= 80/(upper_IR_dist); //Temp fix??
    //upper_IR_dist -= 4;
    upper_IR_conv_dist = upper_IR_dist - 13; //13 for now but change to more accurate number later
  }
  else {
    upper_IR_conv_dist = -1;
  }
  if ((lower_IR_dist > bottomIRMaxRange) || (lower_IR_dist < bottomIRMinRange)) {
    lower_IR_dist = -1;
  }
  if (lower_IR_dist == -1) {
    difference_IR_dist = 0; //just a number below the chosen tolerance
    difference_US_dist = 0;
  }
  else {
    difference_US_dist = ((front_US - 23) - lower_IR_dist);
    difference_IR_dist = upper_IR_conv_dist - lower_IR_dist;
  }
}

int read_left_IR (byte pin) {
  while (GP2D12LeftdataIndex < numSamples) {
    GP2D12Leftdata[GP2D12LeftdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Leftdist = filterSum(GP2D12Leftdata, &GP2D12LeftdataIndex);
  GP2D12Leftdist = GP2D12Leftdist/GP2D12LeftdataIndex;
  GP2D12LeftdataIndex = 0;
  return GP2D12Leftdist;
}

int read_lower_IR (byte pin) {
  while (LowerIRdataIndex < numSamples) {
    LowerIRdata[LowerIRdataIndex++] = read_gp2d12_range(pin);
  }
  LowerIRdist = filterSum(LowerIRdata, &LowerIRdataIndex);
  LowerIRdist = LowerIRdist/LowerIRdataIndex;
  LowerIRdataIndex = 0;
  return LowerIRdist;
}

int read_right_IR (byte pin) {
  while (GP2D12RightdataIndex < numSamples) {
    GP2D12Rightdata[GP2D12RightdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Rightdist = filterSum(GP2D12Rightdata, &GP2D12RightdataIndex);
  GP2D12Rightdist = GP2D12Rightdist/GP2D12RightdataIndex;
  GP2D12RightdataIndex = 0;
  return GP2D12Rightdist;
}

int read_US (byte pin) {
  while (ULdataIndex < numSamples) {
    ULdata[ULdataIndex++] = read_ul_sensor_range(pin);
  }
  ULdist = filterSum(ULdata, &ULdataIndex);
  ULdist = ULdist/ULdataIndex;
  ULdataIndex = 0;
  return ULdist;
}

int read_long_IR (byte pin) {
  while (LongIRdataIndex < numSamples) {
    LongIRdata[LongIRdataIndex++] = read_IR_long_range(pin);
  }
  LongIRdist = filterSum(LongIRdata, &LongIRdataIndex);
  LongIRdist = LongIRdist/LongIRdataIndex;
  LongIRdataIndex = 0;
  return LongIRdist;
}

int filterSum(int* dataArray, int* index) {
  int i;
  int sum = 0;
  for (i = 0; i < numSamples; i++) {
    if (*(dataArray + i) != -1) {
      sum += *(dataArray + i);
    }
    else {
      *index =- 1;
    }
  }
  if (sum == 0) {
    sum = -1;
    *index = 1;
  }
  return sum;
}

// This code seems to work fairly well, although the range is slightly off (close enough though)
float read_gp2d12_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (6787.0 /((float)tmp - 3.0)) - 4.0;
  if ((tmp < 3) || (range > 85)) {
    range =  -1; // Error value
  }
  return range;
}

float read_IR_long_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  if (pin == topLongIRpin) {
    range = (9462.0 /((float)tmp - 16.92)) + 12; //About 6cm too long at long range
  }
  else if (pin == bottomIRpin) {
    range = 30431 * pow(float(tmp), -1.169) + 1; //May want look-up table as pow slow
  }
  else {
    range = (9462.0 /((float)tmp - 16.92));
  }
  if (tmp <= 16.92) {
    range =  -1; // Error value
  }
  return range;
}

float read_gp2d120_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (2914.0 /((float)tmp + 3.0)) - 1.0;
  if ((tmp < 6) || (range > 30)) {
    range =  -1; // Error value
  }
  return range;
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

void read_colour_sensor(void) {
  red, green, blue, colour_sum = 0;
  //uint16_t clear;
  tcs.setInterrupt(false);      // turn on LED
  delay(52);  // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &colour_sum);
  tcs.setInterrupt(true);  // turn off LED
}

void pick_up(void) {
  digitalWrite(DAV_LED_PIN1, HIGH);
  digitalWrite(DAV_LED_PIN2, HIGH);
  digitalWrite(DAV_LED_PIN3, HIGH);
  digitalWrite(DAV_LED_PIN4, HIGH);
  arm.arm_dir_down();
  rack_pinion.RP_dir_forward();
  tray_dir = 1;
  arm.move_arm(600);
  digitalWrite(magnet_control_pin, HIGH);
  delay(200);
  
  arm.arm_dir_up();
  arm.move_arm(320);
  
  rack_pinion.move_RP(1000);
  delay(50);
  time = millis();
  tray_timeout = time;
  limit_switch_reading = digitalRead(limit_switch_pin);
  while (limit_switch_reading == LOW) {
    time = millis();
    rack_pinion.RP_dir_forward();
    tray_dir = 1;
    rack_pinion.move_RP(1000);
    limit_switch_reading = digitalRead(limit_switch_pin);
    if ((time - tray_timeout) >= 2000) {
      limit_switch_reading = HIGH;
    }
  }
  limit_switch_reading = digitalRead(limit_switch_pin);
  arm.arm_dir_down();
  arm.move_arm(90);
  digitalWrite(magnet_control_pin, LOW);
  
  arm.arm_dir_up();
  arm.move_arm(450);
  rack_pinion.RP_dir_backward();
  tray_dir = 0;
  rack_pinion.move_RP(1300);
  
  limit_switch_reading = digitalRead(limit_switch_pin);
  time = millis();
  tray_timeout = time;
  while (limit_switch_reading == HIGH) {
    time = millis();
    rack_pinion.RP_dir_backward();
    tray_dir = 0;
    rack_pinion.move_RP(600);
    delay(50);
    limit_switch_reading = digitalRead(limit_switch_pin);
    if (limit_switch_reading == HIGH) {
      rack_pinion.RP_dir_forward();
      tray_dir = 1;
      rack_pinion.move_RP(100);
    }
    if ((time - tray_timeout) >= 2000) {
      limit_switch_reading = LOW;
    }
  }
  digitalWrite(DAV_LED_PIN1, LOW);
  digitalWrite(DAV_LED_PIN2, LOW);
  digitalWrite(DAV_LED_PIN3, LOW);
  digitalWrite(DAV_LED_PIN4, LOW);
  time = millis();
  prev_check_time = time;
}
