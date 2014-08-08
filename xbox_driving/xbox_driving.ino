/* Driving with panning GP2D12 and ultrasonic sensors to follow walls
*/

#include <Servo.h>
#include <XBOXUSB.h>

#define DRIVE_SPEED 135
#define STOP_SPEED 90
#define K_P 3


float USdistance = 0;
float IRLeftdistance = 0;
float IRRightdistance = 0;

Servo motor_left;
Servo motor_right;
Servo arm;

int motor_left_speed = 90;
int motor_right_speed = 90;
int arm_position = 90;

static byte USpin = 8;
static byte LeftGP2D12pin = 10;
static byte RightGP2D12pin = 6;

int US_sensing_dist = 50;
int IR_sensing_dist = 35;
static int target_distance = 55;

boolean wall_left = false;
boolean wall_right = false;
boolean wall_foward = false;

static long polling_time = 0;

//Defining Variables
boolean leftside_open = false; //detects open space while turning left
boolean rightside_open = false; //detects open space while turning right
boolean get_weight = false; //instructs the robot that there is a weight to be picked up
int detection_dist = 35; //defines the distance at which the differentiating process occurs
int tolerance = 10; //defines the tolerance allowed for the change in US_dist value while turning side to side 
int pickup_dist = 35; //define the distance from the weight at which the robot must be positioned to pickup the weight 
int pickup_angle = 0; //defines the angle of the arm at which the weight should be picked at
int rest_angle = 0; //defines the angle of the arm at which it rests 
int angle = 0; //the value of the current angle of the arm
static volatile long leftside_counter = 0;
static volatile long rightside_counter = 0;
static volatile long turning_counter = 0;
boolean object_detected = false;

#include <XBOXUSB.h>
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
XBOXUSB Xbox(&Usb);

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
  motor_left.attach(3);
  motor_right.attach(2);
  arm.attach(8);
}
void loop() {
  
  /*if (polling_time >= 800) {
    polling_time = 0;
    poll_sensors();
  }
  /*if (!wall_foward) {
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
  drive();
  delay(200);
  polling_time++;
  if (USdistance <= detection_dist){ //if a object/wall is detected, it determines whether it is a wall or an object
    single_object_detection();
  }
  if (leftside_open || rightside_open){ //if there was an open space on either side of the object/wall, it is an object
    object_detected == true;
    align_robot(); //move the robot so the arm is in line with the object
  }
  if (get_weight){
    move_to_weight(); //move to the weight
    pickup(); //pick up the weight
  }
  if (object_detected) {
    motor_stop();
    while(1) {
    }
  }
  turn_left();
  delay(100);
  */
  Usb.Task();
  if (Xbox.Xbox360Connected) {
  if (Xbox.getButtonClick(UP)) {
      drive();
      Xbox.setLedOn(LED1);
    }
    if (Xbox.getButtonClick(DOWN)) {
      reverse();
    }
    if (Xbox.getButtonClick(LEFT)) {
      turn_left();
    }
    if (Xbox.getButtonClick(RIGHT)) {
      turn_right();
    }
    if (Xbox.getButtonClick(BACK)) {
      motor_stop();
      arm_position = 90;
    }
    if (Xbox.getButtonClick(B)) {
      arm.write(90);
    }
    if (Xbox.getButtonClick(A)) {
      arm.write(110);
    }
    if (Xbox.getButtonClick(X) && (arm_position > 0)) {
      arm.write(arm_position);
      arm_position--;
    }
    if (Xbox.getButtonClick(Y) && (arm_position < 180)) {
      arm.write(arm_position);
      arm_position++;
    }
  }
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
  motor_left.write(45);
  motor_right.write(135);
}

void turn_right(void) {
  motor_left.write(135);
  motor_right.write(45);
}

void reverse(void) {
  motor_left.write(45);
  motor_right.write(45);
}

void drive(void) {
  motor_left.write(DRIVE_SPEED);
  motor_right.write(DRIVE_SPEED);
} 

void motor_stop(void) {
  motor_left.write(90);
  motor_right.write(90);
}

//function to differentiate weights from walls
void single_object_detection(){ 
  turn_left();
  while (!leftside_open){ //this may require more variables to define the end of turning ###
  leftside_counter++;
  poll_sensors();
    if (USdistance > detection_dist + tolerance){ //if there is an open space (dist value changes to a large value)
      leftside_open = true;  
    }
  }
    
  turn_right();
  while (!rightside_open){ //this may require more variables to define the end of turning ###
  rightside_counter++;
  poll_sensors();
    if (USdistance > detection_dist + tolerance){ //if there is an open space (dist value changes to a large value)
      rightside_open = true;  
    }
  } 
}

//function to move the robot to align the arm to the weight
void align_robot(){
  if (leftside_open){
    turning_counter = 0;
    turn_left();
    while (USdistance <= detection_dist){ //turn left until the weight moves out of the field of detection
      poll_sensors();
    } 
    turn_right();
    while (turning_counter < (rightside_counter/2)) {
      turning_counter++;
      poll_sensors();
    }
    motor_stop();
  }
  else if (rightside_open){
    turning_counter = 0;
    turn_right();
    while (USdistance <= detection_dist){ //turn right until the weight moves out of the field of detection
    poll_sensors();
    } 
    turn_left();    
    while (turning_counter < (rightside_counter/2)) {
      poll_sensors();
      turning_counter++;
    }
  }
  leftside_open = false; //reset the record
  rightside_open = false; //reset the record
  turning_counter = 0;
  get_weight = true;
}

//function to position the robot to the right spot to pick up the weight
void move_to_weight(){
  //!!!the following process could be set up using na PID control to locate the robot
  drive();
  while (USdistance > pickup_dist){ //until it reaches the right distance go forward
    poll_sensors();
  }
  if (USdistance <= 30){ //if overshoot (moved too close so the value of sensor is 30)
    reverse();
    while (USdistance < pickup_dist){ //move back little bit until the right distance is reached
      poll_sensors();
    }
  }
  motor_stop();
}

//function to pick up the weight
void pickup(){
  //!!!the following process could be set up using na PID control to locate the robot
  //move arm to the weight (lower the arm)
  while (angle > pickup_angle){ //until it reaches the pickup angle
    /*###LOWER THE ARM, DECREASE SERVO ANGLE*/
  }
  //move the arm back to resting position
  while (angle < rest_angle){ //until it reaches the resting angle
    /*###RAISE THE ARM, INCREASE SERVO ANGLE*/
  }
  get_weight = false; //stop the instruction to go get the weight
}
