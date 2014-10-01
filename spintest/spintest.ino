//Weight detection and homing code for funtional assessment 2
//Simple rotating detection method and a basic straight homing in method
//Need to implement grid position storage(able to collect the next weight based on position of the first weight) and navigation (able to manuever around obstacles)

#include "rack_pinion.h"
#include "arm.h"


#include <Servo.h>

#define DRIVE_SPEED 155
#define STOP_SPEED 90
#define REVERSE_SPEED 25
#define TURN_WHEEL_SLOW 25
#define TURN_WHEEL_FAST 155

#define REVOLUTION_DIST 219  //Distance, in mm, covered in one rev of wheel with tracks (measured; could be up to 219 mm)
#define MAX_ENCODER_VAL 1325
#define ANGLE_SCALE 67

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


//global variables
  bool scan_state = true;
  int current_angle = 0;
  int upper_IR_dist = 0;
  int lower_IR_dist = 0;
  int upper_IR_conv_dist = 0;
  int difference_dist = 0;
  int tolerance_upper_lower = 5;
  int initial_angle = 0;
  int final_angle = 0;
  int weight_position = 0;
  int temp_IR_val = 0;
  bool weight_located = false;
  bool homing_state = false;
  bool start_of_homing = false;
  bool collection_state = false;
  bool new_weight = false;
  bool wall_state = false;
  
  
  
    
const int numSamples = 5;

int GP2D120data[numSamples] = {0};
int GP2D120dataIndex = 0;
int GP2D120dist = 0;

int GP2D12Leftdata[numSamples] = {0};
int GP2D12LeftdataIndex = 0;
int GP2D12Leftdist = 0;

int GP2D12Rightdata[numSamples] = {0};
int GP2D12RightdataIndex = 0;
int GP2D12Rightdist = 0;

int LongIRdata[numSamples] = {0};
int LongIRdataIndex = 0;
int LongIRdist = 0;

int GP2D120pin = 1;
int GP2D12Leftpin = 10;
int GP2D12Rightpin = 0;
int LongIRpin = 2;
/*
GP2D120 on pin 1
Front Left GP2S12 on pin 10
Front Right GP2D12 on pin 0
GP2Y0A02YK on pin 2
*/

int Armdirpin = 31;
int Armsteppin = 30;
int RPdirpin = 35; 
int RPsteppin = 34;
  
RP rack_pinion(RPdirpin, RPsteppin);
Arm arm(Armdirpin, Armsteppin);

int limit_switch_pin = A8;
int limit_switch_reading = LOW;

int magnet_control_pin = A6;

int left_IR_dist = 0;
int right_IR_dist = 0;
bool left_obstacle_cruising;
bool right_obstacle_cruising;
bool front_obstacle_cruising;
bool left_obstacle;
bool right_obstacle;
int last_turn = 0;
int side_dist_difference = 0;
int interval_count = 0;

unsigned volatile long time = 0;
unsigned volatile long prev_time = 0;

bool first_start = true;

void setup() {
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  pinMode(magnet_control_pin, OUTPUT);
  pinMode(limit_switch_pin, INPUT);
  motor_left.attach(3);
  motor_right.attach(2);
  delay(4000);
  //Serial.println("STARTING CODE");
}



void loop() {
  time = millis();
  // put your main code here, to run repeatedly:
  //keep a count of cycles to record the angle rotated (this value must be derived from experiments to get the right conversion value which accounts for rotating speed and system clock speed - may want to use interrupts for consistency)
  if (first_start){ //only do at initial boot up
    //IR_check();
    //for (i=0; i<10; i++){ //more stable condition by checking more than one instance
    //  if (upper_IR_dist == -1) { //better more versatile condition ?? more specific
        wall_state = true;
    //  }
    //}
    delay(50);
    first_start = false;
  }
  
  while (wall_state) {
    
    IR_check();
    
    left_obstacle_cruising = ((left_IR_dist < 12)&&(left_IR_dist > 0));
    right_obstacle_cruising = ((right_IR_dist < 12)&&(right_IR_dist > 0));
    front_obstacle_cruising = ((upper_IR_dist < 50)&&(upper_IR_dist > 0));
    left_obstacle = ((left_IR_dist < 30)&&(left_IR_dist > 0));
    right_obstacle = ((right_IR_dist < 30)&&(right_IR_dist > 0));
    
    //for front wall 
    //(want the front distance to be minimal as possible while still being able to not crash into walls infront - back and front of the robot) 
      //(depends on the distance between the front sensor and the side of the robot and the sharpness of the turn the robot can make - experiment)
      //set it as the distance between the front sensor and the sides of the robot for now
    
    if (front_obstacle_cruising) { //for front wall
      //moving direction
      //if object on one side, move away from the closer side
      if (left_obstacle && right_obstacle){
        while (left_obstacle && right_obstacle){ //until only one side has a wall, or neither side has a wall
          reverse();
          IR_check();
          left_obstacle = ((left_IR_dist < 30)&&(left_IR_dist > 0));
          right_obstacle = ((right_IR_dist < 30)&&(right_IR_dist > 0));
        }
        if (left_obstacle){ //determined by side sensor distance detection being less than a set value - modify for later code
          turn_right();
          delay (2000); //90_degreesfor now //might change angle for later code if need be
        }
        else if (right_obstacle){
          turn_left();
          delay (2000); //90_degrees
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
        delay(200); //just to avoid blindspot for functional assessment 2. Take this out later
      }
      else if (right_obstacle_cruising){
        turn_left(); //until (right_sensor_dist > 12) wall follow for now(in the final code consider turning towards wanted path direction)
        delay(200); //just to avoid blindspot for functional assessment 2. Take this out later
      }
      else {
        //move forward if no obstacles
        motor_drive();
        
        
        
        
        
        time = millis();
        if (!left_obstacle && !right_obstacle){
          time = millis();
          //interval_count++; //keeps count when there is no obstacles around the robot
        }
        else {
          prev_time = time;
        }
        
        if ((time - prev_time) > 2000) {//when the robot has been in peace for a while, go into weight search mode (do some calc to determine the count number)
        //Search and pick up weight every once in awhile if there are no walls nearby
          //interval_count = 0; //reset the count
          time = millis();
          prev_time = time;
          scan_state = true;
          
          //encoder step difference between the two sides converted to an angle rotated
          while (scan_state) {
            //Rotate 360 degrees clockwise
            //rotate(360, true, current_angle); //clockwise
            turn_right();
            //conversion of upper IR dist to match lower IR dist
            IR_check();
            if (abs(difference_dist) > tolerance_upper_lower){ //if the distance from the upper and bottom sensors are different (potentially a weight)
              //initial_angle = current_angle;
              new_weight = true;  
              homing_state = true;
              scan_state = false;
              
            }
            /*
            else {   
              if (new_weight) {
                do_calc = true;
                new_weight = false;  
                final_angle = current_angle;
              }
            }
            if (do_calc){
              weight_position = (final_angle-initial_angle)/2 + initial_angle;
              do_calc = false;
              weight_located = true;
            }
            if (weight_located) {
              homing_state = true; 
              start_of_homing = true;
            }
           */
          }
           
          
          while (homing_state){
            /*if (start_of_homing){
              current_angle = 0;
              start_of_homing = false;
            }
            */
            //rotate(weight_position, true, current_angle);
            while (!collection_state){
              IR_check();
              move_to_weight();
              
              if (abs(difference_dist) > tolerance_upper_lower){              //7 FOR NOW
                turn_left();
                delay(10);
                motor_stop();
              }
              homing_state = false;
            }
          }
          time = millis();
            pick_up();
        }
      }
    }
  }    
}



void rotate(int angle, bool clockwise, int current_angle){
  int angle_error = angle - current_angle;
  
  if ((clockwise && (angle_error > 3))||(!clockwise && (angle_error <-3))) {
    //turn_clock();    //Control the speed of turning based on the error maybe? for more accurate control of rotation angle
  }
  else if ((!clockwise && (angle_error >3))||(clockwise && (angle_error <-3))) {
    //turn_anticlock(); //function
  }
  else {
    motor_stop(); //function
  }
}

void move_to_weight(void){
  if (lower_IR_dist > 5){ //need stuff here
    motor_drive();
  }
  /*
  else if (lower_IR_dist <= 4 || (lower_IR_dist == -1)){   THIS IS FOR LATER WILL NEED ENCODER CHANGE
     reverse(); //MIGHT NEED NAME CHANGE 
  }
  else{
    motor_stop();
    collection_state = true;
  }
  */
  else if (lower_IR_dist <= 5){       //THIS WILL GET DELETED
    motor_stop();
    collection_state = true;
  }
}

void IR_check(void){
  upper_IR_dist = read_long_IR (LongIRpin);
  lower_IR_dist = read_short_IR (GP2D120pin);
  left_IR_dist = read_left_IR(GP2D12Leftpin);
  right_IR_dist = read_right_IR(GP2D12Rightpin);
  if (upper_IR_dist != -1) {
    upper_IR_conv_dist = upper_IR_dist - 16; //16 for now but change to more accurate number later
  }
  else {
    upper_IR_conv_dist = -1;
  }
  //problem when upper_IR_dist == -1 or anything below 16
  
  /*Serial.println("FORWARD IR TESTING");
  Serial.print("Upper IR dist: ");
  Serial.println(upper_IR_dist, DEC);
  Serial.print("Lower IR dist: ");
  Serial.println(lower_IR_dist, DEC);
  Serial.print("Converted upper dist: ");
  Serial.println(upper_IR_conv_dist, DEC);*/
  if ((upper_IR_dist == -1) || (lower_IR_dist == -1)) {
    difference_dist = 0; //just a number below the chosen tolerance
  }
  else {
    difference_dist = (upper_IR_conv_dist - lower_IR_dist);
  }
  /*Serial.print("Difference Dist: ");
  Serial.println(difference_dist, DEC);
  delay(500);*/
}


int read_left_IR (byte pin) {
  while (GP2D12LeftdataIndex < 5) {
    GP2D12Leftdata[GP2D12LeftdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Leftdist = filterSum(GP2D12Leftdata, &GP2D12LeftdataIndex);
  GP2D12Leftdist = GP2D12Leftdist/GP2D12LeftdataIndex;
  GP2D12LeftdataIndex = 0;
  return GP2D12Leftdist;
}

int read_right_IR (byte pin) {
  while (GP2D12RightdataIndex < 5) {
    GP2D12Rightdata[GP2D12RightdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Rightdist = filterSum(GP2D12Rightdata, &GP2D12RightdataIndex);
  GP2D12Rightdist = GP2D12Rightdist/GP2D12RightdataIndex;
  GP2D12RightdataIndex = 0;
  return GP2D12Rightdist;
}

int read_short_IR (byte pin) {
  while (GP2D120dataIndex < 5) {
    GP2D120data[GP2D120dataIndex++] = read_gp2d120_range(pin);
  }
  GP2D120dist = filterSum(GP2D120data, &GP2D120dataIndex);
  GP2D120dist = GP2D120dist/GP2D120dataIndex;
  GP2D120dataIndex = 0;
  return GP2D120dist;
}

int read_long_IR (byte pin) {
  while (LongIRdataIndex < 5) {
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
  //range = (9462.0 /((float)tmp - 16.92));
  //range = (9900.0 /((float)tmp - 16.92));
  range = 30431 * pow(float(tmp), -1.169);
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
  if ((tmp < 6) || (range > 35)) {
    range =  -1; // Error value
  }
  return range;
}
  

void pick_up(void) {
  arm.arm_dir_down();
  rack_pinion.RP_dir_forward();
  arm.move_arm(300);
  digitalWrite(magnet_control_pin, HIGH);
  delay(200);
  
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
  
  
  arm.arm_dir_down();
  arm.move_arm(100);
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
