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
  bool weight_located = false;
  bool homing_state = false;
  bool start_of_homing = false;
  bool collection_state = false;
  bool new_weight = false;
  
  
    
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

void setup() {
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  motor_left.attach(3);
  motor_right.attach(2);
  //Serial.begin(9600);
  delay(4000);
  //Serial.println("STARTING CODE");
}



void loop() {
  //keep a count of cycles to record the angle rotated (this value must be derived from experiments to get the right conversion value which accounts for rotating speed and system clock speed - may want to use interrupts for consistency)
  
  
  
  //encoder step difference between the two sides converted to an angle rotated
  while (scan_state) {
    //Rotate 360 degrees clockwise
    //rotate(360, true, current_angle); //clockwise
    turn_right();
    //conversion of upper IR dist to match lower IR dist
    forward_IR_check();
    if (abs(difference_dist) > tolerance_upper_lower){ //if the distance from the upper and bottom sensors are different (potentially a weight)
      initial_angle = current_angle;
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
    if (start_of_homing){
      current_angle = 0;
      start_of_homing = false;
    }
    //rotate(weight_position, true, current_angle);
    while (!collection_state){
      move_to_weight();
      forward_IR_check();
      if (difference_dist < tolerance_upper_lower){              //7 FOR NOW
        turn_left();
        delay(10);
        motor_stop();
      }
      homing_state = false;
    }
  
  while (collection_state){
    pick_up();
  }
  
  
    
    
    //if the distance from the upper and bottom sensors are different (potentially a weight)
          //calculate the change in angle in the last consistent distance (single object) (current angle - last angle [last angle is 0 at start of rotation])
            //the width will be calculated based on last consistent distance from the sensor and the angle moved (2*pi*dist) approximated to be circumference identical from the sensor (could implement a more accurate and consistent mathematical model)
            
            //if the width is within tolerance values of the expected widths of a weight (need to derive value from experiments)
             //store the position of the weight in a detected weight array (should be a grid for future applications) [keep record of the number of weights found to determine index to store the next weight] [resize the array everytime maybe]
               //position = last angle + half of the change in angle - this stores the middle of the weight so makes it easier for homing in 
           
  //If weights are found
    //rotate to the stored angle and go to the weight (for now angle is used as position and robot just goes straight to it)
    
    //home in to weight
      //position the robot at the required distance (PD control maybe to exactly get to the position with tolerance) (derive effective number from experiments)
      
        
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
  lower_IR_dist = read_short_IR (GP2D120pin);
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
  else if (lower_IR_dist <= 4 || (lower_IR_dist == -1)){       //THIS WILL GET DELETED
    motor_stop();
    collection_state = true;
  }
}

void forward_IR_check(void){
  upper_IR_dist = read_long_IR (LongIRpin);
  lower_IR_dist = read_short_IR (GP2D120pin);
  upper_IR_conv_dist = upper_IR_dist - 16; //16 for now but change to more accurate number later
  /*Serial.println("FORWARD IR TESTING");
  Serial.print("Upper IR dist: ");
  Serial.println(upper_IR_dist, DEC);
  Serial.print("Lower IR dist: ");
  Serial.println(lower_IR_dist, DEC);
  Serial.print("Converted upper dist: ");
  Serial.println(upper_IR_conv_dist, DEC);*/
  difference_dist = (upper_IR_conv_dist - lower_IR_dist);
  if ((upper_IR_dist == 0) || (lower_IR_dist == 0)) {
    difference_dist = 0;
  }
  /*Serial.print("Difference Dist: ");
  Serial.println(difference_dist, DEC);
  delay(500);*/
}


int read_left_IR (byte pin) {
  while (GP2D12LeftdataIndex < 5) {
    GP2D12Leftdata[GP2D12LeftdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Leftdist = filterSum(GP2D12Leftdata)/GP2D12LeftdataIndex;
  GP2D12LeftdataIndex = 0;
  return GP2D12Leftdist;
}

int read_right_IR (byte pin) {
  while (GP2D12RightdataIndex < 5) {
    GP2D12Rightdata[GP2D12RightdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Rightdist = filterSum(GP2D12Rightdata)/GP2D12RightdataIndex;
  GP2D12RightdataIndex = 0;
  return GP2D12Rightdist;
}

int read_short_IR (byte pin) {
  while (GP2D120dataIndex < 5) {
    GP2D120data[GP2D120dataIndex++] = read_gp2d120_range(pin);
  }
  GP2D120dist = filterSum(GP2D120data)/GP2D120dataIndex;
  GP2D120dataIndex = 0;
  return GP2D120dist;
}

int read_long_IR (byte pin) {
  while (LongIRdataIndex < 5) {
    LongIRdata[LongIRdataIndex++] = read_IR_long_range(pin);
  }
  LongIRdist = filterSum(LongIRdata)/LongIRdataIndex;
  LongIRdataIndex = 0;
  return LongIRdist;
}

int filterSum(int* dataArray) {
  int i;
  int sum = 0;
  for (i = 0; i < numSamples; i++) {
    if (*dataArray != -1) {
      sum += *dataArray++;
    }
  }
  if (sum == 0) {
    sum = -1;
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
  

void pick_up(){
  arm.move_arm(300);
  delay(200);
  arm.arm_dir_up();
  arm.move_arm(400);
  rack_pinion.move_RP(950);
  arm.arm_dir_down();
  arm.move_arm(300);
  delay(1500);
  arm.arm_dir_up();
  arm.move_arm(300);
  rack_pinion.RP_dir_backward();
  rack_pinion.move_RP(900);
  /*while (1) {
    delay(1000);
  }*/
  arm.arm_dir_down();
  arm.move_arm(300);
  delay(500);
  
  arm.arm_dir_down();
  arm.move_arm(200);
  arm.arm_dir_up();
  arm.move_arm(200);
  
  scan_state = true;
  current_angle = 0;
  upper_IR_dist = 0;
  lower_IR_dist = 0;
  upper_IR_conv_dist =
  difference_dist = 0;
  initial_angle = 0;
  final_angle = 0;
  weight_position = 0;
  weight_located = false;
  homing_state = false;
  start_of_homing = false;
  collection_state = false;
  new_weight = false;
  
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  
  delay(2000);
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
