//Weight detection and homing code for funtional assessment 2
//Simple rotating detection method and a basic straight homing in method
//Need to implement grid position storage(able to collect the next weight based on position of the first weight) and navigation (able to manuever around obstacles)

#include "rack_pinion.h"
#include "arm.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"


#include <Servo.h>

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

static const uint16_t BLACK_R_LOW = 519;
static const uint16_t BLACK_R_HIGH = 535;
static const uint16_t BLACK_G_LOW = 415;
static const uint16_t BLACK_G_HIGH = 436;
static const uint16_t BLACK_B_LOW = 260;
static const uint16_t BLACK_B_HIGH = 270;

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
  int difference_US_dist = 0;
  int difference_IR_dist = 0;
  int tolerance_upper_lower = 8;
  unsigned int min_consec_weight_detects = 4;
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

static unsigned int bottomIRMinRange = 8;
static unsigned int bottomIRMaxRange = 80;

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
int front_US = 0;
bool left_obstacle_cruising;
bool right_obstacle_cruising;
bool front_obstacle_cruising;
bool left_obstacle;
bool right_obstacle;
bool front_obstacle_US;
bool weight_found;
int last_turn = 0;
int side_dist_difference = 0;
int interval_count = 0;
int consecutive_weight_detects = 0;

volatile unsigned long time = 0;
volatile unsigned long prev_time = 0;
volatile unsigned long prev_colour_time = 0;
volatile unsigned long prev_scan_time = 0;

bool first_run = false;
bool enter_scan_state = true;
bool cant_find = false;
bool homing_start = false;
bool dont_pick_up;
bool first_start = true;

void setup() {
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  pinMode(magnet_control_pin, OUTPUT);
  pinMode(limit_switch_pin, INPUT);
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
  delay(2000);
  // NEED TO FIX THIS...
  /*read_colour_sensor();
  BLACK_RED = red;
  BLACK_GREEN = green;
  BLACK_BLUE = blue;*/
  //Serial.begin(9600);
}



void loop() {
  time = millis();
  // put your main code here, to run repeatedly:
  //keep a count of cycles to record the angle rotated (this value must be derived from experiments to get the right conversion value which accounts for rotating speed and system clock speed - may want to use interrupts for consistency)
  if (first_start){ //only do at initial boot up
    motor_drive();
    delay(1500);
    motor_stop();
    IR_check();
    read_colour_sensor();
    BLACK_RED = red;
    BLACK_GREEN = green;
    BLACK_BLUE = blue;
    COLOUR_SUM = colour_sum;
    //for (i=0; i<10; i++){ //more stable condition by checking more than one instance
    //  if (upper_IR_dist == -1) { //better more versatile condition ?? more specific
        wall_state = true;
    //  }
    //}
    motor_drive();
    first_start = false;
  }
  
  while (wall_state) {
    time = millis();
    IR_check();
    
    left_obstacle_cruising = ((left_IR_dist < 12)&&(left_IR_dist > 0));
    right_obstacle_cruising = ((right_IR_dist < 12)&&(right_IR_dist > 0));
    front_obstacle_cruising = ((upper_IR_dist < 50)&&(upper_IR_dist > 0));
    left_obstacle = ((left_IR_dist < 10)&&(left_IR_dist > 0));//30
    right_obstacle = ((right_IR_dist < 10)&&(right_IR_dist > 0));//30
    front_obstacle_US = ((front_US < 40)&&(front_US > 0));//50
    //for front wall 
    //(want the front distance to be minimal as possible while still being able to not crash into walls infront - back and front of the robot) 
      //(depends on the distance between the front sensor and the side of the robot and the sharpness of the turn the robot can make - experiment)
      //set it as the distance between the front sensor and the sides of the robot for now
    
    if ((front_obstacle_cruising)||(front_obstacle_US)) { //for front wall
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
        
        /*if (first_run) {
          prev_time = time;
          first_run = false;
        }*/
        
        
        
        time = millis();
        
        if (!left_obstacle && !right_obstacle){
          interval_count++; //keeps count when there is no obstacles around the robot
        }
        else {
          prev_scan_time = time;
          if (left_obstacle) {
          }
          else if (right_obstacle) {
          }
        }
        
        if (interval_count > 40) {//when the robot has been in peace for a while, go into weight search mode (do some calc to determine the count number)
        //if (time - prev_scan_time >= 1000) {
        //Search and pick up weight every once in awhile if there are no walls nearby
          interval_count = 0; //reset the count
          //time = millis();
          //prev_time = time;
          scan_state = true;
          enter_scan_state = true;
          
          //encoder step difference between the two sides converted to an angle rotated
          while (scan_state) {
            time = millis();
            if(enter_scan_state) {
              reverse();
              delay(500);
              prev_time = time;
              enter_scan_state = false;
            }
            if ((time - prev_time) > 7000){
              scan_state = false;
            }
            
            IR_check();
            //Rotate 360 degrees clockwise
            //rotate(360, true, current_angle); //clockwise
            
            turn_right();
            //conversion of upper IR dist to match lower IR dist
            if ((abs(difference_US_dist) > tolerance_upper_lower) && (abs(difference_IR_dist > tolerance_upper_lower))) { //if the distance from the upper and bottom sensors are different (potentially a weight)
            //check for consequtive difference values being above a tolerance to account for a slight mis allignment in the top and bottom sensors
              motor_stop();
              consecutive_weight_detects++; 
            }
            else {
              consecutive_weight_detects = 0;
              turn_right();
            }
            if (consecutive_weight_detects > min_consec_weight_detects){
              weight_found = true;
            }
            if (weight_found) {
              //initial_angle = current_angle;
              //new_weight = true;  
              
              motor_stop();
              weight_found = false; //reset
              homing_state = true;
              scan_state = false;
              
            }
          }
           
          
          while (homing_state){
            collection_state = false;
            homing_start = true;
            /*if (start_of_homing){
              current_angle = 0;
              start_of_homing = false;
            }
            */
            //rotate(weight_position, true, current_angle);
            while (!collection_state){
              IR_check();
              time = millis();            
            
              if (homing_start){
                prev_time = time;
                homing_start = false;
              }
              if ((time - prev_time) > 3000){
                cant_find = true;
              }
                
              if ((lower_IR_dist > 10)&&(difference_US_dist > tolerance_upper_lower)) { //need stuff here
                motor_drive();
                //delay(500);
              }
              else if ((lower_IR_dist < 0) || (difference_IR_dist < tolerance_upper_lower)) {
                turn_left();
                //delay(1);
                //motor_stop();
              }
              else if (lower_IR_dist <=10) {
                collection_state = true;
                dont_pick_up = false;
                cant_find = false;
              }
              
              if (cant_find) {
                collection_state = true;
                cant_find = false; //reset
                dont_pick_up = true;
              }
            }
            time = millis();
            prev_time = time;
            IR_check();
            bool not_found = false;
            while ((lower_IR_dist > 6) && !not_found) {
              //Serial.print("Lower dist: ");
              //Serial.println(lower_IR_dist, DEC);
              time = millis();
              IR_check();
              motor_drive();
              if ((time - prev_time) > 2000) {
                not_found = true;
                dont_pick_up = true;
              }
              if ((lower_IR_dist == -1) && (prev_lower_IR_dist < 10)) {
                not_found = true;
                dont_pick_up = false;
                reverse();
                delay(20);
              }
              prev_lower_IR_dist = lower_IR_dist;
            }
            //read_colour_sensor();
            //check_location();
            /*if ((location == BLACK_AREA)&&(!dont_pick_up)) {
              pick_up();
            }*/
            if (!dont_pick_up) {
              /*motor_drive();
              delay(50);*/
              turn_right();
              delay(40);
              motor_stop();
              pick_up();
            }
            else {
              reverse();
              delay(600);
              if (last_turn == 1) { //if last turning direction was clockwise
                turn_right(); //until (front_upper_dist > 15)
              }
              else { //if last turning direction was anticlockwise
                turn_left();
              }
              delay(900);
              motor_stop();
            }
            homing_state = false;
          }
        }
        time = millis();
        prev_scan_time = time;
        
      }
      //first_run = true;
    }
  }    
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
  /*else {
    lower_IR_dist += 3; //Temp fix, as top and bottom at different ranges
  }*/
  /*Serial.print("UPPER IR: ");
  Serial.println(upper_IR_dist, DEC);
  Serial.print("LOWER IR: ");
  Serial.println(lower_IR_dist);*/
  //problem when upper_IR_dist == -1 or anything below 16
  
  /*Serial.println("FORWARD IR TESTING");
  Serial.print("Upper IR dist: ");
  Serial.println(upper_IR_dist, DEC);
  Serial.print("Lower IR dist: ");
  Serial.println(lower_IR_dist, DEC);
  Serial.print("Converted upper dist: ");
  Serial.println(upper_IR_conv_dist, DEC);*/
  delay(100);
  if (lower_IR_dist == -1) {
    difference_IR_dist = 0; //just a number below the chosen tolerance
    difference_US_dist = 0;
  }
  else {
    difference_US_dist = ((front_US - 13) - lower_IR_dist);
    difference_IR_dist = upper_IR_conv_dist - lower_IR_dist;
    /*Serial.print("Difference IR is: ");
    Serial.println(difference_IR_dist, DEC);
    Serial.print("Difference US is: ");
    Serial.println(difference_US_dist, DEC);*/
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

/*int read_short_IR (byte pin) {
  while (GP2D120dataIndex < numSamples) {
    GP2D120data[GP2D120dataIndex++] = read_gp2d120_range(pin);
  }
  GP2D120dist = filterSum(GP2D120data, &GP2D120dataIndex);
  GP2D120dist = GP2D120dist/GP2D120dataIndex;
  GP2D120dataIndex = 0;
  return GP2D120dist;
}*/

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

void pick_up(void) {
  arm.arm_dir_down();
  rack_pinion.RP_dir_forward();
  arm.move_arm(600);
  digitalWrite(magnet_control_pin, HIGH);
  delay(200);
  
  arm.arm_dir_up();
  arm.move_arm(320);
  
  rack_pinion.move_RP(950);
  delay(50);
  limit_switch_reading = digitalRead(limit_switch_pin);
  while (limit_switch_reading == LOW) {
    rack_pinion.RP_dir_forward();
    rack_pinion.move_RP(950);
    limit_switch_reading = digitalRead(limit_switch_pin);
  }
  arm.arm_dir_down();
  arm.move_arm(90);
  digitalWrite(magnet_control_pin, LOW);
  
  arm.arm_dir_up();
  arm.move_arm(500);
  rack_pinion.RP_dir_backward();
  rack_pinion.move_RP(1300);
  
  limit_switch_reading = digitalRead(limit_switch_pin);
  while (limit_switch_reading == HIGH) {
    rack_pinion.RP_dir_backward();
    rack_pinion.move_RP(600);
    delay(50);
    limit_switch_reading = digitalRead(limit_switch_pin);
    if (limit_switch_reading == HIGH) {
      rack_pinion.RP_dir_forward();
      rack_pinion.move_RP(100);
    }
  }
}


/*void pick_up(void) {
  arm.arm_dir_down();
  rack_pinion.RP_dir_forward();
  arm.move_arm(700);
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
  arm.move_arm(90);
  digitalWrite(magnet_control_pin, LOW);
  
  arm.arm_dir_up();
  arm.move_arm(500);
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
  //morse_done();
}*/


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

void read_colour_sensor(void) {
  red, green, blue, colour_sum = 0;
  //uint16_t clear;
  tcs.setInterrupt(false);      // turn on LED
  delay(52);  // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &colour_sum);
  tcs.setInterrupt(true);  // turn off LED
}

/*void check_location(void) {
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
    location = AREA_UNKNOWN;
  }
}*/

/*void check_location(void) {
  if ((abs(red - BLACK_RED) < COLOUR_TOLERANCE) && (abs(blue - BLACK_BLUE) < COLOUR_TOLERANCE) && (abs(green - BLACK_GREEN) < COLOUR_TOLERANCE)) {
    location = BLACK_AREA;
  }
  else {
    location = AREA_UNKNOWN;
  }
}*/

/*void check_location(void) {
  if (colour_sum < 1480) {
    location = BLACK_AREA;
  }
  else {
    location = AREA_UNKNOWN;
  }
}*/

void check_location(void) {
  if (abs(COLOUR_SUM - colour_sum) >= COLOUR_TOLERANCE) {
    location = BLACK_AREA;
  }
  else {
    location = AREA_UNKNOWN;
  }
}

void morse_done(void) {
  dash();
  dot();
  dot();
  dash();
  dash();
  dash();
  dash();
  dot();
  dot();
}

void dash(void) {
  digitalWrite(magnet_control_pin, HIGH);
  delay(600);
  digitalWrite(magnet_control_pin, LOW);
  delay(600);
}

void dot(void) {
  digitalWrite(magnet_control_pin, HIGH);
  delay(200);
  digitalWrite(magnet_control_pin, LOW);
  delay(200);
}
