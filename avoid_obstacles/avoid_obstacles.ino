
int left_IR_dist = 0;
int right_IR_dist = 0;
bool left_obstacle_cruising;
bool right_obstacle_cruising;
bool front_obstacle_cruising;
bool left_obstacle;
bool right_obstacle;

bool first_start = true;



void setup() {
  // put your setup code here, to run once:
  rack_pinion.RP_dir_forward();
  arm.arm_dir_down();
  motor_left.attach(3);
  motor_right.attach(2);
  delay(1000)
  //Serial.println("STARTING CODE");
}

void loop() {
  // put your main code here, to run repeatedly:
  //keep a count of cycles to record the angle rotated (this value must be derived from experiments to get the right conversion value which accounts for rotating speed and system clock speed - may want to use interrupts for consistency)
  if (first_start){ //only do at initial boot up
    //IR_check();
    //for (i=0; i<10; i++){ //more stable condition by checking more than one instance
    //  if (upper_IR_dist == -1) { //better more versatile condition ?? more specific
        wall_state = true;
    //  }
    }
    delay(50);
    first_start = false;
  }
  
  while (wall_state) {
    
    IR_check();
    
    left_obstacle_cruising = ((left_IR_dist < 12)&&(left_IR_dist > 0));
    right_obstacle_cruising = ((right_IR_dist < 12)&&(right_IR_dist > 0));
    front_obstacle_cruising = ((upper_IR_dist < 15)&&(upper_IR_dist > 0));
    left_obstacle = ((left_IR_dist < 40)&&(left_IR_dist > 0));
    right_obstacle = ((right_IR_dist < 40)&&(right_IR_dist > 0));
    
    //for front wall 
    //(want the front distance to be minimal as possible while still being able to not crash into walls infront - back and front of the robot) 
      //(depends on the distance between the front sensor and the side of the robot and the sharpness of the turn the robot can make - experiment)
      //set it as the distance between the front sensor and the sides of the robot for now
    
    if (front_obstacle_cruising) { //for front wall
      //moving direction
      //if object on one side, move away from the closer side
      if (left_obstacle){ //determined by side sensor distance detection being less than a set value - modify for later code
        turn_right(); //until (front_upper_dist > 15) wall follow for now(in the final code consider turning towards wanted path direction)
      }
      else if (right_obstacle){
        turn_left(); //until (front_upper_dist > 15);
      }
      //if object on both sides, move backwards until only one side (or no sides) detects an object then spin 90 degress away from the present object side (if no objects on both sides determine the direction of turn based on the overall wanted direction of path)
      else if (left_obstacle && right_obstacle){
        while (left_obstacle && right_obstacle){ //until only one side has a wall, or neither side has a wall
          reverse();
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
      //neither sides have a wall
      else { //if there is no objects on both sides, turn in the direction that points in the original wanted direction - store the wanted direction and the direction the robot has turned so far (FOR NOW WE USE CLOCKWISE)
        turn_right(); //until (front_upper_dist > 15)
      }
    }
    else { //for cruising past sides walls
      if (left_obstacle_cruising){ //if distance below 12cm move away til it is 12cm (minimum sensor distance 10cm)
        turn_right(); //until (left_sensor_dist > 12) wall follow for now(in the final code consider turning towards wanted path direction)
      }
      else if (right_obstacle_cruising){
        turn_left(); //until (right_sensor_dist > 12) wall follow for now(in the final code consider turning towards wanted path direction)
      }
      else {
        //move forward if no obstacles
        motor_drive();
      }
    }
  }    
}

void IR_check(void){
  upper_IR_dist = read_long_IR (LongIRpin);
  lower_IR_dist = read_short_IR (GP2D120pin);
  left_IR_dist = read_left_IR(GP2D12Leftpin);
  right_IR_dist = read_right_IR(GP2D12Rightpin);
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
  GP2D12Leftdist = filterSum(GP2D12Leftdata, GP2D12LeftdataIndex);
  GP2D12Leftdist = GP2D12Leftdist/GP2D12LeftdataIndex;
  GP2D12LeftdataIndex = 0;
  return GP2D12Leftdist;
}

int read_right_IR (byte pin) {
  while (GP2D12RightdataIndex < 5) {
    GP2D12Rightdata[GP2D12RightdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Rightdist = filterSum(GP2D12Rightdata, GP2D12RightdataIndex);
  GP2D12Rightdist = GP2D12Rightdist/GP2D12RightdataIndex;
  GP2D12RightdataIndex = 0;
  return GP2D12Rightdist;
}

int read_short_IR (byte pin) {
  while (GP2D120dataIndex < 5) {
    GP2D120data[GP2D120dataIndex++] = read_gp2d120_range(pin);
  }
  GP2D120dist = filterSum(GP2D120data, GP2D120dataIndex);
  GP2D120dist = GP2D120dist/GP2D120dataIndex;
  GP2D120dataIndex = 0;
  return GP2D120dist;
}

int read_long_IR (byte pin) {
  while (LongIRdataIndex < 5) {
    LongIRdata[LongIRdataIndex++] = read_IR_long_range(pin);
  }
  LongIRdist = filterSum(LongIRdata, LongIRdataIndex);
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
