//Defining Variables
bool leftside_open = false; //detects open space while turning left
bool rightside_open = false; //detects open space while turning right
bool get_weight = false; //instructs the robot that there is a weight to be picked up
int detection_dist = 50; //defines the distance at which the differentiating process occurs
int tolerance = 20; //defines the tolerance allowed for the change in US_dist value while turning side to side 
int pickup_dist = 35; //define the distance from the weight at which the robot must be positioned to pickup the weight 
int pickup_angle = 894273847231470; //defines the angle of the arm at which the weight should be picked at
int rest_angle = 4903249023714023; //defines the angle of the arm at which it rests 
int angle = /*###*/; //the value of the current angle of the arm

void setup() {
  // put your setup code here, to run once:

}

//function to differentiate weights from walls
void single_object_detection(){ 
  while (/*###turning left for an angle enough to get out of the original cone of vision*/){ //this may require more variables to define the end of turning ###
    if (/*###US_dist*/ > detection_dist + tolerance){ //if there is an open space (dist value changes to a large value)
      leftside_open = true;  
    }
  }
    
  /*###TURN RIGHT for the full range again to restore to original position*/
      
  while (/*###turning right for an angle enough to get out of the original cone of vision*/){ //this may require more variables to define the end of turning ###
    if (/*###US_dist*/ > detection_dist + tolerance){ //if there is an open space (dist value changes to a large value)
      rightside_open = true;  
    }
    
    /*###TURN LEFT for the full range again to restore to original position*/
}

//function to move the robot to align the arm to the weight
void align_robot(){
  if (leftside_open){
    while (/*###US_dist*/ <= detection_dist){ //turn left until the weight moves out of the field of detection
      /*###TURN LEFT*/ 
    } 
    /*###TURN RIGHT FOR HALF OF THE FULL CONE RANGE TO PLACE THE OBJECT RIHT INFRONT OF THE SENSOR (this can be tweaked abit to account for the misalignment of the sensor and the arm*/   
  }
  else if (rightside_open){
    while (/*###US_dist*/ <= detection_dist){ //turn right until the weight moves out of the field of detection
      /*###TURN RIGHT*/ 
    } 
    /*###TURN RIGHT FOR HALF OF THE FULL CONE RANGE TO PLACE THE OBJECT RIHT INFRONT OF THE SENSOR (this can be tweaked abit to account for the misalignment of the sensor and the arm*/     
  }
  
  leftside_open = false; //reset the record
  rightside_open = false; //reset the record
  get_weight = true;
}

//function to position the robot to the right spot to pick up the weight
void move_to_weight(){
  //!!!the following process could be set up using na PID control to locate the robot
  while (/*###US_dist*/ > pickup_dist){ //until it reaches the right distance go forward
    //###MOVE FORWARD (not too fast to prevent overshoot)
  }
  if (/*###US_dist*/ == 30){ //if overshoot (moved too close so the value of sensor is 30)
    while (/*###US_dist*/ < pickup_dist){ //move back little bit until the right distance is reached
      //###MOVE BACK SLOWLY
    }
  }
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

void loop() {
  // put your main code here, to run repeatedly:
      if (/*###US_dist*/ <= detection_dist){ //if a object/wall is detected, it determines whether it is a wall or an object
        single_object_detection();
      }
      if (leftside_open || rightside_open){ //if there was an open space on either side of the object/wall, it is an object
        align_robot(); //move the robot so the arm is in line with the object
      }
      if (get_weight){
        move_to_weight(); //move to the weight
        pickup(); //pick up the weight
      }
}
