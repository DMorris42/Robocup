/* Driving using the rotary encoders for sensing distance
Note right encoder lags left. This accumulates over time and is a problem. Need to adjust tension on both tracks to match them.
*/

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
  backwards,
  left,
  right,
  stopped
};

//663 Pulses per revolution, but quadrature so need to double (?). Seems to work better.
static const int ENCODER_COUNTS = 663*2;
//Degrees per count
static const float ENCODER_COUNT_ANGLE = 0.54;

volatile unsigned int encoderPosLeft = 0;
volatile unsigned int lastReportedPosLeft = 1;
volatile unsigned int encoderPosRight = 0;
volatile unsigned int lastReportedPosRight = 1;

volatile int revolutionsLeft = 0;
volatile int revolutionsRight = 0;
volatile int lastReportedRevolutionsLeft = 0;
volatile int lastReportedRevolutionsRight = 0;

volatile int dist_travelled = 0; //Distance robot has travelled (total dist in any combo of directions)

volatile int angle_turned = 0;  //Angle turned, in degrees

Servo motor_left;
Servo motor_right;

int motor_left_speed = 90;
int motor_right_speed = 90;
Direction robot_dir = stopped;

void setup() {
// Encoder pin on interrupt 4
  attachInterrupt(4, doEncoderRight, CHANGE);
// Encoder pin on interrupt 5
  attachInterrupt(5, doEncoderLeft, CHANGE);
  motor_left.attach(3);
  motor_right.attach(2);
  Serial.begin(9600);
  //drive();
  turn_left();
  //reverse();
}


void loop()
{
  //DISTANCE TESTING CODE BEGIN
  
  /*if (lastReportedRevolutionsLeft != revolutionsLeft) {
    motor_stop();
    delay(1000);
    lastReportedRevolutionsLeft = revolutionsLeft;
    lastReportedRevolutionsRight = revolutionsRight;
    drive();
  }*/
  
  //DISTANCE TESTING CODE END
  
  //DISTANCE DRIVING CODE BEGIN
  /*if (max(encoderPosLeft, encoderPosRight) != 0) {
    //dist_travelled = max(revolutionsLeft, revolutionsRight)*REVOLUTION_DIST + (float(max(encoderPosLeft, encoderPosRight))/float(ENCODER_COUNTS))* REVOLUTION_DIST; //Taking max
    dist_travelled = ((revolutionsLeft + revolutionsRight)/2)*REVOLUTION_DIST + (float(max(encoderPosLeft, encoderPosRight))/float(ENCODER_COUNTS))* REVOLUTION_DIST; //Taking average
  }
  else {
    //dist_travelled = max(revolutionsLeft, revolutionsRight)*REVOLUTION_DIST;
    dist_travelled = ((revolutionsLeft + revolutionsRight)/2) * REVOLUTION_DIST;
  }
  Serial.println(dist_travelled, DEC);*/
  
  /*if (dist_travelled >= 1020) {
    motor_stop();
    Serial.print("DISTANCE REACHED:");
    Serial.print(dist_travelled, DEC);
    delay(3000);
    dist_travelled = 0;
    revolutionsLeft = 0;
    revolutionsRight = 0;
    encoderPosLeft = 0;
    encoderPosRight = 0;
    Serial.println("DIST = 0");
    Serial.println(dist_travelled, DEC);
  }
  Serial.println("Driving");
  drive();*/
  
  //DISTANCE TESTING CODE END 
    
  //ENCODER VALUE TESTING BEGIN  
  
  /*if ((lastReportedPosLeft != encoderPosLeft) || (lastReportedPosRight != encoderPosRight)) {
    Serial.print("Left/Right:");
    Serial.print(encoderPosLeft, DEC);
    Serial.print(":");
    Serial.print(encoderPosRight, DEC);
    Serial.println();
    lastReportedPosLeft = encoderPosLeft;
    lastReportedPosRight = encoderPosRight;
  }
  if ((lastReportedRevolutionsLeft != revolutionsLeft) || (lastReportedRevolutionsRight != revolutionsRight)) {
    Serial.print("Left rev/Right rev:");
    Serial.print(revolutionsLeft, DEC);
    Serial.print(":");
    Serial.print(revolutionsRight, DEC);
    Serial.println();
    lastReportedRevolutionsLeft = revolutionsLeft;
    lastReportedRevolutionsRight = revolutionsRight;
  }*/
  
  //ENCODER VALUE TESTING END
  
  //ROTATION TESTING BEGIN
  
  if (max(encoderPosLeft, encoderPosRight) != 0) {
    if (robot_dir == left) {
      angle_turned = ((-1*revolutionsLeft + revolutionsRight)/2)*ANGLE_SCALE + (float((((MAX_ENCODER_VAL - encoderPosLeft) + encoderPosRight)/2))/float(ENCODER_COUNTS))*ANGLE_SCALE; //Taking average
    }
    else {
      angle_turned = ((revolutionsLeft + -1*revolutionsRight)/2)*ANGLE_SCALE + (float((((MAX_ENCODER_VAL - encoderPosRight) + encoderPosLeft)/2))/float(ENCODER_COUNTS))*ANGLE_SCALE; //Taking average
    }
  }
  else {
    if (robot_dir == left) {
      angle_turned = ((-1*revolutionsLeft + revolutionsRight)/2) * ANGLE_SCALE;
    }
    else {
      angle_turned = ((-1*revolutionsRight + revolutionsLeft)/2)* ANGLE_SCALE;
    }
  }
  //Serial.println(angle_turned, DEC);
  
  if (angle_turned >= 360) {
    motor_stop();
    //Serial.println("ANGLE REACHED");
    //Serial.println(angle_turned, DEC);
    angle_turned = 0;
    revolutionsLeft = 0;
    revolutionsRight = 0;
    encoderPosLeft = 0;
    encoderPosRight = 0;
    delay(1000);
    turn_left();
  }
}

// Interrupt on A changing state
void doEncoderLeft(){
  if ((robot_dir != backwards) && (robot_dir != left)) {
    encoderPosLeft++;
    if (encoderPosLeft >= ENCODER_COUNTS) {
        encoderPosLeft = 0;
        revolutionsLeft++;
    }
  }
  else {
    if (encoderPosLeft > 0) {
      encoderPosLeft--;
    }
    else {
      revolutionsLeft--;
      encoderPosLeft = ENCODER_COUNTS - 1;
    }
  }
}


//Interrupt on A changing state
void doEncoderRight() {
  if ((robot_dir != backwards) && (robot_dir != right)) {
    encoderPosRight++;
    if (encoderPosRight >= ENCODER_COUNTS) {
      encoderPosRight = 0;
      revolutionsRight++;
    }
  }
  else {
    if (encoderPosRight > 0) {
      encoderPosRight--;
    }
    else {
      revolutionsRight--;
      encoderPosRight = ENCODER_COUNTS - 1;
    }
  }
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
  robot_dir = backwards;
}

void drive(void) {
  motor_left.write(DRIVE_SPEED);
  motor_right.write(DRIVE_SPEED);
  robot_dir = forward;
} 

void motor_stop(void) {
  motor_left.write(STOP_SPEED);
  motor_right.write(STOP_SPEED);
  robot_dir = stopped;
}
