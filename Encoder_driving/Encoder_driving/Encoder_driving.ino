/* Driving using the rotary encoders for sensing distance
Note right encoder lags left. This accumulates over time and could be a problem. May need to adjust tension on both tracks to match them.
*/

#include <Servo.h>

#define DRIVE_SPEED 135
#define STOP_SPEED 90
#define REVERSE_SPEED 55
#define TURN_WHEEL_SLOW 55
#define TURN_WHEEL_FAST 125

#define REVOLUTION_DIST 215  //Distance, in mm, covered in one rev of wheel with tracks (measured; could be up to 219 mm)
//This distance may be slightly too big from testing

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

volatile float dist_travelled = 0; //Distance robot has travelled (total dist in any combo of directions)

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
  drive();
  delay(100);
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
  if (max(encoderPosLeft, encoderPosRight) != 0) {
    dist_travelled = max(revolutionsLeft, revolutionsRight)*REVOLUTION_DIST + (float(max(encoderPosLeft, encoderPosRight))/float(ENCODER_COUNTS))* REVOLUTION_DIST;
  }
  /*else {
    dist_travelled = max(revolutionsLeft, revolutionsRight)*REVOLUTION_DIST;
  }*/
  Serial.println(dist_travelled, DEC);
  
  if (dist_travelled >= 1000) {
    motor_stop();
    Serial.print("DISTANCE REACHED:");
    Serial.print(dist_travelled, DEC);
    while (1) {
    }
  }
    
  //ENCODER VALUE TESTING BEGIN  
  
  /*if ((lastReportedPosLeft != encoderPosLeft) || (lastReportedPosRight != encoderPosRight)) {
    Serial.print("Left/Right:");
    Serial.print(encoderPosLeft, DEC);
    Serial.print(":");
    Serial.print(encoderPosRight, DEC);
    Serial.println();
    lastReportedPosLeft = encoderPosLeft;
    lastReportedPosRight = encoderPosRight;
  }*/
  /*if ((lastReportedRevolutionsLeft != revolutionsLeft) || (lastReportedRevolutionsRight != revolutionsRight)) {
    Serial.print("Left rev/Right rev:");
    Serial.print(revolutionsLeft, DEC);
    Serial.print(":");
    Serial.print(revolutionsRight, DEC);
    Serial.println();
    lastReportedRevolutionsLeft = revolutionsLeft;
    lastReportedRevolutionsRight = revolutionsRight;
  }*/
  
  //ENCODER VALUE TESTING END
}

// Interrupt on A changing state
void doEncoderLeft(){
  if (robot_dir != backwards) {
    encoderPosLeft++;
    if (encoderPosLeft == ENCODER_COUNTS) {
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
  if (robot_dir != backwards) {
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
      encoderPosRight = ENCODER_COUNTS - 1; //This breaks it if any value other than 0 or 1 is used. I don't know why...
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
