/* Driving with panning GP2D12 and ultrasonic sensor
*/

#include <Servo.h>

Servo servoIR;
Servo servoUS;
int IRPos = 0;
int USPos = 0;
float USdistance = 0;
float IRdistance = 0;
Servo motor_left;
Servo motor_right;
int motor_left_speed = 90;
int motor_right_speed = 90;
int i = 0;
int USpin = 8;
int GP2D12pin = 10;
int sensing_dist = 30;

void setup() {
  servoIR.attach(13);
  servoUS.attach(12);
  motor_left.attach(3);
  motor_right.attach(2);
  servoIR.write(90);
  servoUS.write(90);
  motor_left.write(motor_left_speed);
  motor_right.write(motor_right_speed);
  drive();
}

void loop() {
  for (i = 0; i =< 90; i++) {
    IRPos = i;
    USPos = 180 - i;
    servoIR.write(IRPos);
    servoUS.write(USPos);
    USdistance = read_ul_sensor_range();
    IRdistance = read_gp2d12_range();
    if (((USdistance <= sensing_dist) && (USPos > 90)) || ((IRdistance <= sensing_dist) && (IRPos > 90))) {
      turn_left();
    }
    else if (((USdistance <= sensing_dist) && (USPos < 90)) || ((IRdistance <= sensing_dist) && (IRPos < 90))) {
      turn_right();
    }
    else if (((USdistance <= sensing_dist) && (USPos == 90)) || ((IRdistance <= sensing_dist) && (IRPos == 90))) {
     reverse();
     delay(100);
     turn_left();
    } 
    else {
      drive();
    }
    delay(10);
  }
  for (i = 90; i >= 0; i--) {
    IRPos = i;
    USPos = 90 - i;
    servoIR.write(IRPos);
    servoUS.write(USPos);
    USdistance = read_ul_sensor_range();
    IRdistance = read_gp2d12_range();
    if (((USdistance <= sensing_dist) && (USPos > 90)) || ((IRdistance <= sensing_dist) && (IRPos > 90))) {
      turn_left();
    }
    else if (((USdistance <= sensing_dist) && (USPos < 90)) || ((IRdistance <= sensing_dist) && (IRPos < 90))) {
      turn_right();
    }
    else if (((USdistance <= sensing_dist) && (USPos == 90)) || ((IRdistance <= sensing_dist) && (IRPos == 90))) {
     reverse();
     delay(100);
     turn_left();
    }
   else {
    drive();
   } 
    delay(10);
  }
}

float read_ul_sensor_range(void) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(USpin);
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

float read_gp2d12_range(void) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(GP2D12pin);
  range = (6787.0 /((float)tmp - 3.0)) - 4.0;
  if (tmp < 3) {
    range =  -1; // Error value
  }
  return range;
}

void turn_left(void) {
  motor_left.write(125);
  motor_right.write(55);
}

void turn_right(void) {
  motor_left.write(55);
  motor_right.write(125);
}

void reverse(void) {
  motor_left.write(125);
  motor_right.write(125);
}

void drive(void) {
  motor_left.write(55);
  motor_right.write(55);
} 
