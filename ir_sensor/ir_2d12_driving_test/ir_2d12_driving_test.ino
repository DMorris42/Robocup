//Test using GP2D12 with driving to avoid obstacles
//GP2D12 is on analog port A6
//DC motors on servo port S1
//IT WORKS!

#include <Servo.h>

float distance = 0;
int pin = 10;
static const float ADC_ratio = 5.0/1023.0;
Servo servo_left;
Servo servo_right;
//Speeds for both motors
static int left_speed = 55;
static int right_speed = 55;

void setup() {
  servo_left.attach(3);
  servo_right.attach(2);
  servo_left.write(left_speed);
  servo_right.write(right_speed);
  Serial.begin(9600);
}

void loop() {
  distance = read_gp2d12_range(pin);
  /*if (distance == -1) {
    distance = read_gp2d12_range(pin);
    if (distance == -1) {
      reverse();
      turn_left();
      drive();
    }
  }*/
  Serial.println(distance);
  if ((distance <= 40) && (distance != -1)) {
    turn_left();
  }
  else {
    drive();
  }
  delay(100);
}

// This code seems to work fairly well, although the range is slightly off (close enough though)
float read_gp2d12_range(int pin) {
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
  servo_left.write(125);
  servo_right.write(55);
  delay(2000);
}

void turn_right(void) {
  servo_left.write(55);
  servo_right.write(125);
  delay(200);
}

void reverse(void) {
  servo_left.write(125);
  servo_right.write(125);
  delay(1000);
}

void drive(void) {
  servo_left.write(55);
  servo_right.write(55);
  //delay(1500);
}
