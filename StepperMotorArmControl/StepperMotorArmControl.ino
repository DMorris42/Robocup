/*
This code controls the rotation of a stepper motor
Used for controlling motion of the arm within specified operational angles
Note that the stepper moves 1.8 degreees per step (+/- 5%, full step, no load)
*/

#define GEAR_RATIO 3

// Currently on stepper port 3 (pins closest to motor, far side of board)
int M3dirpin = 31; //35?
int M3steppin = 30;  //34?
static volatile int time = 0;
static volatile int prev_time = 0;

static const float DEGREE_PER_STEP = 1.8;

void setup()
{
  pinMode(M3dirpin,OUTPUT);
  pinMode(M3steppin,OUTPUT);
  //Serial.begin(9600);
  delay(300);
}
void loop()
{
  time = millis();
  /*delay(800);
  arm_dir_down();
  move_arm(90);*/
  //delay(800);
  arm_dir_up();
  move_arm(1);
  if ((time - prev_time) >= 1800) {
    /*arm_dir_down();
    move_arm(200);*/
    time = millis();
    prev_time = time;
    while(1) {
    //Serial.println("Done");
    delay(1000);
  }
  }
  /*arm_dir_up();
  move_arm(1);
  if (time >= 1500) {
    arm_dir_down();
    while (time < 3000) {
      time = millis();
      move_arm(1);
    }
  }
  while (1) {
  }*/
}

// Sets the direction of rotation to lift the arm
void arm_dir_up(void) {
  digitalWrite(M3dirpin, HIGH);
}

// Sets the direction of rotation to lower the arm
void arm_dir_down(void) {
  digitalWrite(M3dirpin, LOW);
}

// Moves the arm by a certain angle, independent of direction
void move_arm(int angle) {
  float j = 0;
  int steps = (angle/DEGREE_PER_STEP) * GEAR_RATIO * 2;
  for (j = 0.0; j <= steps; j++) {
    digitalWrite(M3steppin, LOW);
    delayMicroseconds(1);
    digitalWrite(M3steppin, HIGH);
    delay(1);
  }
}
  
