/*
This code controls the rotation of a stepper motor using the arduino lib
Used for controlling motion of the arm within specified operational angles
Note that the stepper moves 1.8 degreees per step (+/- 5%, full step, no load)
*/

#include <Stepper.h>

#define STEPS 200
#define GEAR_RATIO 3

// Currently on stepper port 3 (pins closest to motor, far side of board)
int M3dirpin = 35;
int M3steppin = 34;

Stepper arm_stepper(STEPS, M3dirpin, M3steppin);

static volatile int time = 0;
static volatile int prev_time = 0;

static const float DEGREE_PER_STEP = 1.8;

void setup()
{
  arm_stepper.setSpeed(300); //Speed, RPM
}

//Positive step goes down
void loop()
{
  arm_stepper.step(-1000);
  while (1) {
  }
}

// Sets the direction of rotation to lift the arm
void arm_dir_up(void) {
  digitalWrite(M3dirpin, HIGH);
}

// Sets the direction of rotation to lower the arm
void arm_dir_down(void) {
  digitalWrite(M3dirpin, LOW);
}

  
