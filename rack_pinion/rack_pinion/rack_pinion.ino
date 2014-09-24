/*
This code controls the rotation of the pinion stepper motor
Note that the stepper moves 1.8 degreees per step (+/- 5%, full step, no load)
*/


int RPdirpin = 31; 
int RPsteppin = 30;
static volatile int time = 0;
static volatile int prev_time = 0;

static const float DEGREE_PER_STEP = 1.8;

void setup()
{
  pinMode(RPdirpin,OUTPUT);
  pinMode(RPsteppin,OUTPUT);
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
  RP_dir_forward();
  move_RP(1);
  if ((time - prev_time) >= 600) {
    RP_dir_backward();
    move_RP(400);
    time = millis();
    prev_time = time;
  }
}

// Sets the direction of rotation to lift the arm
void RP_dir_forward(void) {
  digitalWrite(RPdirpin, LOW);
}

// Sets the direction of rotation to lower the arm
void RP_dir_backward(void) {
  digitalWrite(RPdirpin, HIGH);
}

// Moves the arm by a certain angle, independent of direction
void move_RP(int angle) {
  float j = 0;
  int steps = (angle/DEGREE_PER_STEP) * 2;
  for (j = 0.0; j <= steps; j++) {
    digitalWrite(RPsteppin, LOW);
    delayMicroseconds(1);
    digitalWrite(RPsteppin, HIGH);
    delay(1);
  }
}
  
