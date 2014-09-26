/*
This code controls the rotation of the pinion stepper motor
Note that the stepper moves 1.8 degreees per step (+/- 5%, full step, no load)
*/


int RPdirpin = 35; 
int RPsteppin = 34;
static volatile int time = 0;
static volatile int prev_time = 0;

static const float DEGREE_PER_STEP = 1.8;

enum TRAY_STATE {
  forwards,
  backwards,
  unknown
};

static TRAY_STATE tray_dir = unknown;

void setup()
{
  pinMode(RPdirpin,OUTPUT);
  pinMode(RPsteppin,OUTPUT);
  RP_dir_forward();
  //RP_dir_backward();
  delay(300);
}
void loop()
{
  time = millis();
  //delay(800);
  //RP_dir_forward();
  move_RP(1);
  /*if ((time - prev_time) >= 600) {
    delay(1000);
    RP_dir_backward();
    move_RP(400);
    delay(500);
    time = millis();
    prev_time = time;
  }*/
  if ((time - prev_time) >= 800) {
    if (tray_dir == forwards) {
      RP_dir_backward();
    }
    else {
    RP_dir_forward();
    }
    time = millis();
    prev_time = time;
  }
}

// Sets the direction of rotation to lift the arm
void RP_dir_forward(void) {
  tray_dir = forwards;
  digitalWrite(RPdirpin, LOW);
}

// Sets the direction of rotation to lower the arm
void RP_dir_backward(void) {
  tray_dir = backwards;
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
  
