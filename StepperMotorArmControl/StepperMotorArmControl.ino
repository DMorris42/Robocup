/*
This code controls the rotation of a stepper motor
Used for controlling motion of the arm within specified operational angles
Note that the stepper moves 1.8 degreees per step (+/- 5%, full step, no load)
*/

#define GEAR_RATIO 3

// Currently on stepper port 3 (pins closest to motor, far side of board)
int M3dirpin = 35;
int M3steppin = 34;

static const float DEGREE_PER_STEP = 1.8;

void setup()
{
  pinMode(M3dirpin,OUTPUT);
  pinMode(M3steppin,OUTPUT);
  //Serial.begin(9600);
}
void loop()
{
  delay(800);
  arm_dir_down();
  move_arm(200);
  delay(800);
  arm_dir_up();
  move_arm(300);
  /*while(1) {
    Serial.println("Done");
    delay(1000);
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
  float steps = (angle/DEGREE_PER_STEP) * GEAR_RATIO;
  for (j = 0.0; j <= steps; j++) {
    digitalWrite(M3steppin, LOW);
    delayMicroseconds(2);
    digitalWrite(M3steppin, HIGH);
    delay(5);
  }
}
  
