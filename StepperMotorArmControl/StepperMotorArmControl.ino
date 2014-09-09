/*
This code controls the rotation of a stepper motor
Used for controlling motion of the arm within specified operational angles
Note that the stepper moves 1.8 degreees per step (+/- 5%, full step, no load)
*/

// Currently on stepper port 3 (pins closest to motor, far side of board)
int M3dirpin = 35;
int M3steppin = 34;

static const float DEGREE_PER_STEP = 1.8;

void setup()
{
  pinMode(M3dirpin,OUTPUT);
  pinMode(M3steppin,OUTPUT);
  Serial.begin(9600);
}
void loop()
{
  /*delay(500);
  arm_dir_down();
  move_arm(200);*/
  delay(800);
  arm_dir_up();
  move_arm(800);
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
  float j;
  float steps = angle/DEGREE_PER_STEP;  // Rotates the arm for time/1000 seconds (approx - should use timer later if more accuracy requried)
  for (j = 0.0; j <= steps; j++) {
    digitalWrite(M3steppin, LOW);
    delayMicroseconds(1);
    digitalWrite(M3steppin, HIGH);
    delay(1);
  }
}
  
