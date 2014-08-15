/*
This code controls the rotation of a stepper motor
Used for controlling motion of the arm within specified operational angles
*/

// Currently on stepper port 3 (pins closest to motor, far side of board)
int M3dirpin = 35;
int M3steppin = 34;

void setup()
{
  pinMode(M3dirpin,OUTPUT);
  pinMode(M3steppin,OUTPUT);
  Serial.begin(9600);
}
void loop()
{
  arm_dir_up();
  move_arm(900);
  delay(1000);
  arm_dir_down();
  move_arm(100);
  delay(1000);
}

// Sets the direction of rotation to lift the arm
void arm_dir_up(void) {
  digitalWrite(M3dirpin, LOW);
}

// Sets the direction of rotation to lower the arm
void arm_dir_down(void) {
  digitalWrite(M3dirpin, HIGH);
}

// Moves the arm
void move_arm(int time) {
  int j;
  // Rotates the arm for time/1000 seconds (approx - should use timer later if more accuracy requried)
  for (j = 0; j <= time; j++) {
    digitalWrite(M3steppin, LOW);
    delayMicroseconds(2);
    digitalWrite(M3steppin, HIGH);
    delay(1);
  }
}
  
