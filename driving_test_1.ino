#include <Servo.h>

Servo servo1;
Servo servo2;
int val = 120;

void setup() {
  servo1.attach(2);
  servo2.attach(3);
  servo1.write(val);
  servo2.write(val);
}

void loop() {
}
