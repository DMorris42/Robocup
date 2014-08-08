//Sweeps two servos on pins 10 & 11 (servo port 5, S5 on port schematic)

#include <Servo.h>

Servo servo1;
Servo servo2;

int i = 0;

void setup() {
  servo1.attach(11);
  servo2.attach(10);
  servo1.write(90);
  servo2.write(90);
}

void loop() {
  for (i = 0; i < 180; i++) {
    servo1.write(i);
    servo2.write(i);
    delay(8);
  }
  for (i = 180; i > 1; i--) {
    servo1.write(i);
    servo2.write(i);
    delay(8);
  }
}
