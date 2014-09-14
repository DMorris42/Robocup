#include <Servo.h>

//Directions are with servo body towards you
#define HOLD 92
#define FORWARDS 0
#define BACKWARDS 255

Servo worm_drive;
int worm_drive_pin = 10;

void setup() {
  worm_drive.attach(worm_drive_pin);
  worm_drive.write(HOLD);
  //worm_drive.write(FORWARDS);
}

void loop() {
  /*delay(3000);
  worm_drive.write(HOLD);
  while (1) {
  }*/
  worm_drive.write(200);
}
