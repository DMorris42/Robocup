#include "morse.h"

Morse LED(A6);

void setup (void) {
}

void loop (void) {
  LED.h();
  LED.e();
  LED.l();
  LED.l();
  LED.o();
  delay(1000);
}
  
