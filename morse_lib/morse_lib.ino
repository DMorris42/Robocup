#include "morse.h"

Morse LED(A6);

void setup (void) {
  LED.set_element_length(200);
}

void loop (void) {
  LED.h();
  LED.e();
  LED.l();
  LED.l();
  LED.o();
  LED.word_space();
}
  
