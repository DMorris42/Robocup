int magnet_pin = A6;

void setup() {
  pinMode(magnet_pin, OUTPUT);
}

void loop() {
  morse_done();
  delay(2000);
}

void morse_done(void) {
  dash();
  dot();
  dot();
  dash();
  dash();
  dash();
  dash();
  dot();
  dot();
}

void dash(void) {
  digitalWrite(magnet_pin, HIGH);
  delay(600);
  digitalWrite(magnet_pin, LOW);
  delay(600);
}

void dot(void) {
  digitalWrite(magnet_pin, HIGH);
  delay(200);
  digitalWrite(magnet_pin, LOW);
  delay(200);
}
