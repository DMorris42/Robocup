int pin = A6;

void setup() {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void loop() {
  digitalWrite(pin, HIGH);
  delay(5000);
  digitalWrite(pin, LOW);
  delay(5000);
}
