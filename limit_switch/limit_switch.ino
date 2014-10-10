//Switch returns HIGH when open

int switch_pin = A8;
int led_pin = 13;
int reading = LOW;

void setup() {
  pinMode(switch_pin, INPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  delay(1000);
  digitalWrite(led_pin, LOW);
  delay(1000);
  Serial.begin(9600);
}

void loop() {
  reading = digitalRead(switch_pin);
  Serial.println(reading);
  if (reading == HIGH) {
    digitalWrite(led_pin, HIGH);
    Serial.println("WORKING!");
  }
  else {
    digitalWrite(led_pin, LOW);
  } 
}
