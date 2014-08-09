static byte pin = 10;
static float range = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  range = read_gp2d12_range(pin);
  Serial.println(range);
  delay(100);
}

float read_gp2d12_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (6787.0 /((float)tmp - 3.0)) - 4.0;
  if (tmp < 3) {
    range =  -1; // Error value
  }
  return range;
}
