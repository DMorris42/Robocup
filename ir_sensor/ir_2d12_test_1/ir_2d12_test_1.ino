float val = 0;
int pin = 10;
static const float ADC_ratio = 5.0/1023.0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  val = read_gp2d12_range(pin);
  Serial.println(val);
  delay(500);
}

// This code seems to work fairly well, although the range is slightly off (close enough though)
float read_gp2d12_range(int pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (6787.0 /((float)tmp - 3.0)) - 4.0;
  if (tmp < 3) {
    range =  -1; // Error value
  }
  return range;
}
