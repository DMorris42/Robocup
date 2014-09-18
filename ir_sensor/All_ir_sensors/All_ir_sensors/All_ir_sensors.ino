float val = 0;
int pin = 10;

void setup() {
  Serial.begin(9600);
}

void loop() {
  //val = read_gp2d12_range(pin);
  //val = read_IR_long_range(pin);
  val = read_gp2d120_range(pin);
  Serial.println(val);
  delay(600);
}

// This code seems to work fairly well, although the range is slightly off (close enough though)
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

float read_IR_long_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (9462.0 /((float)tmp - 16.92));
  if (tmp <= 16.92) {
    range =  -1; // Error value
  }
  return range;
}

float read_gp2d120_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  range = (2914.0 /((float)tmp + 3.0)) - 1.0;
  if (tmp < 6) {
    range =  -1; // Error value
  }
  return range;
}
