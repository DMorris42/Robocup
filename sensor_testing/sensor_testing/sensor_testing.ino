float val = 0;
int pin = 10;
const int numSamples = 5;
int data[numSamples] = {0};
int dataIndex = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  val = read_gp2d12_range(pin);
  //val = read_IR_long_range(pin);
  //val = read_gp2d120_range(pin);
  //val = read_ul_sensor_range(pin);
  if (dataIndex == numSamples) {
    val = sum(data)/dataIndex;
    dataIndex = 0;
    Serial.println(val);
  }
  delay(100);
}

int sum(int* dataArray) {
  int i;
  int sum = 0;
  for (i = 0; i < numSamples; i++) {
    sum += *dataArray++;
  }
  return sum;
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
  data[dataIndex++] = range;
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
  data[dataIndex++] = range;
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
  data[dataIndex++] = range;
  return range;
}

float read_ul_sensor_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  //Min val ~ 56 (dist. <= 30cm), ~58 at 30cm
  //ADC value increases by ~20 per 10 cm (from rough testing; check with Julian)
  if (tmp <= 58) {
    range = 30.0;
  }
  else {
    //Works; slight underestimate though
    range = 30.0 + (float(tmp) - 56.0)/2.0;
  }
  data[dataIndex++] = range;
  return range;
}
