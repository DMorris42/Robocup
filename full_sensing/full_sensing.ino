const int numSamples = 5;

static volatile float GP2D120val = 0;
int GP2D120data[numSamples] = {0};
int GP2D120dataIndex = 0;
int GP2D120dist = 0;

static volatile float GP2D12Leftval = 0;
int GP2D12Leftdata[numSamples] = {0};
int GP2D12LeftdataIndex = 0;
int GP2D12Leftdist = 0;

static volatile float GP2D12Rightval = 0;
int GP2D12Rightdata[numSamples] = {0};
int GP2D12RightdataIndex = 0;
int GP2D12Rightdist = 0;

static volatile float LongIRval = 0;
int LongIRdata[numSamples] = {0};
int LongIRdataIndex = 0;
int LongIRdist = 0;

int GP2D120pin = 1;
int GP2D12Leftpin = 10;
int GP2D12Rightpin = 0;
int LongIRpin = 2;
/*
GP2D120 on pin 1
Front Left GP2S12 on pin 10
Front Right GP2D12 on pin 0
GP2Y0A02YK on pin 2
*/

void setup() {
}

void loop() {
  GP2D12Leftval = read_gp2d12_range(GP2D12Leftpin);
  if (GP2D12Leftval != -1) {
    GP2D12Leftdata[GP2D12LeftdataIndex++] = GP2D12Leftval;
  }
  
  GP2D12Rightval = read_gp2d12_range(GP2D12Rightpin);
  if (GP2D12Rightval != -1) {
    GP2D12Rightdata[GP2D12RightdataIndex++] = GP2D12Rightval;
  }
  
  LongIRval = read_IR_long_range(LongIRpin);
  if (LongIRval != -1) {
    LongIRdata[LongIRdataIndex++] = LongIRval;
  }
  
  GP2D120val = read_gp2d120_range(GP2D120pin);
  if (GP2D120val != -1) {
    GP2D120data[GP2D120dataIndex++] = GP2D120val;
  }
  
  if (GP2D120dataIndex == numSamples) {
    GP2D120dist = sum(GP2D120data)/GP2D120dataIndex;
    GP2D120dataIndex = 0;
  }
  
  if (GP2D12LeftdataIndex == numSamples) {
    GP2D12Leftdist = sum(GP2D12Leftdata)/GP2D12LeftdataIndex;
    GP2D12LeftdataIndex = 0;
  }
  
  if (GP2D12RightdataIndex == numSamples) {
    GP2D12Rightdist = sum(GP2D12Rightdata)/GP2D12RightdataIndex;
    GP2D12RightdataIndex = 0;
  }
  
  if (LongIRdataIndex == numSamples) {
    LongIRdist = sum(LongIRdata)/LongIRdataIndex;
    LongIRdataIndex = 0;
  }
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
  if ((tmp < 3) || (range > 85)) {
    range =  -1; // Error value
  }
  return range;
}

float read_IR_long_range(byte pin) {
  int tmp = 0;
  float range = 0;
  tmp = analogRead(pin);
  //range = (9462.0 /((float)tmp - 16.92));
  //range = (9900.0 /((float)tmp - 16.92));
  range = 30431 * pow(float(tmp), -1.169);
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
