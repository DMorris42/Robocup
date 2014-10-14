int current_angle = 0;
int upper_IR_dist = 0;
int lower_IR_dist = 0;
int upper_IR_conv_dist = 0;
int difference_US_dist = 0;
int difference_IR_dist = 0;
int tolerance_upper_lower = 6;
unsigned int min_consec_weight_detects = 4;
int left_IR_dist = 0;
int right_IR_dist = 0;
int front_US = 0;

const int numSamples = 10;

int LowerIRdata[numSamples] = {0};
int LowerIRdataIndex = 0;
int LowerIRdist = 0;
int prev_lower_IR_dist = 0;

int GP2D12Leftdata[numSamples] = {0};
int GP2D12LeftdataIndex = 0;
int GP2D12Leftdist = 0;

int GP2D12Rightdata[numSamples] = {0};
int GP2D12RightdataIndex = 0;
int GP2D12Rightdist = 0;

int LongIRdata[numSamples] = {0};
int LongIRdataIndex = 0;
int LongIRdist = 0;

int ULdata[numSamples] = {0};
int ULdataIndex = 0;
int ULdist = 0;

int bottomIRpin = 1;
int GP2D12Leftpin = 10;
int GP2D12Rightpin = 0;
int topLongIRpin = 2;
int ULpin = 3;

static unsigned int bottomIRMinRange = 8;
static unsigned int bottomIRMaxRange = 80;

void setup (void) {
  Serial.begin(9600);
}

void loop (void) {
  IR_check();
  if ((difference_IR_dist >=tolerance_upper_lower) && (difference_US_dist >= tolerance_upper_lower)) {
    Serial.println("Weight");
    Serial.println();
  }
  delay(500);
}

void IR_check(void){
  upper_IR_dist = read_long_IR (topLongIRpin);
  lower_IR_dist = read_lower_IR (bottomIRpin);
  left_IR_dist = read_left_IR(GP2D12Leftpin);
  right_IR_dist = read_right_IR(GP2D12Rightpin);
  front_US = read_US(ULpin);
  if (upper_IR_dist != -1) {
    upper_IR_dist -= 80/(upper_IR_dist); //Temp fix??
    //upper_IR_dist -= 4;
    upper_IR_conv_dist = upper_IR_dist - 13; //13 for now but change to more accurate number later
  }
  else {
    upper_IR_conv_dist = -1;
  }
  if ((lower_IR_dist > bottomIRMaxRange) || (lower_IR_dist < bottomIRMinRange)) {
    lower_IR_dist = -1;
  }
  /*else {
    lower_IR_dist += 3; //Temp fix, as top and bottom at different ranges
  }*/
  /*Serial.print("UPPER IR: ");
  Serial.println(upper_IR_dist, DEC);
  Serial.print("LOWER IR: ");
  Serial.println(lower_IR_dist);*/
  //problem when upper_IR_dist == -1 or anything below 16
  
  Serial.println("FORWARD IR TESTING");
  Serial.print("Upper IR dist: ");
  Serial.println(upper_IR_dist, DEC);
  Serial.print("Lower IR dist: ");
  Serial.println(lower_IR_dist, DEC);
  Serial.print("Converted upper dist: ");
  Serial.println(upper_IR_conv_dist, DEC);
  Serial.print("US dist: ");
  Serial.println(front_US, DEC);
  if (lower_IR_dist == -1) {
    difference_IR_dist = 0; //just a number below the chosen tolerance
    difference_US_dist = 0;
  }
  else {
    difference_US_dist = ((front_US - 13) - lower_IR_dist);
    difference_IR_dist = upper_IR_conv_dist - lower_IR_dist;
    Serial.print("Difference IR is: ");
    Serial.println(difference_IR_dist, DEC);
    Serial.print("Difference US is: ");
    Serial.println(difference_US_dist, DEC);
    Serial.println();
  }
}

int read_left_IR (byte pin) {
  while (GP2D12LeftdataIndex < numSamples) {
    GP2D12Leftdata[GP2D12LeftdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Leftdist = filterSum(GP2D12Leftdata, &GP2D12LeftdataIndex);
  GP2D12Leftdist = GP2D12Leftdist/GP2D12LeftdataIndex;
  GP2D12LeftdataIndex = 0;
  return GP2D12Leftdist;
}

int read_lower_IR (byte pin) {
  while (LowerIRdataIndex < numSamples) {
    LowerIRdata[LowerIRdataIndex++] = read_gp2d12_range(pin);
  }
  LowerIRdist = filterSum(LowerIRdata, &LowerIRdataIndex);
  LowerIRdist = LowerIRdist/LowerIRdataIndex;
  LowerIRdataIndex = 0;
  return LowerIRdist;
}

int read_right_IR (byte pin) {
  while (GP2D12RightdataIndex < numSamples) {
    GP2D12Rightdata[GP2D12RightdataIndex++] = read_gp2d12_range(pin);
  }
  GP2D12Rightdist = filterSum(GP2D12Rightdata, &GP2D12RightdataIndex);
  GP2D12Rightdist = GP2D12Rightdist/GP2D12RightdataIndex;
  GP2D12RightdataIndex = 0;
  return GP2D12Rightdist;
}

int read_US (byte pin) {
  while (ULdataIndex < numSamples) {
    ULdata[ULdataIndex++] = read_ul_sensor_range(pin);
  }
  ULdist = filterSum(ULdata, &ULdataIndex);
  ULdist = ULdist/ULdataIndex;
  ULdataIndex = 0;
  return ULdist;
}

int read_long_IR (byte pin) {
  while (LongIRdataIndex < numSamples) {
    LongIRdata[LongIRdataIndex++] = read_IR_long_range(pin);
  }
  LongIRdist = filterSum(LongIRdata, &LongIRdataIndex);
  LongIRdist = LongIRdist/LongIRdataIndex;
  LongIRdataIndex = 0;
  return LongIRdist;
}

int filterSum(int* dataArray, int* index) {
  int i;
  int sum = 0;
  for (i = 0; i < numSamples; i++) {
    if (*(dataArray + i) != -1) {
      sum += *(dataArray + i);
    }
    else {
      *index =- 1;
    }
  }
  if (sum == 0) {
    sum = -1;
    *index = 1;
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
  if (pin == topLongIRpin) {
    range = (9462.0 /((float)tmp - 16.92)) + 12; //About 6cm too long at long range
  }
  else if (pin == bottomIRpin) {
    range = 30431 * pow(float(tmp), -1.169) + 1; //May want look-up table as pow slow
  }
  else {
    range = (9462.0 /((float)tmp - 16.92));
  }
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
  if ((tmp < 6) || (range > 30)) {
    range =  -1; // Error value
  }
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
  return range;
}
