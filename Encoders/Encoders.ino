/* Reads encoder interrupts from wheels and increments counters
Has no sense of direction though; will need movement state for each wheel
*/

//663 Pulses per revolution, but quadrature so need to double (?). Seems to work better.
static const int ENCODER_COUNTS = 663*2;
//Degrees per count
static const float ENCODER_COUNT_ANGLE = 0.54;

volatile unsigned int encoderPosLeft = 0;
volatile unsigned int lastReportedPosLeft = 1;
volatile unsigned int encoderPosRight = 0;
volatile unsigned int lastReportedPosRight = 1;

volatile unsigned int revolutionsLeft = 0;
volatile unsigned int revolutionsRight = 0;
volatile unsigned int lastReportedRevolutionsLeft = 0;
volatile unsigned int lastReportedRevolutionsRight = 0;


void setup() {
// Encoder pin on interrupt 4
  attachInterrupt(4, doEncoderRight, CHANGE);
// Encoder pin on interrupt 5
  attachInterrupt(5, doEncoderLeft, CHANGE);
  Serial.begin(9600);
}


void loop()
{   
  /*if ((lastReportedPosLeft != encoderPosLeft) || (lastReportedPosRight != encoderPosRight)) {
    Serial.print("Left/Right:");
    Serial.print(encoderPosLeft, DEC);
    Serial.print(":");
    Serial.print(encoderPosRight, DEC);
    Serial.println();
    lastReportedPosLeft = encoderPosLeft;
    lastReportedPosRight = encoderPosRight;
  }*/
  if ((lastReportedRevolutionsLeft != revolutionsLeft) || (lastReportedRevolutionsRight != revolutionsRight)) {
    Serial.print("Left rev/Right rev:");
    Serial.print(revolutionsLeft, DEC);
    Serial.print(":");
    Serial.print(revolutionsRight, DEC);
    Serial.println();
    lastReportedRevolutionsLeft = revolutionsLeft;
    lastReportedRevolutionsRight = revolutionsRight;
  }
}

// Interrupt on A changing state
void doEncoderLeft(){
  encoderPosLeft++;
  if (encoderPosLeft == ENCODER_COUNTS) {
    encoderPosLeft = 0;
    revolutionsLeft++;
  }
}


// Interrupt on A changing state
void doEncoderRight(){
  encoderPosRight++;
  if (encoderPosRight == ENCODER_COUNTS) {
    encoderPosRight = 0;
    revolutionsRight++;
  }
}

