//EncoderC

#include <Servo.h>

Servo myservo1,myservo2;  // create servo object to control a servo

enum PinAssignments {
  encoder1PinA = 19,
  encoder1PinB = 24,
  encoder2PinA = 18,
  encoder2PinB = 25,
};

volatile unsigned int encoderPosA = 0;
volatile unsigned int lastReportedPosA = 1;
volatile unsigned int encoderPosB = 0;
volatile unsigned int lastReportedPosB = 1;


void setup() {
  myservo1.attach(12);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(13);

// Encoder pin on interrupt 4
  attachInterrupt(4, doEncoder1A, CHANGE);
// Encoder pin on interrupt 5
  attachInterrupt(5, doEncoder2A, CHANGE);
  Serial.begin(9600);
}


void loop()
{   
  if ((lastReportedPosA != encoderPosA) || (lastReportedPosB != encoderPosB)) {
    Serial.print("Index:");
    Serial.print(encoderPosA, DEC);
    Serial.print(":");
    Serial.print(encoderPosB, DEC);
    Serial.println();
    lastReportedPosA = encoderPosA;
    lastReportedPosB = encoderPosB;
  }
}

// Interrupt on A changing state
void doEncoder1A(){
  encoderPosA++;
}


// Interrupt on A changing state
void doEncoder2A(){
  encoderPosB++;
}

