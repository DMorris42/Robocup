/* Code for using the ultrasonic sensor
   Sensor has a minimum range of 30 cm!
*/

int pin = 8; //Attached to A5
float distance = 0;

void setup() {
  Serial.begin(9600);

}

void loop() {
  distance = read_ul_sensor_range();
  Serial.println(distance);
  delay(500);
}

float read_ul_sensor_range(void) {
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
    
