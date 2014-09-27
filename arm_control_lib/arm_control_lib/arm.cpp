/** 
 * @file arm.cpp
 * @author Daniel Morris
 * @date 27/09/2014
 * @brief Implementation of code to control movement of the actuator arm
 * via a stepper motor
 **/
 
#include "Arduino.h"
#include "arm.h" 

static const float DEGREE_PER_STEP = 1.8;

Arm::Arm(int dirpin, int steppin) {
	_dirpin = dirpin;
	_steppin = steppin;
	pinMode(_dirpin, OUTPUT);
	pinMode(_steppin, OUTPUT);
}

void Arm::arm_dir_up(void) {
	digitalWrite(_dirpin, HIGH);
}

void Arm::arm_dir_down(void) {
	digitalWrite(_dirpin, LOW);
}

void Arm::move_arm(unsigned int angle) {
	unsigned int j = 0;
	unsigned int steps = (angle/DEGREE_PER_STEP) * GEAR_RATIO * 2;
	for (j = 0; j < steps; j++) {
		digitalWrite(_steppin, LOW);
        delayMicroseconds(1);
        digitalWrite(_steppin, HIGH);
        delay(1);
	}
}
