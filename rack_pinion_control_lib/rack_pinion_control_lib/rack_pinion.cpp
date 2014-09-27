/** 
 * @file rack_pinion.cpp
 * @author Daniel Morris
 * @date 27/09/2014
 * @brief Implementation of code to control movement of the rack and pinion
 * for the collection tray via a stepper motor
 **/

#include "rack_pinion.h"

static const float DEGREE_PER_STEP = 1.8;

RP::RP(int dirpin, int steppin) {
	_dirpin = dirpin;
	_steppin = steppin;
	pinMode(_dirpin, OUTPUT);
	pinMode(_steppin, OUTPUT);
	current_state = unknown;
}

void RP::RP_dir_forward(void) {
	current_state = forwards;
	digitalWrite(_dirpin, LOW);
}

void RP::RP_dir_backward(void) {
	current_state = backwards;
	digitalWrite(_dirpin, HIGH);
}

void RP::move_RP(int dist) {
	int j = 0;
    int steps = (dist/DEGREE_PER_STEP) * 2;
    for (j = 0; j < steps; j++) {
		digitalWrite(_steppin, LOW);
        delayMicroseconds(1);
        digitalWrite(_steppin, HIGH);
        delay(1);
    }
}

TRAY_STATE RP::get_current_dir(void) {
	return current_state;
}
