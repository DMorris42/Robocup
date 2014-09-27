/**
 * @file rack_pinion.h
 * @author Daniel Morris
 * @date 27/09/2014
 * @brief Code for controlling the rack and pinion weight collection
 * mechanism via a stepper motor
 **/ 

#ifndef RACK_PINION_H
#define RACK_PINION_H

#include "Arduino.h"

enum TRAY_STATE {
    forwards,
	backwards,
	unknown
};

class RP {
	public:
	
	//Constructor
	RP(int dirpin, int steppin);
	
	//Direction setting
	void RP_dir_forward(void);
	void RP_dir_backward(void);
	
	//Move rack and pinion
	void move_RP(int dist);
	
	//Fetcher for current direction of motion
	TRAY_STATE get_current_dir(void);
	
	TRAY_STATE current_state;
	
	private:
	
	int _dirpin;
	int _steppin;
};

#endif
