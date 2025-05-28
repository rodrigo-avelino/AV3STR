/*
 * keypad.c
 *
 *  Created on: May 24, 2025
 *      Author: rodri
 */
#include "keypad.h"
uint16_t keypad_read_key(uint32_t adc_readout){

	if(adc_readout > 800 && adc_readout < 850){
		return KEY_UP;
	}
	else if(adc_readout > 1900 && adc_readout < 2050){
		return KEY_DOWN;
	}
	else if(adc_readout > 3000 && adc_readout < 3150){
		return KEY_LEFT;
	}
	else if(adc_readout >= 0 && adc_readout < 50){
		return KEY_RIGHT;
	}
	else if(adc_readout > 4000 && adc_readout < 5050){
		return KEY_SELECT;
	}
	return KEY_NONE;
}


