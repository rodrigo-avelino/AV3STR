/*
 * keypad.h
 *
 *  Created on: May 24, 2025
 *      Author: rodri
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include "stdint.h"

#define KEY_RIGHT 	0
#define KEY_UP 		1
#define KEY_DOWN 	2
#define KEY_LEFT 	3
#define KEY_SELECT 	4
#define KEY_NONE 	5

uint16_t keypad_read_key(uint32_t adc_readout);



#endif /* INC_KEYPAD_H_ */
