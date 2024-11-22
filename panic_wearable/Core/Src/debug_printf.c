/*
 * debug_printf.c
 *
 *  Created on: Nov 22, 2024
 *      Author: hhwl
 */

#include "debug_printf.h"

// Debugging output
// Code credit (Subzee): https://stackoverflow.com/questions/69695956/printing-in-c-to-ide-console-on-stm32cubeide
// https://www.youtube.com/watch?v=WLqUImiV5Gs
int _write(int file, char *ptr, int len) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
