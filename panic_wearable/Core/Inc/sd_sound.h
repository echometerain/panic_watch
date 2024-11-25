/*
 * sd_sound.h
 *
 *  Created on: Nov 19, 2024
 *      Author: hhwl
 */

#ifndef INC_SD_SOUND_H_
#define INC_SD_SOUND_H_

#include "stm32f4xx_hal.h"

#define SAMP_RATE 8000

typedef enum {
	UNKNOWN, HALF_COMPLETED, FULL_COMPLETED
} CallBack_Result_t;

//void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
//void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void music_init();
void music_start();
void music_stop();
void music_loop();

#endif /* INC_SD_SOUND_H_ */
