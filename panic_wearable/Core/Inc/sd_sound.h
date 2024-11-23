/*
 * sd_sound.h
 *
 *  Created on: Nov 19, 2024
 *      Author: hhwl
 */

#ifndef INC_SD_SOUND_H_
#define INC_SD_SOUND_H_

#include "fatfs_sd.h"
#include "diskio.h"
#include "user_diskio.h"
#include "ff.h"

#define SAMP_RATE 8000

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void music_init();
void music_setup();
void music_start();
void music_reset();
void music_stop();
void music_loop();

#endif /* INC_SD_SOUND_H_ */
