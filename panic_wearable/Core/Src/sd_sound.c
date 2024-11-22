/*
 * st_sound.c
 *
 *  Created on: Nov 19, 2024
 *      Author: hhwl
 */

#include "sd_sound.h"

// FatFS sound
// Code credit: https://www.youtube.com/watch?v=spVIZO-jbxE
FATFS fs; // file system
FIL fil; // file
FRESULT fresult; // file result object
int16_t samples[SAMP_RATE]; // song bytes

uint32_t fread_size = 0;
uint32_t recording_size = 0;
uint32_t played_size = 0;

CallBack_Result_t callback_result = UNKNOWN;

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_result = HALF_COMPLETED;
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	callback_result = FULL_COMPLETED;
	played_size += SAMP_RATE;
}

// Code credit: https://www.youtube.com/watch?v=spVIZO-jbxE
void music_init(I2S_HandleTypeDef i2s) {
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) {
		printf("ERROR!!! in mounting SD CARD...\n");
		fflush(stdout);
		return;
	} else {
		printf("SD CARD mounted successfully...\n");
		fflush(stdout);
	}

	fresult = f_open(&fil, "/music.wav", FA_READ);
	f_lseek(&fil, 40); // hard-coded file size descriptor (WAVE standard)
	f_read(&fil, &recording_size, 4, (UINT*) &fread_size);
	recording_size /= 2; // 16 bit
	fresult = f_read(&fil, samples, 2 * SAMP_RATE, (UINT*) &fread_size); // read 16k bytes (1s of data)
	played_size = 40 + 2 * SAMP_RATE;
	HAL_I2S_Transmit_DMA(&i2s, (uint16_t*) samples, SAMP_RATE); // open data transmit hook
}

// Code credit: https://www.youtube.com/watch?v=spVIZO-jbxE
void music_loop(I2S_HandleTypeDef i2s) {
	if (callback_result == HALF_COMPLETED) {
		// read SAMP_RATE amount of bytes
		// it's 16 bit audio so that's only half
		fresult = f_read(&fil, samples, SAMP_RATE, (UINT*) &fread_size);
		fflush(stdout);
		callback_result = UNKNOWN;
	}

	if (callback_result == FULL_COMPLETED) {
		fresult = f_read(&fil, &samples[SAMP_RATE / 2], SAMP_RATE,
				(UINT*) &fread_size);
		fflush(stdout);
		callback_result = UNKNOWN;
	}

	if (played_size >= recording_size) {
		HAL_I2S_DMAStop(&i2s);
	}
}
