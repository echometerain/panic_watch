/*
 * st_sound.c
 *
 *  Created on: Nov 19, 2024
 *      Author: hhwl
 */

#include "sd_sound.h"

#include "fatfs.h"

#include <stdlib.h>
#include <stdio.h>

static void music_setup();
static void music_reset();

// FatFS sound
// Code credit: https://www.youtube.com/watch?v=spVIZO-jbxE
static FATFS fs; // file system
static FIL fil; // file
static FRESULT fresult; // file result object
static int16_t samples[SAMP_RATE]; // song bytes
static I2S_HandleTypeDef *i2s; // i2s object

static uint32_t fread_size = 0; // always resets to 0 for some reason
static uint32_t recording_size = 0;
static uint32_t played_size = 0;

static CallBack_Result_t callback_result = UNKNOWN;

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) { // override
	callback_result = HALF_COMPLETED;
}
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) { // override
	callback_result = FULL_COMPLETED;
	played_size += SAMP_RATE;
}

void music_init(I2S_HandleTypeDef *i2s_local) {
	i2s = i2s_local; // store i2s pointer
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) {
		printf("ERROR!!! in mounting SD CARD...\n");
		fflush(stdout);
		return;
	} else {
		printf("SD CARD mounted successfully...\n");
		fflush(stdout);
	}
	music_setup();
}

static void music_setup() {
	fresult = f_open(&fil, "/music.wav", FA_READ);
	f_lseek(&fil, 40); // hard-coded file size descriptor (WAVE standard)
	f_read(&fil, &recording_size, 4, (UINT*) &fread_size); // read file size
	recording_size /= 2; // 8 bit -> 16 bit
	fresult = f_read(&fil, samples, 2 * SAMP_RATE, (UINT*) &fread_size); // read 16k bytes (1s of data)
	played_size = 40 + 2 * SAMP_RATE;
}

void music_start() {
	HAL_I2S_Transmit_DMA(i2s, (uint16_t*) samples, SAMP_RATE); // open data transmit hook
}

void music_stop() {
	HAL_I2S_DMAStop(i2s);
}

static void music_reset() {
	f_close(&fil);
	music_setup();
}

void music_loop() {
	if (callback_result == UNKNOWN) {
		return;
	}

	if (callback_result == HALF_COMPLETED) {
		// read SAMP_RATE amount of bytes
		// it's 16 bit audio so that's only half
		fresult = f_read(&fil, samples, SAMP_RATE, (UINT*) &fread_size);
		callback_result = UNKNOWN;
	}

	if (callback_result == FULL_COMPLETED) {
		// read other half
		fresult = f_read(&fil, &samples[SAMP_RATE / 2], SAMP_RATE,
				(UINT*) &fread_size);
		callback_result = UNKNOWN;
	}

	if (played_size >= recording_size) {
		music_stop();
		music_reset();
		music_start();
	}
}
