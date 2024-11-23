/*
 * sensor_utils.h
 *
 *  Created on: Nov 23, 2024
 *      Author: hhwl
 */

#ifndef INC_SENSOR_UTILS_H_
#define INC_SENSOR_UTILS_H_

#define FFT_BUFFER_SIZE 2048
#define FFT_FLAG 0 // regular fft (not ifft)
#define ARM_MATH_CM7
#define DMA_NUM_DATA 3

#include "arm_math.h"
#include <stdbool.h>

// heart rate drops by 3bpm before panic attack
const float PANIC_HR_DROP = 0.05f; // hz
// breathing rate drops by 1bpm before panic attack
const float PANIC_BR_DROP = 0.016f; // hz
const float PANIC_SKN_COND = 13.5f;
const float BR_MAX = 50 / 60.0;
const float BR_MIN = 10 / 60.0;
const int PANIC_IN = 45 * 60 * RATE; // panic attack in 40 minutes
const int RATE = 16; // read sensors 16 times per second
const int SKN_TICKS = RATE; // read skin conductance every second
const int FFT_SECONDS = FFT_BUFFER_SIZE / RATE; // takes 128 seconds to do an fft
const int CLOCK_PRESCALE = 84000000 / RATE / TICKS - 1; // TIM3 prescale set to 62499
const int TICKS = 84; // read sensors every 84th clock tick

void sensor_init(TIM_HandleTypeDef *timer, ADC_HandleTypeDef *local_adc);
void timer_callback();
float fft_argmax(uint16_t arr[], uint min_freq, uint max_freq);
bool predict_panic();

#endif /* INC_SENSOR_UTILS_H_ */
