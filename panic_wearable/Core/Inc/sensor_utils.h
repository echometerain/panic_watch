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
#define FFT_DURATION 128 // takes 128 seconds to do an fft
#define ARM_MATH_CM4
#define DMA_NUM_DATA 3
//#define __FPU_PRESENT  1U

#include "main.h"
#include "arm_math.h"
#include <stdbool.h>
#include <math.h>

void sensor_init(TIM_HandleTypeDef *timer, ADC_HandleTypeDef *local_adc);
void timer_callback();
uint16_t freq2idx(float freq);
float idx2freq(uint16_t idx);
float max_freq(float arr[], uint16_t min_idx, uint16_t max_idx);
float Calculate_Voltage(uint16_t adc_value);
float Calculate_Resistance(uint16_t adc_value);
float Calculate_MicroSiemens(uint16_t adc_value);
float avg_SCL(uint16_t arr[], int cap);
void predict_panic();
bool user_is_panicking();
bool will_panic();
uint16_t will_panic_in_min();

#endif /* INC_SENSOR_UTILS_H_ */
