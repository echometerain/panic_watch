/*
 * sensor_utils.c
 *
 *  Created on: Nov 23, 2024
 *      Author: hhwl
 */
#include "sensor_utils.h"

arm_rfft_fast_instance_f32 fft;
ADC_HandleTypeDef *adc;

float prev_HR = 0;
float prev_BR = 0;
float HR_data[FFT_BUFFER_SIZE];
float BR_data[FFT_BUFFER_SIZE];
uint16_t skn_cond[FFT_SECONDS]; // one per second
volatile uint16_t ADC_results[DMA_NUM_DATA]; // [HR, Skin, BR]
volatile bool ADC_complete = false;
uint HRBR_incr = 0; // heart & breathing rate iterator
uint skn_incr = 0; // skin conductance iterator
uint panic_timer = 0;
bool panicking = false;

void HAL_ADC_convCpltCallback(ADC_HandleTypeDef *hadc) {
	ADC_complete = true;
}

// tutorial followed: https://www.youtube.com/watch?v=d1KvgOwWvkM
void sensor_init(TIM_HandleTypeDef *timer, ADC_HandleTypeDef *local_adc) {
	adc = local_adc;
	HAL_TIM_Base_Start_IT(timer);
	arm_rfft_fast_init_f32(&fft, FFT_BUFFER_SIZE);
}

void timer_callback() {
	if (panic_timer > 0) {
		if (panic_timer == 1) {
			panicking = true;
		}
		panic_timer--;
		return;
	}
	ADC_complete = 0;
	HAL_ADC_Start_DMA(adc, (uint32_t*) ADC_results, DMA_NUM_DATA);
	while (!ADC_complete)
		;
	HR_data[HRBR_incr] = (float) ADC_results[0];
	BR_data[HRBR_incr] = (float) ADC_results[2];
	HRBR_incr++;
	if (HRBR_incr % 16 == 0) {
		skn_cond[skn_incr] = ADC_results[1];
		skn_incr++;
	}
	if (skn_incr >= FFT_SECONDS) {
		predict_panic();
		HRBR_incr = 0;
		skn_incr = 0;
	}
}

inline uint freq2idx(float freq) {
	return FFT_BUFFER_SIZE * freq / RATE;
}

inline float idx2freq(uint idx) {
	return RATE / ((float) FFT_BUFFER_SIZE) * idx;
}

// get maximum frequency in hz
float max_freq(float arr[], uint min_idx, uint max_idx) {
	float fft_output[FFT_BUFFER_SIZE];
	arm_rfft_fast_f32(&fft, arr, fft_output, FFT_FLAG);
	uint16_t max_mag = 0;
	uint max_i = 0;
	for (uint i = min_idx; i < max_idx; i++) {
		uint mag = fft_output[2 * i] * fft_output[2 * i]
				+ fft_output[2 * i + 1] * fft_output[2 * i + 1];
		if (mag > max_mag) {
			max_mag = mag;
			max_i = i;
		}
	}
	return idx2freq(max_i);
}

// average helper function
float avg(uint16_t arr[], int cap) {
	uint32_t sum = 0;
	float real_cap = 0;
	for (uint i = 0; i < cap; i++) {
		if (arr[i] == 0) {
			continue;
		}
		sum += arr[i];
		real_cap++;
	}
	return sum / real_cap;
}

void predict_panic() {
	float HR = max_freq(HR_data, 0, FFT_BUFFER_SIZE / 2);
	float BR = max_freq(BR_data, freq2idx(BR_MIN), freq2idx(BR_MAX));
	float avg_skn_cond = avg(skn_cond, FFT_SECONDS);
	uint count = (prev_HR - HR >= PANIC_HR_DROP)
			+ (prev_BR - BR >= PANIC_BR_DROP)
			+ (avg_skn_cond >= PANIC_SKN_COND);
	if (count >= 2) {
		panic_timer = PANIC_IN;
	}
}

bool user_is_panicking() {
	return panicking;
}

bool will_panic() {
	return panic_timer > 0;
}

uint will_panic_in_min() { // user will panic in x minutes
	return panic_timer / 16 / 60;
}
