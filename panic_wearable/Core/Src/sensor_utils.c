/*
 * sensor_utils.c
 *
 *  Created on: Nov 23, 2024
 *      Author: hhwl
 */

#include "sensor_utils.h"

#include "main.h"
#include "arm_math.h"
#include "stm32f4xx_hal_adc.h"

#include <stdlib.h>
#include <stdio.h>

#define FFT_BUFFER_SIZE 2048
#define FFT_FLAG 0 // regular fft (not ifft)
#define FFT_DURATION 128 // takes 128 seconds to do an fft
#define DMA_NUM_DATA 3

static inline uint16_t freq2idx(float freq);
static inline float idx2freq(uint16_t idx);
static float max_freq(float arr[], uint16_t min_idx, uint16_t max_idx);
static inline float Calculate_Voltage(uint16_t adc_value);
static inline float Calculate_Resistance(uint16_t adc_value);
static inline float Calculate_MicroSiemens(uint16_t adc_value);
static float avg_SCL(uint16_t arr[], int cap);

static const float VDD = 3.78f; // STM Microcontroller's VDD voltage
static const float R_KNOWN = 92000; // Known resistance in the voltage divider
// heart rate drops by 3bpm before attack (/2 for edge cases)
static const float PANIC_HR_DROP = 3.0f / 60 / 2; // hz
// breathing rate drops by 1bpm before attack (/2 for edge cases)
static const float PANIC_BR_DROP = 1.0f / 60 / 2; // hz
static const float PANIC_SKN_COND = 13.5f; // Microsiemens

static const float BR_MAX = 50 / 60.0f; // max breathing rate we'll test for
static const float BR_MIN = 10 / 60.0f;

static const int RATE = FFT_BUFFER_SIZE / FFT_DURATION; // read sensors 16 times per second
static const int PANIC_IN = 45 * 60 * RATE; // panic attack in 45 minutes

// const int SKN_TICKS = RATE; // read skin conductance every second
// static const int TICKS = 84; // read sensors every 84th clock tick
// const int CLOCK_PRESCALE = 84000000 / RATE / TICKS - 1; // TIM3 prescale set to 62499

static arm_rfft_fast_instance_f32 fft;
static ADC_HandleTypeDef *adc;

static float prev_HR = 0;
static float prev_BR = 0;
static float HR_data[FFT_BUFFER_SIZE];
static float BR_data[FFT_BUFFER_SIZE];
static uint16_t skn_cond[FFT_DURATION]; // one per second
static volatile uint16_t ADC_results[DMA_NUM_DATA]; // [HR, Skin, BR]
volatile bool ADC_complete = false;
static uint16_t HRBR_incr = 0; // heart & breathing rate iterator
static uint16_t skn_incr = 0; // skin conductance iterator

static uint16_t panic_timer = 0;
bool user_is_panicking = false;
extern volatile bool panic_mode;

// tutorial followed: https://www.youtube.com/watch?v=d1KvgOwWvkM
void sensor_init(TIM_HandleTypeDef *timer, ADC_HandleTypeDef *local_adc) {
	adc = local_adc;
	HAL_TIM_Base_Start_IT(timer);
	arm_rfft_fast_init_f32(&fft, FFT_BUFFER_SIZE);
}

void timer_callback() {
	if (panic_timer > 0) { // panic prediction timer
		if (panic_timer == 1) {
			user_is_panicking = true;
		}
		panic_timer--;
		return;
	}
	if (panic_mode) { // don't record if playing music
		return;
	}
	ADC_complete = false;
	HAL_ADC_Start_DMA(adc, (uint32_t*) ADC_results, DMA_NUM_DATA);
	while (!ADC_complete)
		;
	HR_data[HRBR_incr] = (float) ADC_results[0] - 2048; // half of ADC range
	BR_data[HRBR_incr] = (float) ADC_results[2] - 2048; // half of ADC range
	if (HRBR_incr % 16 == 0) {
		skn_cond[skn_incr] = ADC_results[1];
		printf("time: %ds, HR: %f, BR: %f, skin: %fohm\n", skn_incr,
				(float) HR_data[HRBR_incr], (float) BR_data[HRBR_incr],
				Calculate_MicroSiemens(skn_cond[skn_incr]));
		fflush(stdout);
		skn_incr++;
	}
	HRBR_incr++;
	if (skn_incr >= FFT_DURATION) {
		predict_panic();
		HRBR_incr = 0;
		skn_incr = 0;
	}
}

static inline uint16_t freq2idx(float freq) { // frequency to fft index
	return FFT_BUFFER_SIZE * freq / RATE;
}

static inline float idx2freq(uint16_t idx) { // fft index to frequency
	return RATE / ((float) FFT_BUFFER_SIZE) * idx;
}

// get maximum frequency in hz
static float max_freq(float arr[], uint16_t min_idx, uint16_t max_idx) {
	float fft_output[FFT_BUFFER_SIZE];
	arm_rfft_fast_f32(&fft, arr, fft_output, FFT_FLAG);
	uint16_t max_mag = 0;
	uint16_t max_i = 0;
	for (uint16_t i = min_idx; i < max_idx; i++) {
		// get magnitude
		uint16_t mag = fft_output[2 * i] * fft_output[2 * i]
				+ fft_output[2 * i + 1] * fft_output[2 * i + 1];
		if (mag > max_mag) { // find max magnitude
			max_mag = mag;
			max_i = i;
		}
	}
	return idx2freq(max_i);
}

static inline float Calculate_Voltage(uint16_t adc_value) {
	return ((float) adc_value) / 4096 * VDD;
}

static inline float Calculate_Resistance(uint16_t adc_value) {
	// Convert ADC value to voltage
	float V_out = Calculate_Voltage(adc_value);
	// Apply voltage divider formula to calculate unknown resistance
	float R_unknown = R_KNOWN * (VDD - V_out) / V_out;
	return R_unknown;
}

static inline float Calculate_MicroSiemens(uint16_t adc_value) {
	return 1000000 / Calculate_Resistance(adc_value);
}

// average skin conductance level function
static float avg_SCL(uint16_t arr[], int cap) {
	float sum = 0;
	uint16_t real_cap = 0;
	for (uint16_t i = 0; i < cap; i++) {
		float SCL = Calculate_MicroSiemens(arr[i]);
		if (isinf(SCL) || isnan(SCL) || SCL <= 0) {
			continue; // drop invalid values
		}
		sum += SCL;
		real_cap++;
	}
	return sum / real_cap;
}

// predict panic attack 45 minutes in advance
void predict_panic() {
	float BR_MAX_IDX = freq2idx(BR_MAX);
	float BR_MIN_IDX = freq2idx(BR_MIN);
	float HR = max_freq(HR_data, 1, FFT_BUFFER_SIZE / 2); // 0th freq bin counts constants
	float BR = max_freq(BR_data, BR_MIN_IDX, BR_MAX_IDX); // from min to max normal BR
	float avg_skn_cond = avg_SCL(skn_cond, FFT_DURATION);
	uint16_t count = (prev_HR - HR >= PANIC_HR_DROP) // count # of indicators
	+ (prev_BR - BR >= PANIC_BR_DROP) + (avg_skn_cond >= PANIC_SKN_COND);
	if (count >= 2) {
		panic_timer = PANIC_IN; // in 45 minutes
		handle_will_panic();
	}
	prev_HR = HR;
	prev_BR = BR;
}

bool will_panic() {
	return panic_timer > 0;
}

uint16_t will_panic_in_min() { // user will panic in x minutes
	return panic_timer / 16 / 60;
}
