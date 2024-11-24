/*
 * sensor_utils.c
 *
 *  Created on: Nov 23, 2024
 *      Author: hhwl
 */
#include "sensor_utils.h"

arm_rfft_fast_instance_f32 fft;
ADC_HandleTypeDef *adc;

// heart rate drops by 3bpm before panic attack
const float PANIC_HR_DROP = 0.05f; // hz
// breathing rate drops by 1bpm before panic attack
const float PANIC_BR_DROP = 0.016f; // hz
const float PANIC_SKN_COND = 13.5f; // Microsiemens
const float BR_MAX = 50 / 60.0; // max breathing rate we'll test for
const float BR_MIN = 10 / 60.0;

const int RATE = 16; // read sensors 16 times per second
const int FFT_DURATION_ = FFT_BUFFER_SIZE / RATE; // takes 128 seconds to do an fft
const int PANIC_IN = 45 * 60 * RATE; // panic attack in 40 minutes
const int SKN_TICKS = RATE; // read skin conductance every second

const float VDD = 3.3; // STM Microcontroller's VDD voltage
const float R_KNOWN = 220; // Known resistance in the voltage divider
const int TICKS = 84; // read sensors every 84th clock tick
const int CLOCK_PRESCALE = 84000000 / RATE / TICKS - 1; // TIM3 prescale set to 62499

float prev_HR = 0;
float prev_BR = 0;
float HR_data[FFT_BUFFER_SIZE];
float BR_data[FFT_BUFFER_SIZE];
uint16_t skn_cond[FFT_DURATION]; // one per second
volatile uint16_t ADC_results[DMA_NUM_DATA]; // [HR, Skin, BR]
volatile bool ADC_complete = false;
uint16_t HRBR_incr = 0; // heart & breathing rate iterator
uint16_t skn_incr = 0; // skin conductance iterator
uint16_t panic_timer = 0;
bool panicking = false;

//void HAL_ADC_convCpltCallback(ADC_HandleTypeDef *hadc) {
//	ADC_complete = true;
//}

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
	if (skn_incr >= FFT_DURATION) {
		predict_panic();
		HRBR_incr = 0;
		skn_incr = 0;
	}
}

uint16_t freq2idx(float freq) {
	return FFT_BUFFER_SIZE * freq / RATE;
}

float idx2freq(uint16_t idx) {
	return RATE / ((float) FFT_BUFFER_SIZE) * idx;
}

// get maximum frequency in hz
float max_freq(float arr[], uint16_t min_idx, uint16_t max_idx) {
	float fft_output[FFT_BUFFER_SIZE];
	arm_rfft_fast_f32(&fft, arr, fft_output, FFT_FLAG);
	uint16_t max_mag = 0;
	uint16_t max_i = 0;
	for (uint16_t i = min_idx; i < max_idx; i++) {
		uint16_t mag = fft_output[2 * i] * fft_output[2 * i]
				+ fft_output[2 * i + 1] * fft_output[2 * i + 1];
		if (mag > max_mag) {
			max_mag = mag;
			max_i = i;
		}
	}
	return idx2freq(max_i);
}

float Calculate_Voltage(uint16_t adc_value) {
	return (float) adc_value / 4096 * VDD;
}

float Calculate_Resistance(uint16_t adc_value) {
	// Convert ADC value to voltage
	float V_out = Calculate_Voltage(adc_value);
	// Apply voltage divider formula to calculate unknown resistance
	float R_unknown = R_KNOWN * V_out / (VDD - V_out);
	return R_unknown;
}

float Calculate_MicroSiemens(uint16_t adc_value) {
	return 1000000 / Calculate_Resistance(adc_value);
}

// average skin conductance level function
float avg_SCL(uint16_t arr[], int cap) {
	float sum = 0;
	uint16_t real_cap = 0;
	for (uint16_t i = 0; i < cap; i++) {
		float SCL = Calculate_MicroSiemens(arr[i]);
		if (isinf(SCL) || isnan(SCL) || SCL <= 0) {
			continue;
		}
		sum += SCL;
		real_cap++;
	}
	return sum / real_cap;
}

void predict_panic() {
	float HR = max_freq(HR_data, 0, FFT_BUFFER_SIZE / 2);
	float BR = max_freq(BR_data, freq2idx(BR_MIN), freq2idx(BR_MAX));
	float avg_skn_cond = avg_SCL(skn_cond, FFT_DURATION);
	uint16_t count = (prev_HR - HR >= PANIC_HR_DROP)
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

uint16_t will_panic_in_min() { // user will panic in x minutes
	return panic_timer / 16 / 60;
}
