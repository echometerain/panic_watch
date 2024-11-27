/*
 * sensor_utils.h
 *
 *  Created on: Nov 23, 2024
 *      Author: hhwl
 */

#ifndef INC_SENSOR_UTILS_H_
#define INC_SENSOR_UTILS_H_

#include "stm32f4xx_hal.h"

#include <stdbool.h>

#define ARM_MATH_CM4

void sensor_init(TIM_HandleTypeDef *timer, ADC_HandleTypeDef *local_adc);
void timer_callback();
void predict_panic();
bool will_panic();
char will_panic_in_min();

#endif /* INC_SENSOR_UTILS_H_ */
