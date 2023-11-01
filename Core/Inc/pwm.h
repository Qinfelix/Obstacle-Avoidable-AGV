#ifndef PWM_H
#define PWM_H
#include "main.h"

void pwm_generation(TIM_HandleTypeDef htim, uint8_t channel);
void pwm_change(TIM_HandleTypeDef htim, uint8_t channel,uint32_t cycle);
void pwm_stop(TIM_HandleTypeDef htim, uint8_t channel);

#endif
