#include "main.h"

//pwm波产生函数，参数为定时器句柄与channel代号，channel为1，2，3，4
void pwm_generation(TIM_HandleTypeDef htim, uint8_t channel)
{
	switch(channel)
	{
	case 1:
		HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
		break;
	case 2:
		HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2);
		break;
	case 3:
		HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3);
		break;
	case 4:
		HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_4);
		break;
	default:
		Error_Handler();
	}
}


//pwm改变占空比函数，参数为定时器句柄和channel代号，channel为1，2，3，4；cycle为占空比×周期；
void pwm_change(TIM_HandleTypeDef htim, uint8_t channel,uint32_t cycle)
{
	switch(channel)
	{
	case 1:
		htim.Instance->CCR1=cycle;
		break;
	case 2:
		htim.Instance->CCR2=cycle;
		break;
	case 3:
		htim.Instance->CCR3=cycle;
		break;
	case 4:
		htim.Instance->CCR4=cycle;
		break;
	default:
		Error_Handler();
	}
}

//pwm波停止函数，参数为定时器句柄与channel代号，channel为1，2，3，4
void pwm_stop(TIM_HandleTypeDef htim, uint8_t channel)
{
	switch(channel)
		{
		case 1:
			HAL_TIM_PWM_Stop(&htim, TIM_CHANNEL_1);
			break;
		case 2:
			HAL_TIM_PWM_Stop(&htim, TIM_CHANNEL_2);
			break;
		case 3:
			HAL_TIM_PWM_Stop(&htim, TIM_CHANNEL_3);
			break;
		case 4:
			HAL_TIM_PWM_Stop(&htim, TIM_CHANNEL_4);
			break;
		default:
			Error_Handler();
		}
}
