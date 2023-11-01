
#include "ultrasonic.h"
#include "main.h"

#define sound_v 34//34cm每毫秒
#define unit_time 0.01//单位时间为0.01ms


void sensor_init(ULTRASONIC_SENSOR* sensor, TIM_HandleTypeDef htim)
{
	sensor->capture_flag=0;
	sensor->distance=0;
	sensor->falling_time=0;
	sensor->rising_time=0;
	sensor->capture_timer=htim;
}


uint32_t get_distance(ULTRASONIC_SENSOR* sensor)
{
	uint32_t result;

	switch(sensor->capture_flag)
	{
	case 3://表示捕捉到了
		result = (sensor->falling_time - sensor->rising_time) * sound_v;
		result = result * unit_time; //计算距离
		result = result/2;//一来一回
		break;
	default://表示未捕捉到
		result = 65535;
		break;
	}

	sensor->capture_flag=0;//旗帜归零
	sensor->distance = result;

	return result;
}

void send_trigger()
{
	HAL_GPIO_WritePin(GPIOB, trigger_Pin,GPIO_PIN_SET);
	for(int i=0;i<100;i++){}//延时
	HAL_GPIO_WritePin(GPIOB, trigger_Pin,GPIO_PIN_RESET);
}

void rising_capture_begin(ULTRASONIC_SENSOR* sensor)//开始捕捉上升沿回波，同时将标识置为1
{
	__HAL_TIM_SET_CAPTUREPOLARITY(&sensor->capture_timer, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
	HAL_TIM_IC_Start_IT(&sensor->capture_timer, TIM_CHANNEL_3);
	sensor->capture_flag=1;
}


void falling_capture_begin(ULTRASONIC_SENSOR* sensor)
{
	__HAL_TIM_SET_CAPTUREPOLARITY(&sensor->capture_timer,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
	HAL_TIM_IC_Start_IT(&sensor->capture_timer, TIM_CHANNEL_3);
	sensor->capture_flag=2;
}

void get_common_rising_time(ULTRASONIC_SENSOR* sensor,uint32_t common_rising_time)
{
	sensor->rising_time = common_rising_time;//将统一的上升沿时间进行赋值
	falling_capture_begin(sensor);//改变flag，同时开启向下捕捉
}


void get_falling_time(ULTRASONIC_SENSOR* sensor,TIM_HandleTypeDef clock_timer)
{
	if(sensor->capture_flag==2)
	{
		sensor->falling_time = clock_timer.Instance->CNT;//获取当前的时间
		HAL_TIM_IC_Stop_IT(&sensor->capture_timer,TIM_CHANNEL_3); //停止捕获
		sensor->capture_flag = 3;
	}

}
