#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include "main.h"

typedef struct
{
	uint32_t rising_time;//捕捉到eco波上升沿时的时间
	uint32_t falling_time;//捕捉到eco波下降沿时的时间
	uint32_t capture_flag;
	//表示是否捕捉到回波的旗帜,0表示未发出trigger波，1表示发出trigger波,等待上升沿，2表示收到上升沿，等待下降沿，3表示均捕捉到
	uint32_t distance;
	TIM_HandleTypeDef capture_timer;//捕捉定时器


} ULTRASONIC_SENSOR;

void sensor_init(ULTRASONIC_SENSOR* sensor,TIM_HandleTypeDef htim);//对超声波进行初始化
uint32_t get_distance(ULTRASONIC_SENSOR* sensor);
void send_trigger(); //send the trigger wave
void rising_capture_begin(ULTRASONIC_SENSOR* sensor);//开始捕捉上升沿
void falling_capture_begin(ULTRASONIC_SENSOR* sensor);//开始捕捉下降沿
void get_common_rising_time(ULTRASONIC_SENSOR* sensor,uint32_t common_rising_time);//赋值统一的上升沿时间
void get_falling_time(ULTRASONIC_SENSOR* sensor,TIM_HandleTypeDef clock_timer);//捕获下降沿的时间
#endif
