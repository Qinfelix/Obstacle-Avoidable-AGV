
//该文件中定义了PID控制器的相关结构体，主要是控制中的数学运算
#ifndef PID_H
#define PID_H
#include "main.h"


typedef struct
{
	int32_t pwm_para;
	float kp;
	float ki;
	float kd;
	int32_t error;
	int32_t error_last;
	int32_t error_integral;
	int32_t control;

	uint8_t control_frequency;
} PID_CONTROLLER;

void pid_controller_init(PID_CONTROLLER* pid, uint8_t sys_control_frequency,float kp,float ki,float kd);
void pid_control(PID_CONTROLLER* pid);// 根据更新过的error，计算control量，并加和到pwm——para上


#endif
