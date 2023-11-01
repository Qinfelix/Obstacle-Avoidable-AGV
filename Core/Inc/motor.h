

//关键头文件，定义了电机结构体，和电机使能函数与控制函数

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "pid.h"

typedef struct
{
	int16_t expect_speed;
	int16_t current_speed;
	PID_CONTROLLER controller;
	TIM_HandleTypeDef encoder_timer;
	TIM_HandleTypeDef pwm_timer;
	uint8_t pwm_channel;
	uint8_t in_pin;
	uint8_t rotate_flag;//表示旋转正负，0表示目标速度和当前速度均为正，1表示均由正转负，2表示均为负，3表示目标为负，当前为正
	uint32_t pulse_number;//表示转一圈的脉冲数
} MOTOR;

void nojtag();//取消jtag，使得TIM2的两个通道可以重新复用

void motor_init(MOTOR* motor,TIM_HandleTypeDef pwm_timer,TIM_HandleTypeDef encoder_timer,uint8_t pwm_channel, uint8_t in_pin,uint32_t pulse_number,uint8_t control_frequency);
void motor_positive(MOTOR* motor);//电机正转
void motor_negative(MOTOR* motor);//电机反转
void motor_start(MOTOR* motor);//电机使能，输出pwm波，开环控制
void control_start(MOTOR* motor,TIM_HandleTypeDef control_timer);//控制使能


void get_rotate_flag(MOTOR* motor);
int16_t get_current_speed(MOTOR* motor);//返回一个电机的当前速度
void motor_control(MOTOR* motor);//闭环控制的所有步骤：检测速度，计算控制量，输出pwm波

#endif
