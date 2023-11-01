
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "pwm.h"
#include "pid.h"
#include "bluetooth.h"
#include "motor.h"
#include "ultrasonic.h"
#include "servo.h"

#define camera_channel 1
#define grabber_sg90 2
#define grabber_mg995 4
#define zero_angle_point 45
//该定时器的周期为20ms，区间为1800，计数器每计一次代表1/90ms，代表1°

void grabber_lift(uint32_t angle)
{
	pwm_change(htim8, grabber_mg995, angle+zero_angle_point);
	pwm_generation(htim8, grabber_mg995);
}

void grabber_move(uint32_t angle)
{
	pwm_change(htim8, grabber_sg90, angle+zero_angle_point);
	pwm_generation(htim8, grabber_sg90);
}

void camera_lift(uint32_t angle)
{
	pwm_change(htim8, camera_channel, angle+zero_angle_point);
	pwm_generation(htim8, camera_channel);
}

void grab()
{
	grabber_move(90);
	for(int i=0;i<40;i++)
	{
		grabber_lift(i+20);
	}
}

void drop()
{
	for(int i=0;i<30;i++)
	{
		grabber_lift(43-i);
	}
	for(int i=0;i<100;i++){}
	grabber_move(15);
}
