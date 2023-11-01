
#include "main.h"
#include "motor.h"

void send_current_speed(MOTOR* motor,UART_HandleTypeDef* huart)
{
	int16_t speed;
	uint8_t data[3];//第一位是正负标志位，第二位为数据位，第三位为帧尾
	data[2]=0xaa;
	speed = motor->current_speed;

	if(speed>=0)
	{
		data[0]=0;//符号位为0，就是正
		data[1]=speed&0xff;
	}
	else
	{
		data[0]=1;//符号位为1，就是负
		speed=-speed;
		data[1]=speed&0xff;
	}

	HAL_UART_Transmit(huart, data, 3, 1000);
}

int16_t get_sign(uint8_t flag, uint8_t data)
{
	int16_t result;
	if(flag == 0x01)
		result = -data;
	else if(flag == 0x00)
		result = data;
	else
		Error_Handler();
	return result;
}
