
#include "main.h"
#include "pid.h"
#include "motor.h"
#include "pwm.h"

void nojtag()
{
	__HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();           	//�???????????????????????????启AFIO时钟
	__HAL_AFIO_REMAP_SWJ_NOJTAG();				//禁用JTAG
}


void motor_init(MOTOR* motor,TIM_HandleTypeDef pwm_timer,TIM_HandleTypeDef encoder_timer,uint8_t pwm_channel, uint8_t in_pin,uint32_t pulse_number,uint8_t control_frequency)
{
	pid_controller_init(&motor->controller, control_frequency,5,0,0.1);
	motor->expect_speed=0;
	motor->current_speed=0;

	motor->pwm_timer=pwm_timer;
	motor->encoder_timer=encoder_timer;
	motor->pwm_channel=pwm_channel;
	motor->in_pin=in_pin;
	motor->pulse_number=pulse_number;

}


void motor_positive(MOTOR* motor)
{
	switch(motor->in_pin)
	{
	case 1:
		HAL_GPIO_WritePin(GPIOD, motor1_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, motor1_in2_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, motor2_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, motor2_in2_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, motor3_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, motor3_in2_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOD, motor4_in1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, motor4_in2_Pin, GPIO_PIN_SET);
		break;
	default:
		Error_Handler();
	}
}

void motor_negative(MOTOR* motor)
{
	switch(motor->in_pin)
	{
	case 1:
		HAL_GPIO_WritePin(GPIOD, motor1_in1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, motor1_in2_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, motor2_in1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, motor2_in2_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, motor3_in1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, motor3_in2_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOD, motor4_in1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, motor4_in2_Pin, GPIO_PIN_RESET);
		break;
	default:
		Error_Handler();
	}

}

void motor_start(MOTOR* motor)
{
	motor_positive(motor);
	pwm_generation(motor->pwm_timer, motor->pwm_channel);
}


void control_start(MOTOR* motor,TIM_HandleTypeDef control_timer)
{
	HAL_TIM_Encoder_Start(&motor->encoder_timer, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&control_timer);// begin the control timer in interrupt mode
}

void get_rotate_flag(MOTOR* motor)
{
	if(motor->expect_speed>0)
	{
		if(motor->current_speed>=0)
			motor->rotate_flag=0;//均为正
		else
			motor->rotate_flag=1;//由负转正
	}
	else if(motor->expect_speed<0)
	{
		if(motor->current_speed<=0)
			motor->rotate_flag=2;//均为负
		else
			motor->rotate_flag=3;//由正转负
	}
	else if(motor->expect_speed==0)//刹车状态
	{
		if(motor->current_speed>=0)
			motor->rotate_flag=4;//正向刹车
		else
			motor->rotate_flag=5;//负向刹车
	}
}

int16_t get_current_speed(MOTOR* motor)
{
	int16_t counter;//取出计数器值
	uint32_t calculate_number;//中间计算值，防止溢出
	int16_t current_speed;//最终的返回值

	counter =(int16_t)(__HAL_TIM_GET_COUNTER(&motor->encoder_timer));

	if(counter<0)
	{
		calculate_number = - counter;//取绝对值
		calculate_number = calculate_number * motor->controller.control_frequency * 60;// per minute
		current_speed = calculate_number / motor->pulse_number ;//xx pulse for a physical circle
		current_speed = - current_speed;//保证输出负数
	}
	else
	{
		calculate_number =  counter;
		calculate_number = calculate_number *motor->controller.control_frequency * 60;// per minute
		current_speed = calculate_number / motor->pulse_number ;//xx pulse for a physical circle
	}

	motor->encoder_timer.Instance->CNT=0;// count again

    return current_speed;//
}

void motor_control(MOTOR* motor)
{
	motor->current_speed=get_current_speed(motor);//更新当前速度

	get_rotate_flag(motor);//判断此时的旋转顺序以及是否需要反转

	switch(motor->rotate_flag)
	{
	case 0://均为正
		motor->controller.error=motor->expect_speed-motor->current_speed;
		pid_control(&motor->controller);
		pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		break;

	case 1://由负转正
		if(motor->current_speed > -40)//速度够小，可以直接反向
		{
			motor_positive(motor);
		}
		else//速度不够小，pid控制电机减速，视为目标速度为0，但是不改变结构体中值（只有外部能改变）
		{
			motor->controller.error=motor->current_speed - 0;
			pid_control(&motor->controller);
			pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		}
		break;

	case 2://均为负
		motor->controller.error=motor->current_speed - motor->expect_speed;
		pid_control(&motor->controller);
		pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		break;

	case 3://由正转负
		if(motor->current_speed < 40)//速度够小，可以直接反向
		{
			motor_negative(motor);
		}
		else//速度不够小，pid控制电机减速，视为目标速度为0，但是不改变结构体中值（只有外部能改变）
		{
			motor->controller.error=0 - motor->current_speed;
			pid_control(&motor->controller);
			pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		}
		break;

	case 4://正向刹车
		if(motor->current_speed < 40)//速度够小，可以直接刹车
		{
			motor->controller.pwm_para=0;
			pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		}
		else//速度不够小，pid控制电机减速，视为目标速度为0，但是不改变结构体中值（只有外部能改变）
		{
			motor->controller.error=0 - motor->current_speed;
			pid_control(&motor->controller);
			pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		}
		break;

	case 5://反向刹车
		if(motor->current_speed > -40)//速度够小，可以直接反向
		{
			motor->controller.pwm_para=0;
			pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		}
		else//速度不够小，pid控制电机减速，视为目标速度为0，但是不改变结构体中值（只有外部能改变）
		{
			motor->controller.error=motor->current_speed - 0;
			pid_control(&motor->controller);
			pwm_change(motor->pwm_timer, motor->pwm_channel, motor->controller.pwm_para);
		}
		break;
	}
}
