/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <math.h>
#include "pwm.h"
#include "pid.h"
#include "bluetooth.h"
#include "motor.h"
#include "ultrasonic.h"
#include "servo.h"
#include "obstacle.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define bluetooth_receive_start()  HAL_UART_Receive_IT(&huart1, receive, 6);
#define tracing_receive_start() HAL_UART_Receive_IT(&huart1, receive, 2);
#define sensor_receive_start() HAL_TIM_Base_Start_IT(&htim7);
#define imu_reset() HAL_UART_Transmit(&huart3, imu_reset, 5 ,10000);
#define imu_set_data() HAL_UART_Transmit(&huart3, imu_set, 5 ,10000);
#define imu_receive_start() HAL_UART_Receive_IT(&huart3, imu_data, 22);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//general
MOTOR motor1,motor2,motor3,motor4;//represent the right front, left front, left back, right back（象限）
ULTRASONIC_SENSOR sensor1,sensor2,sensor3,sensor4,sensor5;//left,lf,front,rf,right
uint8_t sys_control_frequency=100;
uint8_t mode=0;//0--remote mode;1--tracing mode;2--obstacle mode
uint8_t receive[8];//receive buffer
uint32_t control_time=0;
uint32_t grabber_lift_angel = 30;
uint32_t imu_time = 0;

//tracing
int32_t path_error;
int32_t path_error_last;
uint8_t tracing_flag;
uint32_t  receive_time=1;
uint32_t receive_last_time=0;
uint32_t pause_time=0;


//obstacle
uint8_t imu_set[5]={0xff,0xaa,0x02,0x08,0x00};
uint8_t imu_speed[5]={0xff,0xaa,0x03,0x09,0x00};
uint8_t imu_reset[5]={0xff,0xaa,0x00,0x01,0x00};
uint32_t yaw;
uint8_t imu_data[24];
uint32_t measure_time=0;//ultrasonic wave sensor measure time
uint32_t distance_final[5];//过滤之后的距�????????
uint32_t distance_l[12];//obstacle distance measured by ultrasonic
uint32_t distance_lf[12];
uint32_t distance_f[12];
uint32_t distance_rf[12];
uint32_t distance_r[12];
uint32_t direction;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//initialize and the mode choosing
void sys_init();
void sys_start();
void bluetooth_mode_start();
void sensor_mode_start();//for obstacle avoiding
void get_bluetooth_instruction();//for the bluetooth mode

//move function
void go_back();
void brake();
void go_straight(uint32_t speed);
void go_left(uint32_t speed);
void go_right(uint32_t speed);
void go_leftfront(uint32_t speed);
void go_rightfront(uint32_t speed);
void go_leftback(uint32_t speed);
void go_rightback(uint32_t speed);
void rotate_left(uint32_t speed);
void rotate_right(uint32_t speed);
void turn_left();
void turn_right();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */



  sys_init();
  sys_start();
  bluetooth_mode_start();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback  ( TIM_HandleTypeDef *  htim )
{

	if(htim->Instance==TIM6)//控制周期
	{
		control_time++;

		if(tracing_flag==1)
		{
			//计算未收到新指令的时�?
			if(receive_last_time==receive_time)
				pause_time++;
			else
				pause_time=0;
			//如果200ms未收到新指令就停�?
			if(pause_time>20)
				brake();
			else
			{
				int32_t path_control;
				path_control=path_error;
				if(path_control<-30)
					rotate_left((-path_control-30)*0.11+30);
				else if(path_control>30)
					rotate_right((path_control-30)*0.11+30);
				else if(path_error>16)
					turn_right();
				else if(path_error<-16)
					turn_left();
				else
					go_straight(28);
			}
		receive_last_time=receive_time;
		}

		motor_control(&motor1);
		motor_control(&motor2);
		motor_control(&motor3);
		motor_control(&motor4);
	}


	if(htim -> Instance == TIM7 && mode == 2)//obstacle mode
	{
		distance_l[measure_time] = get_distance(&sensor1);
		distance_lf[measure_time] = get_distance(&sensor2);
		distance_f[measure_time] = get_distance(&sensor3);
		distance_rf[measure_time] = get_distance(&sensor4);
		distance_r[measure_time] = get_distance(&sensor5);

		measure_time++;
		if(measure_time==10)
		{
			HAL_GPIO_TogglePin(GPIOB, running_led_Pin);//灯闪烁来表示运行正常
			measure_time=0;

			//调整姿�??
			if(yaw>20 && yaw<340)
			{
				if(yaw<180)
					rotate_right(30);
				if(yaw>180)
					rotate_left(30);

			}

			//正常避障
			else
			{
				//进行过滤
				distance_final[0]=filter(distance_l);
				distance_final[1]=filter(distance_lf);
				distance_final[2]=filter(distance_f);
				distance_final[3]=filter(distance_rf);
				distance_final[4]=filter(distance_r);

				//输出方向
				direction = direction_output(distance_final);

				switch(direction)
				{
				case 1://�????????
					go_left(50);
					break;
				case 2://左前
					go_leftfront(50);
					break;
				case 3://�????????
					go_straight(50);
					break;
				case 4://右前
					go_rightfront(50);
					break;
				case 5://�????????
					go_right(50);
					break;
				}
			}
		}
        rising_capture_begin(&sensor3);//only one sensor for the rising capture
	    send_trigger();
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		get_falling_time(&sensor1, htim7);
	}
	if(htim->Instance==TIM3)
	{
		get_falling_time(&sensor2, htim7);
	}

	if(htim->Instance==TIM4)
	{
		if(sensor3.capture_flag==1)//common rising capture of the eco wave
		{
			uint32_t common_rising_time;
			common_rising_time= htim7.Instance->CNT;
			get_common_rising_time(&sensor1, common_rising_time);
			get_common_rising_time(&sensor2, common_rising_time);
			get_common_rising_time(&sensor3, common_rising_time);
			get_common_rising_time(&sensor4, common_rising_time);
			get_common_rising_time(&sensor5, common_rising_time);
		}
		else if(sensor3.capture_flag==2)
		{
			get_falling_time(&sensor3, htim7);
		}
	}

	if(htim->Instance==TIM5)
	{
		get_falling_time(&sensor4, htim7);
	}

	if(htim->Instance==TIM8)
	{
		get_falling_time(&sensor5, htim7);
	}
}


void HAL_UART_RxCpltCallback  ( UART_HandleTypeDef *  huart )
{
	if(huart->Instance == USART1)
	{
		get_bluetooth_instruction();
	}
	if(huart->Instance == USART3)
	{

		yaw=get_yaw(imu_data);
		imu_receive_start();
		imu_time++;
	}
}


void sys_init()
{
	nojtag();
	HAL_GPIO_WritePin(GPIOB,wrong_led_Pin , GPIO_PIN_SET);// show it is running correctly

	motor_init(&motor1, htim1, htim2, 1, 1, 900, sys_control_frequency);
	motor_init(&motor2, htim1, htim3, 2, 2, 936, sys_control_frequency);
	motor_init(&motor3, htim1, htim4, 3, 3, 900, sys_control_frequency);
	motor_init(&motor4, htim1, htim5, 4, 4, 896, sys_control_frequency);

	motor2.controller.kp=4;

	sensor_init(&sensor1,htim2);
	sensor_init(&sensor2,htim3);
	sensor_init(&sensor3,htim4);
	sensor_init(&sensor4,htim5);
	sensor_init(&sensor5,htim8);


}

void sys_start()
{
	 motor_start(&motor1);
	  motor_start(&motor2);
	  motor_start(&motor3);
	  motor_start(&motor4);

	  control_start(&motor1, htim6);
	  control_start(&motor2, htim6);
	  control_start(&motor3, htim6);
	  control_start(&motor4, htim6);

//	  camera_lift(20);
	  grabber_lift(grabber_lift_angel);
	  grabber_move(5);

	  imu_receive_start();

}
void bluetooth_mode_start()
{
	mode=0;
	bluetooth_receive_start();
}

void sensor_mode_start()
{
	mode=2;
	sensor_receive_start();
	imu_receive_start();
}



void get_bluetooth_instruction()
{
	if(receive[0] == 0xEA && receive[5]== 0xFF)//帧头帧尾
	{
		uint32_t sum=0;
		for(int i=1;i<5;i++)
		{
			sum+=receive[i];
		}
		if(sum%256 == 0 && receive[1]== 0x00)//校验�??????????????????????????????
		{
			switch(receive[2])
			{
			case 0x00:
				mode=0;
				brake();
				break;
			case 0x01:
				go_straight(45);
				break;
			case 0x02:
				go_back(45);
				break;
			case 0x03:
				go_left(45);
				break;
			case 0x04:
				go_right(45);
				break;
			case 0x05:
				go_leftfront(45);
				break;
			case 0x06:
				go_rightfront(45);
				break;
			case 0x07:
				go_leftback(45);
				break;
			case 0x08:
				go_rightback(45);
				break;
			case 0x09:
				rotate_left(45);
				break;
			case 0x0a:
				rotate_right(45);
				break;
			case 0x0b:
				grabber_move(45);
				break;
			case 0x0c://抬升
				grabber_lift_angel+=5;
				grabber_lift(grabber_lift_angel);
				break;
			case 0x0d:
				grabber_move(5);
				break;
			case 0x0e://下降
				grabber_lift_angel-=5;
				grabber_lift(grabber_lift_angel);
				break;
			case 0x0f:
				sensor_mode_start();
				break;
			case 0x10:
				tracing_receive_start();
				mode=1;
				break;
			}
		}
		bluetooth_receive_start();
	}

if(mode == 1)
{
	if(receive[0]==0x00 || receive[0]==0x01)
	{
		receive_time++;
	path_error=path_error_last;
	path_error=get_sign(receive[0], receive[1]);

	tracing_receive_start();

	tracing_flag=1;
	}
	else if(receive[0]==0x02)
	{
		mode =0 ;
		bluetooth_receive_start();
		brake();
	}
}
}


void go_back()
{
	motor1.expect_speed = -60;
	motor2.expect_speed = -60;
	motor3.expect_speed = -60;
	motor4.expect_speed = -60;
}
void brake()
{
	motor1.expect_speed = 0;
	motor2.expect_speed = 0;
	motor3.expect_speed = 0;
	motor4.expect_speed = 0;
}
void go_straight(uint32_t speed)
{
	motor1.expect_speed = speed;
	motor2.expect_speed = speed;
	motor3.expect_speed = speed;
	motor4.expect_speed = speed;
}
void go_left(uint32_t speed)
{
	motor1.expect_speed = -speed;
	motor2.expect_speed = speed;
	motor3.expect_speed = -speed;
	motor4.expect_speed = speed;
}
void go_right(uint32_t speed)
{
	motor1.expect_speed = speed;
	motor2.expect_speed = -speed;
	motor3.expect_speed = speed;
	motor4.expect_speed = -speed;
}
void go_leftfront(uint32_t speed)
{
	motor1.expect_speed = 0;
	motor2.expect_speed = speed;
	motor3.expect_speed = 0;
	motor4.expect_speed = speed;
}
void go_rightfront(uint32_t speed)
{
	motor1.expect_speed = speed;
	motor2.expect_speed = 0;
	motor3.expect_speed = speed;
	motor4.expect_speed = 0;
}

void go_leftback(uint32_t speed)
{
	motor1.expect_speed = 0;
	motor2.expect_speed = -speed;
	motor3.expect_speed = 0;
	motor4.expect_speed = -speed;
}

void go_rightback(uint32_t speed)
{
	motor1.expect_speed = -speed;
	motor2.expect_speed = 0;
	motor3.expect_speed = -speed;
	motor4.expect_speed = 0;

}
void rotate_left(uint32_t speed)
{
	motor1.expect_speed = speed;
	motor2.expect_speed = -speed;
	motor3.expect_speed = -speed;
	motor4.expect_speed =speed;
}

void rotate_right(uint32_t speed)
{
	motor1.expect_speed = -speed;
	motor2.expect_speed = speed;
	motor3.expect_speed = speed;
	motor4.expect_speed = -speed;
}

void turn_left()
{
	motor1.expect_speed = 28;
	motor2.expect_speed = 0;
	motor3.expect_speed = 0;
	motor4.expect_speed = 28;
}

void turn_right()
{
	motor1.expect_speed = 0;
	motor2.expect_speed = 28;
	motor3.expect_speed = 28;
	motor4.expect_speed = 0;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
