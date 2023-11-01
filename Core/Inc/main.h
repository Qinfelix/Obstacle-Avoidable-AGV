/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ultasonic4_Pin GPIO_PIN_2
#define ultasonic4_GPIO_Port GPIOA
#define ultrasonic2_Pin GPIO_PIN_0
#define ultrasonic2_GPIO_Port GPIOB
#define trigger_Pin GPIO_PIN_1
#define trigger_GPIO_Port GPIOB
#define ultrasonic1_Pin GPIO_PIN_10
#define ultrasonic1_GPIO_Port GPIOB
#define running_led_Pin GPIO_PIN_13
#define running_led_GPIO_Port GPIOB
#define wrong_led_Pin GPIO_PIN_14
#define wrong_led_GPIO_Port GPIOB
#define motor3_in1_Pin GPIO_PIN_8
#define motor3_in1_GPIO_Port GPIOD
#define motor3_in2_Pin GPIO_PIN_9
#define motor3_in2_GPIO_Port GPIOD
#define motor4_in1_Pin GPIO_PIN_10
#define motor4_in1_GPIO_Port GPIOD
#define motor4_in2_Pin GPIO_PIN_11
#define motor4_in2_GPIO_Port GPIOD
#define ultrasonic3_Pin GPIO_PIN_14
#define ultrasonic3_GPIO_Port GPIOD
#define sg90_pwm_camera_Pin GPIO_PIN_6
#define sg90_pwm_camera_GPIO_Port GPIOC
#define sg90_pwm_grabber_Pin GPIO_PIN_7
#define sg90_pwm_grabber_GPIO_Port GPIOC
#define ultrasonic5_Pin GPIO_PIN_8
#define ultrasonic5_GPIO_Port GPIOC
#define mg995_pwm_Pin GPIO_PIN_9
#define mg995_pwm_GPIO_Port GPIOC
#define motor1_in1_Pin GPIO_PIN_1
#define motor1_in1_GPIO_Port GPIOD
#define motor1_in2_Pin GPIO_PIN_2
#define motor1_in2_GPIO_Port GPIOD
#define motor2_in1_Pin GPIO_PIN_3
#define motor2_in1_GPIO_Port GPIOD
#define motor2_in2_Pin GPIO_PIN_4
#define motor2_in2_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
