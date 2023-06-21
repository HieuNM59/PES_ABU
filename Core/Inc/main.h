/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define IN1_7_Pin GPIO_PIN_14
#define IN1_7_GPIO_Port GPIOC
#define IN1_8_Pin GPIO_PIN_15
#define IN1_8_GPIO_Port GPIOC
#define IN3_1_Pin GPIO_PIN_0
#define IN3_1_GPIO_Port GPIOA
#define IN3_2_Pin GPIO_PIN_1
#define IN3_2_GPIO_Port GPIOA
#define IN3_3_Pin GPIO_PIN_2
#define IN3_3_GPIO_Port GPIOA
#define IN3_4_Pin GPIO_PIN_3
#define IN3_4_GPIO_Port GPIOA
#define IN3_5_Pin GPIO_PIN_4
#define IN3_5_GPIO_Port GPIOA
#define IN3_6_Pin GPIO_PIN_5
#define IN3_6_GPIO_Port GPIOA
#define IN3_7_Pin GPIO_PIN_6
#define IN3_7_GPIO_Port GPIOA
#define IN3_8_Pin GPIO_PIN_7
#define IN3_8_GPIO_Port GPIOA
#define IN2_1_Pin GPIO_PIN_0
#define IN2_1_GPIO_Port GPIOB
#define IN2_2_Pin GPIO_PIN_1
#define IN2_2_GPIO_Port GPIOB
#define IN1_1_Pin GPIO_PIN_2
#define IN1_1_GPIO_Port GPIOB
#define PES_CLK_Pin GPIO_PIN_12
#define PES_CLK_GPIO_Port GPIOB
#define PES_ATT_Pin GPIO_PIN_13
#define PES_ATT_GPIO_Port GPIOB
#define PES_CMD_Pin GPIO_PIN_14
#define PES_CMD_GPIO_Port GPIOB
#define PES_DATA_Pin GPIO_PIN_15
#define PES_DATA_GPIO_Port GPIOB
#define IN2_3_Pin GPIO_PIN_8
#define IN2_3_GPIO_Port GPIOA
#define IN2_4_Pin GPIO_PIN_9
#define IN2_4_GPIO_Port GPIOA
#define IN2_5_Pin GPIO_PIN_10
#define IN2_5_GPIO_Port GPIOA
#define IN2_6_Pin GPIO_PIN_11
#define IN2_6_GPIO_Port GPIOA
#define IN2_7_Pin GPIO_PIN_12
#define IN2_7_GPIO_Port GPIOA
#define IN2_8_Pin GPIO_PIN_15
#define IN2_8_GPIO_Port GPIOA
#define IN1_2_Pin GPIO_PIN_3
#define IN1_2_GPIO_Port GPIOB
#define IN1_3_Pin GPIO_PIN_4
#define IN1_3_GPIO_Port GPIOB
#define IN1_4_Pin GPIO_PIN_5
#define IN1_4_GPIO_Port GPIOB
#define IN1_5_Pin GPIO_PIN_8
#define IN1_5_GPIO_Port GPIOB
#define IN1_6_Pin GPIO_PIN_9
#define IN1_6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
