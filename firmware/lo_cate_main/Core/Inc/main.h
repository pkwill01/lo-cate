/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LORA_MISO_Pin GPIO_PIN_2
#define LORA_MISO_GPIO_Port GPIOC
#define LORA_MOSI_Pin GPIO_PIN_3
#define LORA_MOSI_GPIO_Port GPIOC
#define LED2_BLUE_Pin GPIO_PIN_1
#define LED2_BLUE_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define IMU_CSB_Pin GPIO_PIN_4
#define IMU_CSB_GPIO_Port GPIOC
#define LED3_GREEN_Pin GPIO_PIN_0
#define LED3_GREEN_GPIO_Port GPIOB
#define LORA_SCK_Pin GPIO_PIN_13
#define LORA_SCK_GPIO_Port GPIOB
#define LED3_RED_Pin GPIO_PIN_6
#define LED3_RED_GPIO_Port GPIOC
#define LED3_BLUE_Pin GPIO_PIN_7
#define LED3_BLUE_GPIO_Port GPIOC
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define LED2_RED_Pin GPIO_PIN_15
#define LED2_RED_GPIO_Port GPIOA
#define LED1_RED_Pin GPIO_PIN_6
#define LED1_RED_GPIO_Port GPIOB
#define LED1_BLUE_Pin GPIO_PIN_7
#define LED1_BLUE_GPIO_Port GPIOB
#define LED1_GREEN_Pin GPIO_PIN_8
#define LED1_GREEN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
