/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define TH0_Pin GPIO_PIN_0
#define TH0_GPIO_Port GPIOC
#define TH1_Pin GPIO_PIN_2
#define TH1_GPIO_Port GPIOA
#define ICM_INT1_Pin GPIO_PIN_4
#define ICM_INT1_GPIO_Port GPIOC
#define ICM_INT2_Pin GPIO_PIN_5
#define ICM_INT2_GPIO_Port GPIOC
#define VBAT_Pin GPIO_PIN_1
#define VBAT_GPIO_Port GPIOB
#define RF_RESET_Pin GPIO_PIN_2
#define RF_RESET_GPIO_Port GPIOB
#define GPS_1PPS_Pin GPIO_PIN_12
#define GPS_1PPS_GPIO_Port GPIOE
#define RF_DIO0_Pin GPIO_PIN_10
#define RF_DIO0_GPIO_Port GPIOB
#define RF_DIO1_Pin GPIO_PIN_11
#define RF_DIO1_GPIO_Port GPIOB
#define RGB_DIN_Pin GPIO_PIN_12
#define RGB_DIN_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOB
#define ARM_DET_Pin GPIO_PIN_9
#define ARM_DET_GPIO_Port GPIOD
#define SD_CD_Pin GPIO_PIN_3
#define SD_CD_GPIO_Port GPIOD
#define BMP_INT_Pin GPIO_PIN_9
#define BMP_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
