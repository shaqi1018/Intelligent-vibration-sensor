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
#include "stm32u5xx_hal.h"

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
/* USER CODE BEGIN Private defines */
/* Temporary single-sensor bring-up target:
 * 0 = normal multi-thread, three-sensor behavior
 * 1 = LSM6DSOX single-sensor test
 * 2 = H3LIS100DL single-sensor test
 * 3 = QMA6100P single-sensor test
 */
#define APP_SENSOR_TEST_NONE        0U
#define APP_SENSOR_TEST_LSM6DSOX    1U
#define APP_SENSOR_TEST_H3LIS100DL  2U
#define APP_SENSOR_TEST_QMA6100P    3U

#define APP_SENSOR_TEST_TARGET APP_SENSOR_TEST_NONE

#define SDMMC1_DET_Pin              GPIO_PIN_13
#define SDMMC1_DET_GPIO_Port        GPIOC
#define SDMMC1_DET_INSERTED_LEVEL   GPIO_PIN_RESET

#define KEY_ADC_Pin                 GPIO_PIN_0
#define KEY_ADC_GPIO_Port           GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
