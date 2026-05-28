/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "main.h"
#include "qma6100p.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* SDMMC1 card-detect (PC13) */
  GPIO_InitStruct.Pin = SDMMC1_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDMMC1_DET_GPIO_Port, &GPIO_InitStruct);

  /* LSM6DSOX INT1 on PB1: rising-edge EXTI for FIFO watermark interrupt.
     Pull-down keeps line stable when sensor is inactive; the sensor drives
     active-high in default configuration. NVIC priority must be >= 5 so the
     ISR can safely call FromISR FreeRTOS APIs. */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* QMA6100P INT (currently INT2 routed to PB15 — see qma6100p.h INT macros).
     Pull-up because INT2 default is open-drain low-active on this chip. */
  GPIO_InitStruct.Pin = QMA6100P_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(QMA6100P_INT_GPIO_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(QMA6100P_INT_EXTI_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(QMA6100P_INT_EXTI_IRQn);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
