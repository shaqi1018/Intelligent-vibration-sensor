/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdmmc.c
  * @brief   SDMMC1 initialization for SD card bring-up.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "sdmmc.h"
#include <stdio.h>

SD_HandleTypeDef hsd1;

uint8_t SDMMC1_IsCardDetected(void)
{
  return (HAL_GPIO_ReadPin(SDMMC1_DET_GPIO_Port, SDMMC1_DET_Pin) == SDMMC1_DET_INSERTED_LEVEL) ? 1U : 0U;
}

uint8_t SDMMC1_InitCard(void)
{
  HAL_SD_CardStateTypeDef state;

  if (SDMMC1_IsCardDetected() == 0U)
  {
    return 0U;
  }

  if (hsd1.Instance != SDMMC1)
  {
    MX_SDMMC1_SD_Init();
    return (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) ? 1U : 0U;
  }

  state = HAL_SD_GetCardState(&hsd1);
  if (state == HAL_SD_CARD_TRANSFER)
  {
    return 1U;
  }

  /* Card is not in TRANSFER (ERROR, SENDING, RECEIVE, etc.) → full re-init */
  HAL_SD_DeInit(&hsd1);
  MX_SDMMC1_SD_Init();

  return (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) ? 1U : 0U;
}

void MX_SDMMC1_SD_Init(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 24U;  /* 48 MHz / (24+2) ≈ 1.85 MHz — faster identification */

  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) == HAL_OK)
  {
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  }
  /* Boost data transfer clock: 48 MHz / (1+2) = 16 MHz.
   * Only touches CLKCR, no card re-init needed. */
  MODIFY_REG(hsd1.Instance->CLKCR, SDMMC_CLKCR_CLKDIV, 1U << SDMMC_CLKCR_CLKDIV_Pos);
}

void HAL_SD_MspInit(SD_HandleTypeDef *sdHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (sdHandle->Instance != SDMMC1)
  {
    return;
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ICLK | RCC_PERIPHCLK_SDMMC;
  PeriphClkInit.IclkClockSelection = RCC_ICLK_CLKSOURCE_HSI48;
  PeriphClkInit.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_SDMMC1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* SDMMC1 IRQ disabled — polling mode only (HAL_SD_ReadBlocks/WriteBlocks
   * with HAL_MAX_DELAY). Keeping the IRQ enabled caused spurious
   * HAL_SD_IRQHandler invocations that corrupted hsd1.State between
   * consecutive FatFs operations in the RTOS context. */
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *sdHandle)
{
  if (sdHandle->Instance != SDMMC1)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

  __HAL_RCC_SDMMC1_CLK_DISABLE();

  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
}
