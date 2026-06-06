/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdmmc.h
  * @brief   SDMMC1 bring-up interface.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __SDMMC_H__
#define __SDMMC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SD_HandleTypeDef hsd1;

HAL_StatusTypeDef MX_SDMMC1_SD_Init(void);
uint8_t SDMMC1_IsCardDetected(void);
uint8_t SDMMC1_InitCard(void);

#ifdef __cplusplus
}
#endif

#endif /* __SDMMC_H__ */
