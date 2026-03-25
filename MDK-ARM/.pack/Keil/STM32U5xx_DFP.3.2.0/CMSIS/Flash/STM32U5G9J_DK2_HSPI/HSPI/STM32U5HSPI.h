/**
 ******************************************************************************
 * @file    STM32U5HSPI.h
 * @author  MCD Application Team
 * @brief   Header file of STM32U5HSPI.c
 *
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

#ifndef STM32U5OSPI_H
#define STM32U5OSPI_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include <string.h>
#include "stm32u5xx_hal.h"
#include "stm32u5g9j_discovery_hspi.h"
#include "Components/mx66uw1g45g/mx66uw1g45g.h"

/* Private defines -----------------------------------------------------------*/
#define TIMEOUT 5000U
/* Exported types ------------------------------------------------------------*/

/**
 * @brief Memory map status typedef
 */
typedef enum
{
  MEM_MAPDISABLE = 0, /*!< map mode disabled      */
  MEM_MAPENABLE       /*!< map mode enabled       */
} MEM_MAPSTAT;

/**
 * @brief Loader status typedef
 */
typedef enum
{
  LOADER_STATUS_SUCCESS = 1, /*!< Loader status: Success */
  LOADER_STATUS_FAIL = 0     /*!< Loader status: Fail    */
} loader_status;

/* Exported functions prototypes ---------------------------------------------*/
int Init_OSPI(void);
int Write(uint32_t Address, uint32_t Size, uint8_t *buffer);
int SectorErase(uint32_t EraseStartAddress, uint32_t EraseEndAddress);
int MassErase(void);
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

#endif /* STM32U5HSPI_H */
