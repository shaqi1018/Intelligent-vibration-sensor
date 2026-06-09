/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32u5xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32u5xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dma_sampling.h"
#include "sai.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "usart.h"
#include <string.h>

/* Emit a fault marker directly over USART1 (polling, no RTOS/printf) so a
 * crash becomes visible instead of a silent while(1) freeze. */
static void FaultPrint(const char *msg)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
}

static void FaultPrintHex(const char *label, uint32_t v)
{
  char buf[16];
  static const char hx[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) { buf[i] = hx[(v >> ((7 - i) * 4)) & 0xFU]; }
  buf[8] = '\r'; buf[9] = '\n'; buf[10] = '\0';
  FaultPrint(label);
  FaultPrint(buf);
}

/* Decode the exception stack frame of the faulting task (FreeRTOS tasks run on
 * PSP). sp[6] = PC of the faulting instruction — look it up in SensorProj.map. */
void HardFault_ReportC(uint32_t *sp)
{
  FaultPrint("\r\n*** HARDFAULT ***\r\n");
  FaultPrintHex("R0  =0x", sp[0]);
  FaultPrintHex("R1  =0x", sp[1]);
  FaultPrintHex("R2  =0x", sp[2]);
  FaultPrintHex("R3  =0x", sp[3]);
  FaultPrintHex("R12 =0x", sp[4]);
  FaultPrintHex("LR  =0x", sp[5]);
  FaultPrintHex("PC  =0x", sp[6]);   /* faulting instruction */
  FaultPrintHex("xPSR=0x", sp[7]);
  FaultPrintHex("CFSR=0x", SCB->CFSR);
  FaultPrintHex("HFSR=0x", SCB->HFSR);
  FaultPrintHex("BFAR=0x", SCB->BFAR);
  for (;;) { }
}
/* USER CODE END 0 */

extern TIM_HandleTypeDef htim6;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SD_HandleTypeDef hsd1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;


/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  HardFault_ReportC((uint32_t *)__get_PSP());
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  FaultPrint("\r\n*** MEMMANAGE FAULT ***\r\n");
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  FaultPrint("\r\n*** BUSFAULT ***\r\n");
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  FaultPrint("\r\n*** USAGEFAULT ***\r\n");
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32U5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u5xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  DmaSampling_NotifySpiIrq();
  HAL_SPI_IRQHandler(&hspi1);
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  DmaSampling_NotifySpi2Irq();
  HAL_SPI_IRQHandler(&hspi2);
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  HAL_SD_IRQHandler(&hsd1);
}

/**
  * @brief This function handles EXTI line 0 interrupt (PB0 = LSM6DSOX INT1).
  */
void EXTI0_IRQHandler(void)   /* LSM6DSOX INT1 → PB0 */
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
  * @brief This function handles EXTI line 1 interrupt (PA1 = H3LIS100DL DRDY).
  */
void EXTI1_IRQHandler(void)   /* H3LIS100DL DRDY → PA1 */
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
  * @brief This function handles EXTI line 4 interrupt (PC4 = QMA6100P INT1).
  */
void EXTI4_IRQHandler(void)   /* QMA6100P INT1 → PC4 */
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
  * @brief This function handles GPDMA1 Channel 4 global interrupt
  *        (SAI1_A RX circular DMA feeding the ES8311 mic ring buffer).
  */
void GPDMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&handle_GPDMA1_Channel_SAI);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
