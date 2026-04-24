/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os2.h"
#include "icache.h"
#include "usart.h"
#include "gpio.h"
#include "sdmmc.h"
#include "fatfs_sd.h"
#include "usb_otg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "bsp_spi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
static void SystemPower_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();

  printf("[初始化] GPIO/ICACHE/UART1 初始化完成\r\n");
  printf("[初始化] SD DET(PC13)=%u, inserted_level=%u\r\n",
         (unsigned int)HAL_GPIO_ReadPin(SDMMC1_DET_GPIO_Port, SDMMC1_DET_Pin),
         (unsigned int)SDMMC1_DET_INSERTED_LEVEL);

  if (SDMMC1_IsCardDetected() != 0U)
  {
    MX_SDMMC1_SD_Init();
    printf("[初始化] SDMMC1 初始化完成 (检测到SD卡)\r\n");
    FatFs_SD_RunPhaseBSmokeTest();
  }
  else
  {
    printf("[初始化] 未检测到SD卡，已跳过 SDMMC1 卡初始化\r\n");
  }

  MX_USB_OTG_FS_PCD_Init();
  printf("[初始化] USB OTG FS PCD 初始化完成\r\n");

  /* SPI1(PA5/PA6/PA7 + PC4(CS)) -> LSM6DSOX */
#if (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_NONE) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_LSM6DSOX)
  MX_SPI1_Init();
  printf("[初始化] SPI1 初始化完成\r\n");
#endif

  /* SPI2(PB10/PC2/PC1 + PC5/PA4(CS)) -> H3LIS100DL + QMA6100P */
#if (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_NONE) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P)
  MX_SPI2_Init();
  printf("[初始化] SPI2 初始化完成\r\n");
#endif

  /* USER CODE BEGIN 2 */

  printf("\r\n========================================\r\n");
#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_LSM6DSOX
  printf("  STM32U575 LSM6DSOX 单测模式\r\n");
  printf("  SPI1: PA5/PA6/PA7 + PC4(CS) (LSM6DSOX)\r\n");
  printf("  串口: PA9/PA10 115200bps\r\n");
  printf("  说明: 已跳过 SPI2 与其它线程\r\n");
#elif APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL
  printf("  STM32U575 H3LIS100DL 单测模式\r\n");
  printf("  SPI2: PB10/PC2/PC1 + PC5(CS) (H3LIS100DL)\r\n");
  printf("  串口: PA9/PA10 115200bps\r\n");
  printf("  说明: 已跳过 LSM6DSOX、QMA6100P 与其它线程\r\n");
#elif APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P
  printf("  STM32U575 QMA6100P 单测模式\r\n");
  printf("  SPI2: PB10/PC2/PC1 + PA4(CS) (QMA6100P)\r\n");
  printf("  串口: PA9/PA10 115200bps\r\n");
  printf("  说明: 已跳过 LSM6DSOX、H3LIS100DL 与其它线程\r\n");
#else
  printf("  STM32U575 FreeRTOS 三传感器平台\r\n");
  printf("  SPI1: PA5/PA6/PA7 + PC4(CS) (LSM6DSOX)\r\n");
  printf("  SPI2: PB10/PC2/PC1 + PC5(CS) (H3LIS100DL)\r\n");
  printf("  SPI2: PB10/PC2/PC1 + PA4(CS) (QMA6100P)\r\n");
  printf("  SDMMC1: PC8/PC9/PC10/PC11/PC12 + PD2, DET=PC13\r\n");
  printf("  USB FS: PA11(D-) PA12(D+)\r\n");
  printf("  串口: PA9/PA10 115200bps\r\n");
#endif
  printf("========================================\r\n\r\n");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in app_freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



