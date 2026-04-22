/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lsm6dsox.h"
#include "h3lis100dl.h"
#include "qma6100p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_PERIOD_MS 500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ======================== Bus architecture ================================
 *
 *   SPI1 (PA5/PA6/PA7 + PC4 CS)          -> LSM6DSOX   (dedicated bus)
 *   SPI2 (PB10/PC2/PC1 + PC5/PA4 CS)     -> H3LIS100DL + QMA6100P (shared)
 */

static osMutexId_t spi2_mutex;
static const osMutexAttr_t spi2_mutex_attr = {
  .name      = "spi2Mutex",
  .attr_bits = osMutexPrioInherit,
};

/* USER CODE END Variables */

/* ======================== Sensor threads ================================== */
osThreadId_t lsm6dsoxTaskHandle;
const osThreadAttr_t lsm6dsoxTask_attributes = {
  .name = "lsm6dsoxTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 512 * 4
};

osThreadId_t qma6100pTaskHandle;
const osThreadAttr_t qma6100pTask_attributes = {
  .name = "qma6100pTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 256 * 4
};

osThreadId_t h3lis100dlTaskHandle;
const osThreadAttr_t h3lis100dlTask_attributes = {
  .name = "h3lis100dlTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLsm6dsoxTask(void *argument);
void StartQma6100pTask(void *argument);
void StartH3lis100dlTask(void *argument);

void MX_FREERTOS_Init(void);

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
__weak void configureTimerForRunTimeStats(void)
{
}

__weak unsigned long getRunTimeCounterValue(void)
{
  return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
  (void)ulExpectedIdleTime;
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
  (void)ulExpectedIdleTime;
}
/* USER CODE END PREPOSTSLEEP */

void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
#if (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_NONE) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P)
  spi2_mutex = osMutexNew(&spi2_mutex_attr);
#endif
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_LSM6DSOX
  lsm6dsoxTaskHandle = osThreadNew(StartLsm6dsoxTask, NULL, &lsm6dsoxTask_attributes);
#elif APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL
  h3lis100dlTaskHandle = osThreadNew(StartH3lis100dlTask, NULL, &h3lis100dlTask_attributes);
#elif APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P
  qma6100pTaskHandle = osThreadNew(StartQma6100pTask, NULL, &qma6100pTask_attributes);
#else
  lsm6dsoxTaskHandle = osThreadNew(StartLsm6dsoxTask, NULL, &lsm6dsoxTask_attributes);
  h3lis100dlTaskHandle = osThreadNew(StartH3lis100dlTask, NULL, &h3lis100dlTask_attributes);
  qma6100pTaskHandle = osThreadNew(StartQma6100pTask, NULL, &qma6100pTask_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_LSM6DSOX
static const char *LSM6DSOX_DiagPullName(uint32_t pull)
{
  switch (pull)
  {
    case GPIO_PULLUP:   return "PULLUP";
    case GPIO_PULLDOWN: return "PULLDOWN";
    default:            return "NOPULL";
  }
}

static void LSM6DSOX_DiagShortDelay(volatile uint32_t cycles)
{
  while (cycles--)
  {
    __NOP();
  }
}

static void LSM6DSOX_DiagConfigureMisoPull(uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LSM_SPI_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = LSM_SPI_MISO_AF;
  HAL_GPIO_Init(LSM_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);
}

static HAL_StatusTypeDef LSM6DSOX_DiagReadWhoAmIRaw(uint8_t *rx0, uint8_t *rx1)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2] = {(uint8_t)(LSM6DSOX_REG_WHO_AM_I | LSM6DSOX_SPI_READ), 0x00U};
  uint8_t rx[2] = {0};

  LSM_SPI_CS_HIGH();
  LSM6DSOX_DiagShortDelay(200U);
  LSM_SPI_CS_LOW();
  LSM6DSOX_DiagShortDelay(200U);
  ret = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, LSM_SPI_TIMEOUT_MS);
  LSM_SPI_CS_HIGH();
  LSM6DSOX_DiagShortDelay(200U);

  if (rx0 != NULL)
  {
    *rx0 = rx[0];
  }
  if (rx1 != NULL)
  {
    *rx1 = rx[1];
  }

  return ret;
}

static void LSM6DSOX_DiagPrintHwSpiSweep(void)
{
  const uint32_t pulls[] = {GPIO_PULLUP, GPIO_PULLDOWN, GPIO_NOPULL};
  uint32_t pull_idx;

  printf("[LSM6DSOX DIAG] HW-SPI WHO_AM_I sweep\r\n");

  for (pull_idx = 0U; pull_idx < (uint32_t)(sizeof(pulls) / sizeof(pulls[0])); pull_idx++)
  {
    uint8_t attempt;

    LSM6DSOX_DiagConfigureMisoPull(pulls[pull_idx]);
    HAL_Delay(1U);

    for (attempt = 0U; attempt < 3U; attempt++)
    {
      uint8_t rx0 = 0U;
      uint8_t rx1 = 0U;
      HAL_StatusTypeDef ret = LSM6DSOX_DiagReadWhoAmIRaw(&rx0, &rx1);

      printf("[LSM6DSOX DIAG] HW pull=%s try=%u ret=%d TX=[8F 00] RX=[%02X %02X]\r\n",
             LSM6DSOX_DiagPullName(pulls[pull_idx]),
             (unsigned int)(attempt + 1U),
             (int)ret,
             rx0,
             rx1);
      HAL_Delay(2U);
    }
  }
}

static void LSM6DSOX_DiagConfigureBitBangPins(uint32_t miso_pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_SPI_DeInit(&hspi1);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = LSM_SPI_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LSM_SPI_CS_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LSM_SPI_SCK_PIN | LSM_SPI_MOSI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LSM_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LSM_SPI_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = miso_pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LSM_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LSM_SPI_CS_GPIO_PORT, LSM_SPI_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LSM_SPI_SCK_GPIO_PORT, LSM_SPI_SCK_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LSM_SPI_MOSI_GPIO_PORT, LSM_SPI_MOSI_PIN, GPIO_PIN_RESET);
}

static uint8_t LSM6DSOX_DiagBitBangTransfer(uint8_t tx)
{
  uint8_t rx = 0U;
  uint8_t bit;

  for (bit = 0U; bit < 8U; bit++)
  {
    uint8_t mask = (uint8_t)(0x80U >> bit);

    HAL_GPIO_WritePin(LSM_SPI_SCK_GPIO_PORT, LSM_SPI_SCK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LSM_SPI_MOSI_GPIO_PORT,
                      LSM_SPI_MOSI_PIN,
                      (tx & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    LSM6DSOX_DiagShortDelay(2000U);

    HAL_GPIO_WritePin(LSM_SPI_SCK_GPIO_PORT, LSM_SPI_SCK_PIN, GPIO_PIN_SET);
    LSM6DSOX_DiagShortDelay(2000U);

    if (HAL_GPIO_ReadPin(LSM_SPI_MISO_GPIO_PORT, LSM_SPI_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= mask;
    }
  }

  return rx;
}

static void LSM6DSOX_DiagRunBitBangRead(uint32_t miso_pull)
{
  uint8_t rx0;
  uint8_t rx1;

  LSM6DSOX_DiagConfigureBitBangPins(miso_pull);
  LSM6DSOX_DiagShortDelay(4000U);

  HAL_GPIO_WritePin(LSM_SPI_CS_GPIO_PORT, LSM_SPI_CS_PIN, GPIO_PIN_RESET);
  LSM6DSOX_DiagShortDelay(4000U);
  rx0 = LSM6DSOX_DiagBitBangTransfer((uint8_t)(LSM6DSOX_REG_WHO_AM_I | LSM6DSOX_SPI_READ));
  rx1 = LSM6DSOX_DiagBitBangTransfer(0x00U);
  HAL_GPIO_WritePin(LSM_SPI_CS_GPIO_PORT, LSM_SPI_CS_PIN, GPIO_PIN_SET);
  LSM6DSOX_DiagShortDelay(4000U);

  printf("[LSM6DSOX DIAG] BITBANG pull=%s TX=[8F 00] RX=[%02X %02X]\r\n",
         LSM6DSOX_DiagPullName(miso_pull),
         rx0,
         rx1);

  MX_SPI1_Init();
}

static void LSM6DSOX_RunConnectivityDiagnostics(void)
{
  printf("\r\n[LSM6DSOX DIAG] ===== begin =====\r\n");
  printf("[LSM6DSOX DIAG] Pins: CS=PC4 SCK=PA5 MISO=PA6 MOSI=PA7\r\n");

  LSM6DSOX_DiagPrintHwSpiSweep();
  LSM6DSOX_DiagRunBitBangRead(GPIO_NOPULL);
  LSM6DSOX_DiagRunBitBangRead(GPIO_PULLDOWN);

  printf("[LSM6DSOX DIAG] Hint: pull-up->FF and pull-down->00 usually means MISO is floating.\r\n");
  printf("[LSM6DSOX DIAG] ===== end =====\r\n\r\n");
}
#endif

void StartLsm6dsoxTask(void *argument)
{
  LSM6DSOX_AllData_t all_data;

  (void)argument;

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_LSM6DSOX
  printf("[LSM6DSOX TEST] start connectivity diagnostics...\r\n");
  LSM6DSOX_RunConnectivityDiagnostics();

  while (LSM6DSOX_Init() != HAL_OK)
  {
    printf("[LSM6DSOX TEST] init failed, retry in 2s\r\n");
    osDelay(2000U);
    LSM6DSOX_RunConnectivityDiagnostics();
  }

  printf("[LSM6DSOX TEST] started, print every %lu ms\r\n", (unsigned long)SAMPLE_PERIOD_MS);

  for (;;)
  {
    uint8_t status_reg = 0U;

    if (LSM6DSOX_ReadStatus(&status_reg) != HAL_OK)
    {
      printf("[LSM6DSOX TEST] read STATUS failed\r\n");
    }
    else if (LSM6DSOX_ReadAllData(&all_data) != HAL_OK)
    {
      printf("[LSM6DSOX TEST] read data failed (STATUS=0x%02X)\r\n", status_reg);
    }
    else
    {
      printf("[LSM6DSOX TEST] STATUS=0x%02X | ACC(mg) X:%7.1f Y:%7.1f Z:%7.1f | "
             "GYRO(mdps) X:%8.1f Y:%8.1f Z:%8.1f | TEMP:%.1fC\r\n",
             status_reg,
             all_data.acc.x, all_data.acc.y, all_data.acc.z,
             all_data.gyro.x, all_data.gyro.y, all_data.gyro.z,
             all_data.temp_C);
    }

    osDelay(SAMPLE_PERIOD_MS);
  }
#else
  if (LSM6DSOX_Init() != HAL_OK)
  {
    osThreadTerminate(NULL);
    return;
  }

  for (;;)
  {
    uint8_t status_reg = 0U;

    if (LSM6DSOX_ReadStatus(&status_reg) == HAL_OK &&
        (status_reg & LSM6DSOX_STATUS_XLDA) != 0U &&
        LSM6DSOX_ReadAllData(&all_data) == HAL_OK)
    {
      printf("LSM6DSOX: ACC(mg) X:%7.1f Y:%7.1f Z:%7.1f | "
             "GYRO(mdps) X:%8.1f Y:%8.1f Z:%8.1f | TEMP:%.1fC\r\n",
             all_data.acc.x, all_data.acc.y, all_data.acc.z,
             all_data.gyro.x, all_data.gyro.y, all_data.gyro.z,
             all_data.temp_C);
    }

    osDelay(SAMPLE_PERIOD_MS);
  }
#endif
}

void StartH3lis100dlTask(void *argument)
{
  H3LIS100DL_Data_t data;

  (void)argument;

  osMutexAcquire(spi2_mutex, osWaitForever);
  if (H3LIS100DL_Init() != 0)
  {
    osMutexRelease(spi2_mutex);
    osThreadTerminate(NULL);
    return;
  }
  osMutexRelease(spi2_mutex);

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL
  printf("[H3LIS100DL TEST] started, print every %lu ms\r\n", (unsigned long)SAMPLE_PERIOD_MS);
#endif

  for (;;)
  {
    int ret;

    osMutexAcquire(spi2_mutex, osWaitForever);
    ret = H3LIS100DL_ReadAccXYZ(&data);
    osMutexRelease(spi2_mutex);

    if (ret == 0)
    {
      printf("H3LIS100DL: raw[%4d,%4d,%4d]  acc(mg)[%7.1f,%7.1f,%7.1f]\r\n",
             data.raw[0], data.raw[1], data.raw[2],
             data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);
    }
    else if (ret != -2)
    {
#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL
      printf("[H3LIS100DL TEST] read failed (ret=%d)\r\n", ret);
#endif
    }

    osDelay(SAMPLE_PERIOD_MS);
  }
}

void StartQma6100pTask(void *argument)
{
  QMA6100P_Data_t data;

  (void)argument;

  osMutexAcquire(spi2_mutex, osWaitForever);
  if (QMA6100P_Init() != HAL_OK)
  {
    osMutexRelease(spi2_mutex);
    osThreadTerminate(NULL);
    return;
  }
  osMutexRelease(spi2_mutex);

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P
  printf("[QMA6100P TEST] started, print every %lu ms\r\n", (unsigned long)SAMPLE_PERIOD_MS);
#endif

  for (;;)
  {
    HAL_StatusTypeDef ret;

    osMutexAcquire(spi2_mutex, osWaitForever);
    ret = QMA6100P_ReadAccXYZ(&data);
    osMutexRelease(spi2_mutex);

    if (ret == HAL_OK)
    {
      printf("QMA6100P: acc(mg) X:%7.1f Y:%7.1f Z:%7.1f\r\n",
             data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);
    }
    else
    {
#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P
      printf("[QMA6100P TEST] read failed\r\n");
#endif
    }

    osDelay(SAMPLE_PERIOD_MS);
  }
}

/* USER CODE END Application */
