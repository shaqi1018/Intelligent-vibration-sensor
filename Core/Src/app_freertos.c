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
#include "lsm6dsox.h"             /* LSM6DSOX   - SPI1 dedicated   */
#include "h3lis100dl.h"           /* H3LIS100DL - SPI2 shared bus  */
#include "qma6100p.h"             /* QMA6100P   - SPI2 shared bus  */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** Key polling period (ms) - all sensor threads share this */
#define KEY_POLL_PERIOD_MS    10U

/** Sensor sampling interval (ms) */
#define SAMPLE_PERIOD_MS      500U

/** Number of polling ticks per sample interval */
#define SAMPLE_INTERVAL_TICKS (SAMPLE_PERIOD_MS / KEY_POLL_PERIOD_MS)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ======================== Bus architecture ================================
 *
 *   SPI1 (PA5/PA6/PA7 + PA4 CS)        -> LSM6DSOX (dedicated bus, no mutex)
 *   SPI2 (PB13/PB14/PB15 + PB12/PB0 CS) -> H3LIS100DL + QMA6100P (shared bus)
 *
 *   SPI2 is shared: a mutex is required before every CS assert/deassert
 *   sequence so that H3LIS100DL and QMA6100P threads cannot interleave on
 *   the same physical bus.
 */

/** Mutex protecting the SPI2 shared bus (H3LIS100DL + QMA6100P) */
static osMutexId_t spi2_mutex;
static const osMutexAttr_t spi2_mutex_attr = {
  .name      = "spi2Mutex",
  .attr_bits = osMutexPrioInherit,   /* priority inheritance: avoid inversion */
};

/* USER CODE END Variables */

/* ======================== System default thread =========================== */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* ======================== LED blink threads =============================== */
osThreadId_t blueBlinkTaskHandle;
const osThreadAttr_t blueBlinkTask_attributes = {
  .name = "blueBlinkTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

osThreadId_t greenBlinkTaskHandle;
const osThreadAttr_t greenBlinkTask_attributes = {
  .name = "greenBlinkTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* ======================== Sensor threads ================================== */

/**
 * @brief  LSM6DSOX thread (currently the only active sensor)
 * @note   Uses SPI1 (PA5/PA6/PA7 + PA4 CS) dedicated bus
 *         Stack: 2048 bytes (for float printf)
 */
osThreadId_t lsm6dsoxTaskHandle;
const osThreadAttr_t lsm6dsoxTask_attributes = {
  .name = "lsm6dsoxTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};

/**
 * @brief  QMA6100P thread (SPI2, PB13/PB14/PB15 + PB0 CS)
 * @note   Shares SPI2 with H3LIS100DL; access protected by spi2_mutex.
 *         Stack 256*4=1024 bytes: covers float printf and sensor init frames.
 */
osThreadId_t qma6100pTaskHandle;
const osThreadAttr_t qma6100pTask_attributes = {
  .name = "qma6100pTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/**
 * @brief  H3LIS100DL thread (SPI2, PB13/PB14/PB15 + PB12 CS)
 * @note   Shares SPI2 with QMA6100P; access protected by spi2_mutex.
 *         Stack 256*4=1024 bytes: covers float printf and sensor init frames.
 */
osThreadId_t h3lis100dlTaskHandle;
const osThreadAttr_t h3lis100dlTask_attributes = {
  .name = "h3lis100dlTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartBlueBlinkTask(void *argument);
void StartGreenBlinkTask(void *argument);
void StartLsm6dsoxTask(void *argument);    /* LSM6DSOX  (suspended)  */
void StartQma6100pTask(void *argument);    /* QMA6100P  (active)     */
void StartH3lis100dlTask(void *argument);  /* H3LIS100DL (suspended) */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */


/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
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
/* place for user code */
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* SPI1 is dedicated to LSM6DSOX - no mutex needed.
     SPI2 is shared by H3LIS100DL and QMA6100P - must create mutex first,
     before the sensor threads start, so it is valid when they acquire it. */
  spi2_mutex = osMutexNew(&spi2_mutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of blueBlinkTask */
  blueBlinkTaskHandle = osThreadNew(StartBlueBlinkTask, NULL, &blueBlinkTask_attributes);

  /* creation of greenBlinkTask */
  greenBlinkTaskHandle = osThreadNew(StartGreenBlinkTask, NULL, &greenBlinkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /*
   * ========== Sensor thread creation ==========
   *
   * Current phase: QMA6100P driver validation only.
   *   - LSM6DSOX  : created then SUSPENDED (SPI1)
   *   - QMA6100P  : ACTIVE                 (SPI2, mutex protected)
   *   - H3LIS100DL: created then SUSPENDED (SPI2)
   *
   * To activate a suspended thread call osThreadResume(handle).
   */

  /* LSM6DSOX thread - suspended during QMA6100P validation */
  lsm6dsoxTaskHandle = osThreadNew(StartLsm6dsoxTask, NULL, &lsm6dsoxTask_attributes);
  osThreadSuspend(lsm6dsoxTaskHandle);

  /* H3LIS100DL thread - suspended during QMA6100P validation */
  h3lis100dlTaskHandle = osThreadNew(StartH3lis100dlTask, NULL, &h3lis100dlTask_attributes);
  osThreadSuspend(h3lis100dlTaskHandle);

  /* QMA6100P thread - ACTIVE for driver testing */
  qma6100pTaskHandle = osThreadNew(StartQma6100pTask, NULL, &qma6100pTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartBlueBlinkTask */
/**
* @brief Function implementing the blueBlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlueBlinkTask */
__weak void StartBlueBlinkTask(void *argument)
{
  /* USER CODE BEGIN blueBlinkTask */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END blueBlinkTask */
}

/* USER CODE BEGIN Header_StartGreenBlinkTask */
/**
* @brief Function implementing the greenBlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGreenBlinkTask */
__weak void StartGreenBlinkTask(void *argument)
{
  /* USER CODE BEGIN greenBlinkTask */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END greenBlinkTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ============================================================================
 *  Key press detection helper
 * ============================================================================
 *  PA12 key: idle=pull-up HIGH, pressed=GND LOW
 *  Detect falling edge + 10ms debounce + wait release, return 1 if confirmed
 *
 * @param  p_key_last: pointer to previous key state variable
 * @retval 1=confirmed press (and release waited), 0=no event
 */
static uint8_t Key_DetectPress(GPIO_PinState *p_key_last)
{
  GPIO_PinState key_now = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);

  if (*p_key_last == GPIO_PIN_SET && key_now == GPIO_PIN_RESET)
  {
    /* Falling edge detected, debounce */
    osDelay(10);
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      /* Wait for key release */
      while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
      {
        osDelay(10);
      }
      *p_key_last = GPIO_PIN_SET;
      return 1U;
    }
  }

  *p_key_last = key_now;
  return 0U;
}

/* ============================================================================
 *  LSM6DSOX sampling thread (currently the only active sensor)
 * ============================================================================
 *  Uses SPI1 (PA5/PA6/PA7 + PA4 CS) dedicated bus, no mutex needed.
 *  Features:
 *    1. Sensor init + configure (accel +/-4g/104Hz, gyro +/-2000dps/104Hz)
 *    2. PA12 key control: press=start sampling, press again=stop
 *    3. STATUS_REG XLDA flag check
 *    4. Burst read accel + gyro + temp (14 bytes)
 *    5. Loop period 10ms, key response is fast; sample interval 500ms
 */
void StartLsm6dsoxTask(void *argument)
{
  LSM6DSOX_AllData_t  all_data;                /* 完整数据 (加速度+陀螺仪+温度) */
  uint8_t  sampling_enable = 0;                /* 采样标志: 0=停止 1=运行 */
  uint32_t sample_tick_count = 0;              /* 采样间隔计数器 */
  GPIO_PinState key_last = GPIO_PIN_SET;       /* 上次按键状态 */

  (void)argument;

  /* ---- 传感器初始化 (SPI1专用总线, 无需互斥) ---- */
  printf("[LSM6DSOX] 正在初始化传感器...\r\n");

  if (LSM6DSOX_Init() != HAL_OK)
  {
    printf("[LSM6DSOX] 初始化失败, 请检查接线!\r\n");
  }

  printf("[LSM6DSOX] 任务已启动. 按键PA12: 按下=开始采集, 再按=停止采集\r\n\r\n");

  /* ====================== Main loop (10ms period) ====================== */
  for (;;)
  {
    /* ---- Key detection ---- */
    if (Key_DetectPress(&key_last))
    {
      sampling_enable = !sampling_enable;
      sample_tick_count = 0;
      printf("[按键] PA12 按下, LSM6DSOX采集: %s\r\n\r\n",
             sampling_enable ? ">> 开始" : ">> 停止");
    }

    /* ---- Data sampling (500ms interval) ---- */
    if (sampling_enable)
    {
      sample_tick_count++;

      if (sample_tick_count >= SAMPLE_INTERVAL_TICKS)
      {
        sample_tick_count = 0;
        uint8_t status_reg = 0;

        /* 检查数据就绪标志 */
        if (LSM6DSOX_ReadStatus(&status_reg) == HAL_OK &&
            (status_reg & LSM6DSOX_STATUS_XLDA))
        {
          /* 批量读取全部数据 (加速度 + 陀螺仪 + 温度) */
          if (LSM6DSOX_ReadAllData(&all_data) == HAL_OK)
          {
            printf("LSM6DSOX: 加速度(mg) X:%7.1f Y:%7.1f Z:%7.1f | "
                   "陀螺仪(mdps) X:%8.1f Y:%8.1f Z:%8.1f | 温度:%.1fC\r\n",
                   all_data.acc.x,  all_data.acc.y,  all_data.acc.z,
                   all_data.gyro.x, all_data.gyro.y, all_data.gyro.z,
                   all_data.temp_C);
          }
        }
      }
    }
    else
    {
      sample_tick_count = 0;
    }

    osDelay(KEY_POLL_PERIOD_MS);
  }
}

/* ============================================================================
 *  H3LIS100DL sampling thread
 * ============================================================================
 *  Uses SPI2 (PB13/PB14/PB15 + PB12 CS), shared with QMA6100P.
 *  Every SPI2 access sequence is bracketed by spi2_mutex acquire/release.
 *  Features:
 *    1. Sensor init (100g, 100Hz, XYZ all-axis)
 *    2. PA12 key control: press=start sampling, press again=stop
 *    3. Burst read X/Y/Z acceleration (3 bytes)
 *    4. Loop period 10ms (key responsive); sample interval 500ms
 */
void StartH3lis100dlTask(void *argument)
{
  H3LIS100DL_Data_t data;
  uint8_t           sampling_enable  = 0;
  uint32_t          sample_tick_count = 0;
  GPIO_PinState     key_last         = GPIO_PIN_SET;

  (void)argument;

  /* ---- Sensor init (hold mutex for the whole init sequence) ---- */
  printf("[H3LIS100DL] 正在初始化传感器 (SPI2)...\r\n");

  osMutexAcquire(spi2_mutex, osWaitForever);
  int init_ret = H3LIS100DL_Init();
  osMutexRelease(spi2_mutex);

  if (init_ret != 0)
  {
    printf("[H3LIS100DL] 初始化失败, 请检查接线!\r\n");
  }

  printf("[H3LIS100DL] 任务已启动. 按键PA12: 按下=开始采集, 再按=停止采集\r\n\r\n");

  /* ====================== Main loop (10ms period) ====================== */
  for (;;)
  {
    /* ---- Key detection ---- */
    if (Key_DetectPress(&key_last))
    {
      sampling_enable = !sampling_enable;
      sample_tick_count = 0;
      printf("[按键] PA12 按下, H3LIS100DL采集: %s\r\n\r\n",
             sampling_enable ? ">> 开始" : ">> 停止");
    }

    /* ---- Data sampling (500ms interval) ---- */
    if (sampling_enable)
    {
      sample_tick_count++;

      if (sample_tick_count >= SAMPLE_INTERVAL_TICKS)
      {
        sample_tick_count = 0;

        osMutexAcquire(spi2_mutex, osWaitForever);
        int ret = H3LIS100DL_ReadAccXYZ(&data);
        osMutexRelease(spi2_mutex);

        if (ret == 0)
        {
          printf("H3LIS100DL: raw[%4d,%4d,%4d]  acc(mg)[%7.1f,%7.1f,%7.1f]\r\n",
                 data.raw[0], data.raw[1], data.raw[2],
                 data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);
        }
        else if (ret == -2)
        {
          printf("[H3LIS100DL] 数据未就绪 (STATUS ZYXDA=0)\r\n");
        }
      }
    }
    else
    {
      sample_tick_count = 0;
    }

    osDelay(KEY_POLL_PERIOD_MS);
  }
}

/* ============================================================================
 *  QMA6100P sampling thread
 * ============================================================================
 *  Uses SPI2 (PB13/PB14/PB15 + PB0 CS), shared with H3LIS100DL.
 *  Every SPI2 access sequence is bracketed by spi2_mutex acquire/release.
 *  Features:
 *    1. Sensor init (4g, 100Hz, active mode)
 *    2. PA12 key control: press=start sampling, press again=stop
 *    3. Burst read X/Y/Z acceleration (6 bytes, 14-bit)
 *    4. Loop period 10ms (key responsive); sample interval 500ms
 */
void StartQma6100pTask(void *argument)
{
  QMA6100P_Data_t data;
  uint8_t         sampling_enable   = 0;
  uint32_t        sample_tick_count = 0;
  GPIO_PinState   key_last          = GPIO_PIN_SET;

  (void)argument;

  /* ---- Sensor init (hold mutex for the whole init sequence) ---- */
  printf("[QMA6100P] 正在初始化传感器 (SPI2)...\r\n");

  osMutexAcquire(spi2_mutex, osWaitForever);
  HAL_StatusTypeDef init_ret = QMA6100P_Init();
  osMutexRelease(spi2_mutex);

  if (init_ret != HAL_OK)
  {
    printf("[QMA6100P] 初始化失败, 请检查接线!\r\n");
  }

  printf("[QMA6100P] 任务已启动. 按键PA12: 按下=开始采集, 再按=停止采集\r\n\r\n");

  /* ====================== Main loop (10ms period) ====================== */
  for (;;)
  {
    /* ---- Key detection ---- */
    if (Key_DetectPress(&key_last))
    {
      sampling_enable = !sampling_enable;
      sample_tick_count = 0;
      printf("[按键] PA12 按下, QMA6100P采集: %s\r\n\r\n",
             sampling_enable ? ">> 开始" : ">> 停止");
    }

    /* ---- Data sampling (500ms interval) ---- */
    if (sampling_enable)
    {
      sample_tick_count++;

      if (sample_tick_count >= SAMPLE_INTERVAL_TICKS)
      {
        sample_tick_count = 0;

        osMutexAcquire(spi2_mutex, osWaitForever);
        HAL_StatusTypeDef ret = QMA6100P_ReadAccXYZ(&data);
        osMutexRelease(spi2_mutex);

        if (ret == HAL_OK)
        {
          printf("QMA6100P:  加速度(mg) X:%7.1f Y:%7.1f Z:%7.1f\r\n",
                 data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);
        }
      }
    }
    else
    {
      sample_tick_count = 0;
    }

    osDelay(KEY_POLL_PERIOD_MS);
  }
}

/* USER CODE END Application */
