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

/** Key polling period (ms) */
#define KEY_POLL_PERIOD_MS    10U

/** Sensor sampling interval (ms) */
#define SAMPLE_PERIOD_MS      500U

/** Number of polling ticks per sample interval */
#define SAMPLE_INTERVAL_TICKS (SAMPLE_PERIOD_MS / KEY_POLL_PERIOD_MS)

/** Sensor ready bits for g_sensor_ready_mask */
#define SENSOR_MASK_LSM6DSOX   (1U << 0)
#define SENSOR_MASK_H3LIS100DL (1U << 1)
#define SENSOR_MASK_QMA6100P   (1U << 2)
#define SENSOR_MASK_ALL        (0x07U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ======================== Bus architecture ================================
 *
 *   SPI1 (PA5/PA6/PA7 + PA4 CS)          -> LSM6DSOX   (dedicated bus)
 *   SPI2 (PB13/PB14/PB15 + PB12/PB0 CS)  -> H3LIS100DL + QMA6100P (shared)
 *
 *   SPI2 is shared: a mutex is required before every CS sequence.
 */

/** Mutex protecting the SPI2 shared bus */
static osMutexId_t spi2_mutex;
static const osMutexAttr_t spi2_mutex_attr = {
  .name      = "spi2Mutex",
  .attr_bits = osMutexPrioInherit,
};

/**
 * @brief  Sensor init ready flags.
 *         Bit 0 = LSM6DSOX, Bit 1 = H3LIS100DL, Bit 2 = QMA6100P.
 *         Set by each sensor task after successful init.
 *         Read by defaultTask to determine when to enable key control.
 */
static volatile uint8_t g_sensor_ready_mask = 0U;

/**
 * @brief  Global sampling enable flag.
 *         Toggled by defaultTask when PA12 is pressed (only after all 3 sensors ready).
 *         Read by all sensor tasks.
 */
static volatile uint8_t g_sampling_enable = 0U;

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
 * @brief  LSM6DSOX thread  (SPI1 dedicated bus, no mutex needed)
 *         Stack: 2048 bytes (for float printf)
 */
osThreadId_t lsm6dsoxTaskHandle;
const osThreadAttr_t lsm6dsoxTask_attributes = {
  .name = "lsm6dsoxTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};

/**
 * @brief  QMA6100P thread  (SPI2 shared bus, mutex protected)
 *         Stack: 1024 bytes
 */
osThreadId_t qma6100pTaskHandle;
const osThreadAttr_t qma6100pTask_attributes = {
  .name = "qma6100pTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/**
 * @brief  H3LIS100DL thread  (SPI2 shared bus, mutex protected)
 *         Stack: 1024 bytes
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
void StartLsm6dsoxTask(void *argument);
void StartQma6100pTask(void *argument);
void StartH3lis100dlTask(void *argument);

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
  /* SPI2 is shared by H3LIS100DL and QMA6100P - create mutex before sensor threads */
  spi2_mutex = osMutexNew(&spi2_mutex_attr);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of blueBlinkTask */
  blueBlinkTaskHandle = osThreadNew(StartBlueBlinkTask, NULL, &blueBlinkTask_attributes);

  /* creation of greenBlinkTask */
  greenBlinkTaskHandle = osThreadNew(StartGreenBlinkTask, NULL, &greenBlinkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* Three sensor threads, all active concurrently.
   * Each thread initializes its own sensor, then sets its ready bit in
   * g_sensor_ready_mask.  The defaultTask starts key-controlled sampling
   * only after all three bits are set (SENSOR_MASK_ALL). */
  lsm6dsoxTaskHandle  = osThreadNew(StartLsm6dsoxTask,  NULL, &lsm6dsoxTask_attributes);
  h3lis100dlTaskHandle = osThreadNew(StartH3lis100dlTask, NULL, &h3lis100dlTask_attributes);
  qma6100pTaskHandle  = osThreadNew(StartQma6100pTask,  NULL, &qma6100pTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ============================================================================
 *  Key press detection helper
 * ============================================================================
 *  PA12: idle=pull-up HIGH, pressed=GND LOW
 *  Detects falling edge + 10ms debounce + waits for release.
 *
 * @param  p_key_last  pointer to previous key state variable (caller owns)
 * @retval 1 = confirmed press-and-release cycle, 0 = no event
 */
static uint8_t Key_DetectPress(GPIO_PinState *p_key_last)
{
  GPIO_PinState key_now = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);

  if (*p_key_last == GPIO_PIN_SET && key_now == GPIO_PIN_RESET)
  {
    /* Falling edge detected - debounce */
    osDelay(10);
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      /* Wait for release */
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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
*        Handles PA12 key detection and broadcasts g_sampling_enable to all
*        sensor threads.  Sampling is only allowed after all three sensors
*        have successfully initialized (g_sensor_ready_mask == SENSOR_MASK_ALL).
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  GPIO_PinState key_last        = GPIO_PIN_SET;
  uint8_t       all_ready_notified = 0U;

  (void)argument;

  printf("[SYS] 系统启动, 等待三个传感器初始化...\r\n\r\n");

  for(;;)
  {
    /* Notify once when all sensors are ready */
    if (!all_ready_notified && (g_sensor_ready_mask == SENSOR_MASK_ALL))
    {
      all_ready_notified = 1U;
      printf("[SYS] 所有传感器初始化完成! 按下PA12开始采集\r\n\r\n");
    }

    /* Key detection - only effective after all sensors ready */
    if (all_ready_notified)
    {
      if (Key_DetectPress(&key_last))
      {
        g_sampling_enable = !g_sampling_enable;
        printf("[按键] PA12 按下, 采集: %s\r\n\r\n",
               g_sampling_enable ? ">> 开始" : ">> 停止");
      }
    }

    osDelay(KEY_POLL_PERIOD_MS);
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

/* ============================================================================
 *  LSM6DSOX sampling thread
 * ============================================================================
 *  SPI1 dedicated bus (PA5/PA6/PA7 + PA4 CS) - no mutex needed.
 *  Waits for g_sampling_enable (set by defaultTask via PA12 key).
 *  Sample interval: 500 ms.
 */
void StartLsm6dsoxTask(void *argument)
{
  LSM6DSOX_AllData_t all_data;
  uint32_t           sample_tick_count = 0U;

  (void)argument;

  /* Sensor init */
  if (LSM6DSOX_Init() != HAL_OK)
  {
    /* Driver already printed the error */
    osThreadTerminate(NULL);
    return;
  }

  /* Signal init complete */
  taskENTER_CRITICAL();
  g_sensor_ready_mask |= SENSOR_MASK_LSM6DSOX;
  taskEXIT_CRITICAL();

  /* ====================== Main loop (10ms period) ====================== */
  for (;;)
  {
    if (g_sampling_enable)
    {
      sample_tick_count++;

      if (sample_tick_count >= SAMPLE_INTERVAL_TICKS)
      {
        sample_tick_count = 0U;
        uint8_t status_reg = 0U;

        if (LSM6DSOX_ReadStatus(&status_reg) == HAL_OK &&
            (status_reg & LSM6DSOX_STATUS_XLDA))
        {
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
      sample_tick_count = 0U;
    }

    osDelay(KEY_POLL_PERIOD_MS);
  }
}

/* ============================================================================
 *  H3LIS100DL sampling thread
 * ============================================================================
 *  SPI2 shared bus (PB13/PB14/PB15 + PB12 CS), protected by spi2_mutex.
 *  Waits for g_sampling_enable (set by defaultTask via PA12 key).
 *  Sample interval: 500 ms.
 */
void StartH3lis100dlTask(void *argument)
{
  H3LIS100DL_Data_t data;
  uint32_t          sample_tick_count = 0U;

  (void)argument;

  /* Sensor init - hold mutex for the whole init sequence */
  osMutexAcquire(spi2_mutex, osWaitForever);
  int init_ret = H3LIS100DL_Init();
  osMutexRelease(spi2_mutex);

  if (init_ret != 0)
  {
    /* Driver already printed the error */
    osThreadTerminate(NULL);
    return;
  }

  /* Signal init complete */
  taskENTER_CRITICAL();
  g_sensor_ready_mask |= SENSOR_MASK_H3LIS100DL;
  taskEXIT_CRITICAL();

  /* ====================== Main loop (10ms period) ====================== */
  for (;;)
  {
    if (g_sampling_enable)
    {
      sample_tick_count++;

      if (sample_tick_count >= SAMPLE_INTERVAL_TICKS)
      {
        sample_tick_count = 0U;

        osMutexAcquire(spi2_mutex, osWaitForever);
        int ret = H3LIS100DL_ReadAccXYZ(&data);
        osMutexRelease(spi2_mutex);

        if (ret == 0)
        {
          printf("H3LIS100DL: raw[%4d,%4d,%4d]  acc(mg)[%7.1f,%7.1f,%7.1f]\r\n",
                 data.raw[0], data.raw[1], data.raw[2],
                 data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);
        }
      }
    }
    else
    {
      sample_tick_count = 0U;
    }

    osDelay(KEY_POLL_PERIOD_MS);
  }
}

/* ============================================================================
 *  QMA6100P sampling thread
 * ============================================================================
 *  SPI2 shared bus (PB13/PB14/PB15 + PB0 CS), protected by spi2_mutex.
 *  Waits for g_sampling_enable (set by defaultTask via PA12 key).
 *  Sample interval: 500 ms.
 */
void StartQma6100pTask(void *argument)
{
  QMA6100P_Data_t data;
  uint32_t        sample_tick_count = 0U;

  (void)argument;

  /* Sensor init - hold mutex for the whole init sequence */
  osMutexAcquire(spi2_mutex, osWaitForever);
  HAL_StatusTypeDef init_ret = QMA6100P_Init();
  osMutexRelease(spi2_mutex);

  if (init_ret != HAL_OK)
  {
    /* Driver already printed the error */
    osThreadTerminate(NULL);
    return;
  }

  /* Signal init complete */
  taskENTER_CRITICAL();
  g_sensor_ready_mask |= SENSOR_MASK_QMA6100P;
  taskEXIT_CRITICAL();

  /* ====================== Main loop (10ms period) ====================== */
  for (;;)
  {
    if (g_sampling_enable)
    {
      sample_tick_count++;

      if (sample_tick_count >= SAMPLE_INTERVAL_TICKS)
      {
        sample_tick_count = 0U;

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
      sample_tick_count = 0U;
    }

    osDelay(KEY_POLL_PERIOD_MS);
  }
}

/* USER CODE END Application */
