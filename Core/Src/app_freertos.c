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
#include <string.h>
#include <stdlib.h>
#include "fatfs_sd.h"
#include "sensor_snapshot.h"
#include "lsm6dsox.h"
#include "h3lis100dl.h"
#include "qma6100p.h"
#include "usb_cdc_service.h"
#include "dma_sampling.h"
#include "acq_config.h"
#include "device_config.h"
#include "boot_mode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_PERIOD_MS         APP_SENSOR_SAMPLE_PERIOD_MS
#define LOGGER_RETRY_DELAY_MS    1000U
#define LOGGER_SYNC_EVERY_ROWS   32U
#define USB_CDC_POLL_DELAY_MS    10U
#define USB_CDC_ALIVE_PERIOD_MS  1000U
#define USB_CDC_TX_BUFFER_SIZE   96U
#define USB_CDC_RX_BUFFER_SIZE   128U
#define USB_CDC_CMD_BUFFER_SIZE  64U
#define USB_UPLOAD_PERIOD_MS     5U
#define APP_ACQ_IDLE_DELAY_MS    10U
#define APP_ACQ_SINK_USB         1U
#define APP_ACQ_SINK_SD          2U
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
static osMutexId_t snapshot_mutex;
static osMutexId_t frame_buffer_mutex;
static osMutexId_t acq_ctrl_mutex;
static osSemaphoreId_t s_lsm_fifo_sem;  /* released by EXTI1 ISR on PB1 rising edge */
static AppSensorSnapshot_t g_sensor_snapshot;

typedef struct
{
  uint8_t running;
  uint8_t sink;
  uint8_t stop_pending;
  uint32_t requested_hz;
  uint32_t period_ms;
  uint32_t effective_hz;
  uint32_t duration_ms;
  uint32_t start_tick_ms;
  uint32_t stop_tick_ms;
  uint32_t last_stop_tick_ms;
} AppAcqControl_t;

typedef struct
{
  AppSensorFrame_t frames[APP_SENSOR_FRAME_BUFFER_DEPTH];
  uint32_t head;
  uint32_t tail;
  uint32_t count;
  uint32_t dropped;
  uint32_t high_watermark;
} AppFrameBuffer_t;

typedef struct
{
  AppLsmBatch_t batches[APP_LSM_BATCH_BUFFER_DEPTH];
  uint32_t head;
  uint32_t tail;
  uint32_t count;
  uint32_t dropped;
  uint32_t high_watermark;
} AppLsmBatchBuffer_t;

typedef struct
{
  uint32_t lsm_updates;
  uint32_t h3_updates;
  uint32_t qma_updates;
  uint32_t logger_rows;
  uint32_t logger_write_failures;
  uint32_t stale_rows;
  uint32_t mixed_rows;
  uint32_t coherent_rows;
  uint32_t last_log_tick_ms;
  uint32_t last_changed_mask;
  uint32_t last_fresh_mask;
  uint32_t last_max_delta_ms;
  uint32_t last_log_seq;
  uint32_t sd_frames_written;
  uint32_t usb_frames_sent;
  uint32_t frame_buffer_depth;
  uint32_t frame_dropped;
  uint32_t frame_high_watermark;
  uint32_t frame_id;
  uint8_t usb_streaming_active;
  uint8_t sd_logging_active;
} AppFlowStats_t;

static AppFlowStats_t g_flow_stats;
static AppAcqControl_t g_acq_ctrl;
static AppFrameBuffer_t g_frame_buffer;
static AppLsmBatchBuffer_t g_lsm_batch_buffer;
static const osMutexAttr_t spi2_mutex_attr = {
  .name      = "spi2Mutex",
  .attr_bits = osMutexPrioInherit,
};
static const osMutexAttr_t snapshot_mutex_attr = {
  .name      = "snapshotMutex",
  .attr_bits = osMutexPrioInherit,
};
static const osMutexAttr_t frame_buffer_mutex_attr = {
  .name      = "frameBufferMutex",
  .attr_bits = osMutexPrioInherit,
};
static const osMutexAttr_t acq_ctrl_mutex_attr = {
  .name      = "acqCtrlMutex",
  .attr_bits = osMutexPrioInherit,
};

/* USER CODE END Variables */

/* ======================== Sensor threads ================================== */
osThreadId_t lsm6dsoxTaskHandle;
const osThreadAttr_t lsm6dsoxTask_attributes = {
  .name = "lsm6dsoxTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 2048 * 4  /* 8KB — holds AppLsmBatch_t (~1.5KB) + fifo_buf (1.8KB) */
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

osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes = {
  .name = "loggerTask",
  .priority = (osPriority_t)osPriorityNormal,
  .stack_size = 4096 * 4  /* 16KB — large enough to hold AppLsmBatch_t (~1.5KB) on stack */
};

osThreadId_t usbCdcTaskHandle;
const osThreadAttr_t usbCdcTask_attributes = {
  .name = "usbCdcTask",
  .priority = (osPriority_t)osPriorityAboveNormal,  /* 提高优先级测试 */
  .stack_size = 1536 * 4  /* 6KB */
};

osThreadId_t usbUploadTaskHandle;
const osThreadAttr_t usbUploadTask_attributes = {
  .name = "usbUploadTask",
  .priority = (osPriority_t)osPriorityNormal,
  .stack_size = 1024 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static uint32_t AppFrameBufferPush(const AppSensorFrame_t *frame);
static uint32_t AppFrameBufferPop(AppSensorFrame_t *frame);
static uint32_t AppLsmBatchPush(const AppLsmBatch_t *batch);
static uint32_t AppLsmBatchPop(AppLsmBatch_t *batch);
static void AppFramePopulateLsm6dsox(AppSensorFrame_t *frame, const LSM6DSOX_AllData_t *data, uint32_t tick_ms);
static void AppFramePopulateH3lis100dl(AppSensorFrame_t *frame, const H3LIS100DL_Data_t *data, uint32_t tick_ms);
static void AppFramePopulateQma6100p(AppSensorFrame_t *frame, const QMA6100P_Data_t *data, uint32_t tick_ms);
static uint32_t AppAcqResolvePeriodMs(uint32_t requested_hz);
static uint32_t AppAcqResolveEffectiveHz(uint32_t period_ms);
static const char *AppAcqSinkToString(uint8_t sink);
static uint8_t AppAcqParseSink(const char *text, uint8_t *sink_out);
static void AppAcqGetCopy(AppAcqControl_t *ctrl);
static uint32_t AppAcqIsRunning(void);
static uint32_t AppAcqCurrentPeriodMs(void);
static uint8_t AppAcqIsUsbSinkActive(void);
static uint8_t AppAcqIsSdSinkActive(void);
static uint8_t AppAcqIsSdSessionActive(void);
static uint8_t AppUsbRawStreamingActive(void);
static void AppFlowStatsSetMode(uint8_t usb_active, uint8_t sd_active);
static void AppLoggerStopSdSession(uint8_t *sd_file_open, uint32_t *rows_since_sync);
static void AppAcqStopInternal(uint32_t now_ms);
static void AppAcqCheckAutoStop(void);
static uint32_t AppAcqStart(uint8_t sink, uint32_t requested_hz, uint32_t duration_ms);
static uint32_t AppAcqStop(void);
static uint32_t AppAcqDrainPendingStop(void);
static void UsbCmd_AcqStatus(void);
static void UsbCmd_AcqStart(const char *cmd);
static void UsbCmd_AcqStop(void);


void StartLsm6dsoxTask(void *argument);
void StartQma6100pTask(void *argument);
void StartH3lis100dlTask(void *argument);
void StartLoggerTask(void *argument);
void StartUsbCdcTask(void *argument);
void StartUsbUploadTask(void *argument);

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
  AcqConfig_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  snapshot_mutex = osMutexNew(&snapshot_mutex_attr);
  frame_buffer_mutex = osMutexNew(&frame_buffer_mutex_attr);
  acq_ctrl_mutex = osMutexNew(&acq_ctrl_mutex_attr);
#if (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_NONE) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P)
  spi2_mutex = osMutexNew(&spi2_mutex_attr);
#endif
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  s_lsm_fifo_sem = osSemaphoreNew(1, 0, NULL);  /* binary semaphore, init=0 */
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
  loggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &loggerTask_attributes);
  usbCdcTaskHandle = osThreadNew(StartUsbCdcTask, NULL, &usbCdcTask_attributes);
  usbUploadTaskHandle = osThreadNew(StartUsbUploadTask, NULL, &usbUploadTask_attributes);
  printf("[RTOS] usbCdcTask created: %s\r\n", (usbCdcTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] usbUploadTask created: %s\r\n", (usbUploadTaskHandle != NULL) ? "ok" : "FAILED");
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* EXTI rising-edge callback. PB1 is wired to LSM6DSOX INT1 (FIFO watermark). */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_1)
  {
    if (s_lsm_fifo_sem != NULL)
    {
      osSemaphoreRelease(s_lsm_fifo_sem);
    }
  }
}

static void AppFramePopulateLsm6dsox(AppSensorFrame_t *frame, const LSM6DSOX_AllData_t *data, uint32_t tick_ms)
{
  if ((snapshot_mutex == NULL) || (frame == NULL) || (data == NULL))
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_sensor_snapshot.lsm6dsox.data = *data;
  g_sensor_snapshot.lsm6dsox.valid = 1U;
  g_sensor_snapshot.lsm6dsox.sample_seq++;
  g_sensor_snapshot.lsm6dsox.last_update_ms = tick_ms;
  g_flow_stats.lsm_updates = g_sensor_snapshot.lsm6dsox.sample_seq;

  frame->lsm6dsox.valid = 1U;
  frame->lsm6dsox.sample_seq = g_sensor_snapshot.lsm6dsox.sample_seq;
  frame->lsm6dsox.data = *data;
  frame->present_mask |= APP_SENSOR_MASK_LSM6DSOX;
  osMutexRelease(snapshot_mutex);
}

static void AppFramePopulateH3lis100dl(AppSensorFrame_t *frame, const H3LIS100DL_Data_t *data, uint32_t tick_ms)
{
  if ((snapshot_mutex == NULL) || (frame == NULL) || (data == NULL))
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_sensor_snapshot.h3lis100dl.data = *data;
  g_sensor_snapshot.h3lis100dl.valid = 1U;
  g_sensor_snapshot.h3lis100dl.sample_seq++;
  g_sensor_snapshot.h3lis100dl.last_update_ms = tick_ms;
  g_flow_stats.h3_updates = g_sensor_snapshot.h3lis100dl.sample_seq;

  frame->h3lis100dl.valid = 1U;
  frame->h3lis100dl.sample_seq = g_sensor_snapshot.h3lis100dl.sample_seq;
  frame->h3lis100dl.data = *data;
  frame->present_mask |= APP_SENSOR_MASK_H3LIS100DL;
  osMutexRelease(snapshot_mutex);
}

static void AppFramePopulateQma6100p(AppSensorFrame_t *frame, const QMA6100P_Data_t *data, uint32_t tick_ms)
{
  if ((snapshot_mutex == NULL) || (frame == NULL) || (data == NULL))
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_sensor_snapshot.qma6100p.data = *data;
  g_sensor_snapshot.qma6100p.valid = 1U;
  g_sensor_snapshot.qma6100p.sample_seq++;
  g_sensor_snapshot.qma6100p.last_update_ms = tick_ms;
  g_flow_stats.qma_updates = g_sensor_snapshot.qma6100p.sample_seq;

  frame->qma6100p.valid = 1U;
  frame->qma6100p.sample_seq = g_sensor_snapshot.qma6100p.sample_seq;
  frame->qma6100p.data = *data;
  frame->present_mask |= APP_SENSOR_MASK_QMA6100P;
  osMutexRelease(snapshot_mutex);
}

static void AppSnapshotPublishLsm6dsox(const LSM6DSOX_AllData_t *data)
{
  (void)data;
}

static void AppSnapshotPublishH3lis100dl(const H3LIS100DL_Data_t *data)
{
  (void)data;
}

static void AppSnapshotPublishQma6100p(const QMA6100P_Data_t *data)
{
  (void)data;
}

static void AppSnapshotCopy(AppSensorSnapshot_t *snapshot)
{
  if ((snapshot_mutex == NULL) || (snapshot == NULL))
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  *snapshot = g_sensor_snapshot;
  osMutexRelease(snapshot_mutex);
}

static void AppSnapshotReset(void)
{
  if (snapshot_mutex == NULL)
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  memset(&g_sensor_snapshot, 0, sizeof(g_sensor_snapshot));
  memset(&g_flow_stats, 0, sizeof(g_flow_stats));
  osMutexRelease(snapshot_mutex);

  if (frame_buffer_mutex != NULL)
  {
    osMutexAcquire(frame_buffer_mutex, osWaitForever);
    memset(&g_frame_buffer, 0, sizeof(g_frame_buffer));
    osMutexRelease(frame_buffer_mutex);
  }
}

static uint32_t AppAcqResolvePeriodMs(uint32_t requested_hz)
{
  if (requested_hz == 0U)
  {
    return SAMPLE_PERIOD_MS;
  }

  if (requested_hz >= 1000U)
  {
    return 1U;
  }

  {
    uint32_t period_ms = (1000U + requested_hz - 1U) / requested_hz;
    return (period_ms == 0U) ? 1U : period_ms;
  }
}

static uint32_t AppAcqResolveEffectiveHz(uint32_t period_ms)
{
  if (period_ms == 0U)
  {
    return 0U;
  }

  return 1000U / period_ms;
}

static const char *AppAcqSinkToString(uint8_t sink)
{
  switch (sink)
  {
    case APP_ACQ_SINK_USB:
      return "usb";
    case APP_ACQ_SINK_SD:
      return "sd";
    default:
      return "none";
  }
}

static uint8_t AppAcqParseSink(const char *text, uint8_t *sink_out)
{
  if ((text == NULL) || (sink_out == NULL))
  {
    return 0U;
  }

  if (strcmp(text, "usb") == 0)
  {
    *sink_out = APP_ACQ_SINK_USB;
    return 1U;
  }

  if (strcmp(text, "sd") == 0)
  {
    *sink_out = APP_ACQ_SINK_SD;
    return 1U;
  }

  return 0U;
}

static void AppAcqGetCopy(AppAcqControl_t *ctrl)
{
  if ((acq_ctrl_mutex == NULL) || (ctrl == NULL))
  {
    if (ctrl != NULL)
    {
      memset(ctrl, 0, sizeof(*ctrl));
    }
    return;
  }

  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  *ctrl = g_acq_ctrl;
  osMutexRelease(acq_ctrl_mutex);
}

static uint32_t AppAcqIsRunning(void)
{
  uint32_t running = 0U;

  if (acq_ctrl_mutex == NULL)
  {
    return 0U;
  }

  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  running = g_acq_ctrl.running;
  osMutexRelease(acq_ctrl_mutex);
  return running;
}

static uint32_t AppAcqCurrentPeriodMs(void)
{
  uint32_t period_ms = SAMPLE_PERIOD_MS;

  if (acq_ctrl_mutex == NULL)
  {
    return SAMPLE_PERIOD_MS;
  }

  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  if (g_acq_ctrl.period_ms != 0U)
  {
    period_ms = g_acq_ctrl.period_ms;
  }
  osMutexRelease(acq_ctrl_mutex);
  return period_ms;
}

static uint8_t AppAcqIsUsbSinkActive(void)
{
  AppAcqControl_t ctrl;

  AppAcqGetCopy(&ctrl);
  return (uint8_t)((ctrl.running != 0U) &&
                   (ctrl.sink == APP_ACQ_SINK_USB) &&
                   (AppUsbRawStreamingActive() != 0U));
}

static uint8_t AppAcqIsSdSinkActive(void)
{
  AppAcqControl_t ctrl;

  AppAcqGetCopy(&ctrl);
  return (uint8_t)((ctrl.running != 0U) && (ctrl.sink == APP_ACQ_SINK_SD));
}

static uint8_t AppAcqIsSdSessionActive(void)
{
  AppAcqControl_t ctrl;

  AppAcqGetCopy(&ctrl);
  return (uint8_t)((ctrl.running != 0U) && (ctrl.sink == APP_ACQ_SINK_SD));
}

static void AppLoggerStopSdSession(uint8_t *sd_file_open, uint32_t *rows_since_sync)
{
  if (sd_file_open == NULL)
  {
    return;
  }

  if (*sd_file_open != 0U)
  {
    (void)DeviceCfg_WriteCurrentToSD();
    FatFs_SD_LoggerStop();
    *sd_file_open = 0U;
  }

  if (rows_since_sync != NULL)
  {
    *rows_since_sync = 0U;
  }
}

static void AppAcqStopInternal(uint32_t now_ms)
{
  if (acq_ctrl_mutex == NULL)
  {
    return;
  }

  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  if (g_acq_ctrl.running != 0U)
  {
    g_acq_ctrl.running = 0U;
    g_acq_ctrl.stop_tick_ms = now_ms;
    g_acq_ctrl.last_stop_tick_ms = now_ms;
    g_acq_ctrl.stop_pending = 1U;
  }
  osMutexRelease(acq_ctrl_mutex);
}

static uint32_t AppAcqDrainPendingStop(void)
{
  uint32_t stop_pending = 0U;

  if (acq_ctrl_mutex == NULL)
  {
    return 0U;
  }

  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  stop_pending = g_acq_ctrl.stop_pending;
  g_acq_ctrl.stop_pending = 0U;
  osMutexRelease(acq_ctrl_mutex);

  return stop_pending;
}

static void AppAcqCheckAutoStop(void)
{
  AppAcqControl_t ctrl;
  uint32_t now_ms = osKernelGetTickCount();

  AppAcqGetCopy(&ctrl);
  if ((ctrl.running == 0U) || (ctrl.duration_ms == 0U))
  {
    return;
  }

  if ((now_ms - ctrl.start_tick_ms) >= ctrl.duration_ms)
  {
    AppAcqStopInternal(now_ms);
  }
}
static uint32_t AppAcqStart(uint8_t sink, uint32_t requested_hz, uint32_t duration_ms)
{
  uint32_t now_ms = osKernelGetTickCount();
  uint32_t period_ms;
  uint32_t effective_hz;

  if ((sink != APP_ACQ_SINK_USB) && (sink != APP_ACQ_SINK_SD))
  {
    return 0U;
  }

  period_ms = AppAcqResolvePeriodMs(requested_hz);
  effective_hz = AppAcqResolveEffectiveHz(period_ms);

  if (acq_ctrl_mutex == NULL)
  {
    return 0U;
  }

  AppSnapshotReset();

  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  g_acq_ctrl.running = 1U;
  g_acq_ctrl.sink = sink;
  g_acq_ctrl.requested_hz = requested_hz;
  g_acq_ctrl.period_ms = period_ms;
  g_acq_ctrl.effective_hz = effective_hz;
  g_acq_ctrl.duration_ms = duration_ms;
  g_acq_ctrl.start_tick_ms = now_ms;
  osMutexRelease(acq_ctrl_mutex);

  AppFlowStatsSetMode((uint8_t)((sink == APP_ACQ_SINK_USB) ? 1U : 0U),
                      (uint8_t)((sink == APP_ACQ_SINK_SD) ? 1U : 0U));
  return 1U;
}

static uint32_t AppAcqStop(void)
{
  AppAcqControl_t ctrl;

  AppAcqGetCopy(&ctrl);
  if (ctrl.running == 0U)
  {
    return 0U;
  }

  AppAcqStopInternal(osKernelGetTickCount());
  AppFlowStatsSetMode(0U, 0U);
  return 1U;
}

static void UsbCmd_AcqStatus(void)
{
  char line[160];
  int len;
  uint32_t now_ms = osKernelGetTickCount();
  AppAcqControl_t ctrl;
  uint32_t elapsed_ms = 0U;
  uint32_t remaining_ms = 0U;

  AppAcqCheckAutoStop();
  AppAcqGetCopy(&ctrl);

  if (ctrl.start_tick_ms != 0U)
  {
    uint32_t end_tick = (ctrl.running != 0U) ? now_ms : ctrl.last_stop_tick_ms;
    elapsed_ms = end_tick - ctrl.start_tick_ms;
  }

  if ((ctrl.running != 0U) && (ctrl.duration_ms > elapsed_ms))
  {
    remaining_ms = ctrl.duration_ms - elapsed_ms;
  }

#define USB_ACQ_LINE(fmt, ...) \
  do { \
    len = snprintf(line, sizeof(line), fmt, ##__VA_ARGS__); \
    if (len > 0 && (uint32_t)len < sizeof(line)) { \
      UsbCdcService_Write((const uint8_t *)line, (uint32_t)len); \
    } \
  } while (0)

  USB_ACQ_LINE("ACQ state=%s sink=%s req_hz=%lu eff_period_ms=%lu eff_hz=%lu duration_ms=%lu elapsed_ms=%lu remaining_ms=%lu\r\n",
               (ctrl.running != 0U) ? "running" : "stopped",
               AppAcqSinkToString(ctrl.sink),
               (unsigned long)ctrl.requested_hz,
               (unsigned long)ctrl.period_ms,
               (unsigned long)ctrl.effective_hz,
               (unsigned long)ctrl.duration_ms,
               (unsigned long)elapsed_ms,
               (unsigned long)remaining_ms);

#undef USB_ACQ_LINE
}

static void UsbCmd_AcqStart(const char *cmd)
{
  char sink_text[8];
  unsigned long requested_hz = 0UL;
  unsigned long duration_ms = 0UL;
  uint8_t sink = 0U;
  char line[128];
  int len;

  if (sscanf(cmd, "acq_start %7s %lu %lu", sink_text, &requested_hz, &duration_ms) != 3)
  {
    const char *msg = "Usage: acq_start <usb|sd> <freq_hz> <duration_ms>\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
    return;
  }

  if ((AppAcqParseSink(sink_text, &sink) == 0U) || (requested_hz == 0UL))
  {
    const char *msg = "ERR invalid acq_start args\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
    return;
  }

  if (AppAcqStart(sink, (uint32_t)requested_hz, (uint32_t)duration_ms) == 0U)
  {
    const char *msg = "ERR acq_start failed\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
    return;
  }

  len = snprintf(line, sizeof(line),
                 "OK acq_start sink=%s req_hz=%lu eff_period_ms=%lu eff_hz=%lu duration_ms=%lu\r\n",
                 AppAcqSinkToString(sink),
                 requested_hz,
                 (unsigned long)AppAcqResolvePeriodMs((uint32_t)requested_hz),
                 (unsigned long)AppAcqResolveEffectiveHz(AppAcqResolvePeriodMs((uint32_t)requested_hz)),
                 duration_ms);
  if ((len > 0) && ((uint32_t)len < sizeof(line)))
  {
    UsbCdcService_Write((const uint8_t *)line, (uint32_t)len);
  }
}

static void UsbCmd_AcqStop(void)
{
  const char *msg;

  AppAcqCheckAutoStop();
  msg = (AppAcqStop() != 0U) ? "OK acq_stop\r\n" : "ACQ already stopped\r\n";
  UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
}

static void AppFlowStatsUpdateBufferStatsLocked(void)
{
  g_flow_stats.frame_buffer_depth = g_frame_buffer.count;
  g_flow_stats.frame_dropped = g_frame_buffer.dropped;
  g_flow_stats.frame_high_watermark = g_frame_buffer.high_watermark;
}

static uint32_t AppFrameBufferPush(const AppSensorFrame_t *frame)
{
  if ((frame_buffer_mutex == NULL) || (frame == NULL))
  {
    return 0U;
  }

  osMutexAcquire(frame_buffer_mutex, osWaitForever);

  if (g_frame_buffer.count >= APP_SENSOR_FRAME_BUFFER_DEPTH)
  {
    g_frame_buffer.dropped++;
    AppFlowStatsUpdateBufferStatsLocked();
    osMutexRelease(frame_buffer_mutex);
    return 0U;
  }

  g_frame_buffer.frames[g_frame_buffer.head] = *frame;
  g_frame_buffer.head = (g_frame_buffer.head + 1U) % APP_SENSOR_FRAME_BUFFER_DEPTH;
  g_frame_buffer.count++;
  if (g_frame_buffer.count > g_frame_buffer.high_watermark)
  {
    g_frame_buffer.high_watermark = g_frame_buffer.count;
  }

  AppFlowStatsUpdateBufferStatsLocked();
  osMutexRelease(frame_buffer_mutex);
  return 1U;
}

static uint32_t AppFrameBufferPop(AppSensorFrame_t *frame)
{
  if ((frame_buffer_mutex == NULL) || (frame == NULL))
  {
    return 0U;
  }

  osMutexAcquire(frame_buffer_mutex, osWaitForever);

  if (g_frame_buffer.count == 0U)
  {
    AppFlowStatsUpdateBufferStatsLocked();
    osMutexRelease(frame_buffer_mutex);
    return 0U;
  }

  *frame = g_frame_buffer.frames[g_frame_buffer.tail];
  g_frame_buffer.tail = (g_frame_buffer.tail + 1U) % APP_SENSOR_FRAME_BUFFER_DEPTH;
  g_frame_buffer.count--;
  AppFlowStatsUpdateBufferStatsLocked();
  osMutexRelease(frame_buffer_mutex);
  return 1U;
}

static uint32_t AppLsmBatchPush(const AppLsmBatch_t *batch)
{
  if ((frame_buffer_mutex == NULL) || (batch == NULL)) return 0U;

  osMutexAcquire(frame_buffer_mutex, osWaitForever);

  if (g_lsm_batch_buffer.count >= APP_LSM_BATCH_BUFFER_DEPTH)
  {
    g_lsm_batch_buffer.dropped++;
    osMutexRelease(frame_buffer_mutex);
    return 0U;
  }

  g_lsm_batch_buffer.batches[g_lsm_batch_buffer.head] = *batch;
  g_lsm_batch_buffer.head = (g_lsm_batch_buffer.head + 1U) % APP_LSM_BATCH_BUFFER_DEPTH;
  g_lsm_batch_buffer.count++;
  if (g_lsm_batch_buffer.count > g_lsm_batch_buffer.high_watermark)
  {
    g_lsm_batch_buffer.high_watermark = g_lsm_batch_buffer.count;
  }
  osMutexRelease(frame_buffer_mutex);
  return 1U;
}

static uint32_t AppLsmBatchPop(AppLsmBatch_t *batch)
{
  if ((frame_buffer_mutex == NULL) || (batch == NULL)) return 0U;

  osMutexAcquire(frame_buffer_mutex, osWaitForever);

  if (g_lsm_batch_buffer.count == 0U)
  {
    osMutexRelease(frame_buffer_mutex);
    return 0U;
  }

  *batch = g_lsm_batch_buffer.batches[g_lsm_batch_buffer.tail];
  g_lsm_batch_buffer.tail = (g_lsm_batch_buffer.tail + 1U) % APP_LSM_BATCH_BUFFER_DEPTH;
  g_lsm_batch_buffer.count--;
  osMutexRelease(frame_buffer_mutex);
  return 1U;
}

static uint32_t AppSnapshotComputeAgeMs(uint32_t tick_ms, uint32_t last_update_ms)
{
  return tick_ms - last_update_ms;
}

static uint32_t AppSnapshotComputeFreshMask(const AppSensorSnapshot_t *snapshot, uint32_t tick_ms)
{
  uint32_t mask = 0U;

  if ((snapshot->lsm6dsox.valid != 0U) &&
      (AppSnapshotComputeAgeMs(tick_ms, snapshot->lsm6dsox.last_update_ms) <= APP_SENSOR_STALE_TIMEOUT_MS))
  {
    mask |= (1U << 0);
  }

  if ((snapshot->h3lis100dl.valid != 0U) &&
      (AppSnapshotComputeAgeMs(tick_ms, snapshot->h3lis100dl.last_update_ms) <= APP_SENSOR_STALE_TIMEOUT_MS))
  {
    mask |= (1U << 1);
  }

  if ((snapshot->qma6100p.valid != 0U) &&
      (AppSnapshotComputeAgeMs(tick_ms, snapshot->qma6100p.last_update_ms) <= APP_SENSOR_STALE_TIMEOUT_MS))
  {
    mask |= (1U << 2);
  }

  return mask;
}

static uint32_t AppSnapshotComputeChangedMask(const AppSensorSnapshot_t *snapshot,
                                              const AppSensorSnapshot_t *prev_snapshot)
{
  uint32_t mask = 0U;

  if (snapshot->lsm6dsox.sample_seq != prev_snapshot->lsm6dsox.sample_seq)
  {
    mask |= (1U << 0);
  }

  if (snapshot->h3lis100dl.sample_seq != prev_snapshot->h3lis100dl.sample_seq)
  {
    mask |= (1U << 1);
  }

  if (snapshot->qma6100p.sample_seq != prev_snapshot->qma6100p.sample_seq)
  {
    mask |= (1U << 2);
  }

  return mask;
}

static uint32_t AppSnapshotComputeMaxDeltaMs(const AppSensorSnapshot_t *snapshot)
{
  uint32_t min_ts = 0U;
  uint32_t max_ts = 0U;
  uint8_t seen = 0U;

  if (snapshot->lsm6dsox.valid != 0U)
  {
    min_ts = snapshot->lsm6dsox.last_update_ms;
    max_ts = snapshot->lsm6dsox.last_update_ms;
    seen = 1U;
  }

  if (snapshot->h3lis100dl.valid != 0U)
  {
    if (seen == 0U)
    {
      min_ts = snapshot->h3lis100dl.last_update_ms;
      max_ts = snapshot->h3lis100dl.last_update_ms;
      seen = 1U;
    }
    else
    {
      if (snapshot->h3lis100dl.last_update_ms < min_ts)
      {
        min_ts = snapshot->h3lis100dl.last_update_ms;
      }
      if (snapshot->h3lis100dl.last_update_ms > max_ts)
      {
        max_ts = snapshot->h3lis100dl.last_update_ms;
      }
    }
  }

  if (snapshot->qma6100p.valid != 0U)
  {
    if (seen == 0U)
    {
      min_ts = snapshot->qma6100p.last_update_ms;
      max_ts = snapshot->qma6100p.last_update_ms;
      seen = 1U;
    }
    else
    {
      if (snapshot->qma6100p.last_update_ms < min_ts)
      {
        min_ts = snapshot->qma6100p.last_update_ms;
      }
      if (snapshot->qma6100p.last_update_ms > max_ts)
      {
        max_ts = snapshot->qma6100p.last_update_ms;
      }
    }
  }

  if (seen == 0U)
  {
    return 0U;
  }

  return max_ts - min_ts;
}

static uint32_t AppSnapshotComputeCoherent(const AppSensorSnapshot_t *snapshot, uint32_t fresh_mask)
{
  if (fresh_mask != 0x07U)
  {
    return 0U;
  }

  return (AppSnapshotComputeMaxDeltaMs(snapshot) <= APP_SENSOR_COHERENT_WINDOW_MS) ? 1U : 0U;
}

static void AppFlowStatsRecord(uint32_t tick_ms,
                               uint32_t row_seq,
                               uint32_t fresh_mask,
                               uint32_t changed_mask,
                               uint32_t coherent)
{
  if (snapshot_mutex == NULL)
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_flow_stats.logger_rows = row_seq;
  g_flow_stats.last_log_tick_ms = tick_ms;
  g_flow_stats.last_log_seq = row_seq;
  g_flow_stats.last_changed_mask = changed_mask;
  g_flow_stats.last_fresh_mask = fresh_mask;
  g_flow_stats.last_max_delta_ms = AppSnapshotComputeMaxDeltaMs(&g_sensor_snapshot);

  if (fresh_mask != 0x07U)
  {
    g_flow_stats.stale_rows++;
  }

  if (changed_mask != 0x07U)
  {
    g_flow_stats.mixed_rows++;
  }

  if (coherent != 0U)
  {
    g_flow_stats.coherent_rows++;
  }

  osMutexRelease(snapshot_mutex);
}

static uint8_t AppUsbRawStreamingActive(void)
{
  return UsbCdcService_IsReady();
}

static void AppFlowStatsSetMode(uint8_t usb_active, uint8_t sd_active)
{
  if (snapshot_mutex == NULL)
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_flow_stats.usb_streaming_active = usb_active;
  g_flow_stats.sd_logging_active = sd_active;
  osMutexRelease(snapshot_mutex);
}

static void AppFlowStatsRecordFrameWrite(const AppSensorFrame_t *frame)
{
  if ((snapshot_mutex == NULL) || (frame == NULL))
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_flow_stats.logger_rows++;
  g_flow_stats.sd_frames_written++;
  g_flow_stats.coherent_rows++;
  g_flow_stats.last_log_tick_ms = frame->tick_ms;
  g_flow_stats.last_log_seq = frame->frame_id;
  g_flow_stats.frame_id = frame->frame_id;
  g_flow_stats.last_fresh_mask = frame->present_mask;
  g_flow_stats.last_changed_mask = frame->present_mask;
  g_flow_stats.last_max_delta_ms = 0U;
  if (frame->present_mask != frame->enabled_mask)
  {
    g_flow_stats.stale_rows++;
    g_flow_stats.mixed_rows++;
  }
  osMutexRelease(snapshot_mutex);
}

static void AppFlowStatsRecordWriteFailure(void)
{
  if (snapshot_mutex == NULL)
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_flow_stats.logger_write_failures++;
  osMutexRelease(snapshot_mutex);
}

static void AppFlowStatsRecordUsbSent(uint32_t sample_seq)
{
  if (snapshot_mutex == NULL)
  {
    return;
  }

  osMutexAcquire(snapshot_mutex, osWaitForever);
  g_flow_stats.usb_frames_sent++;
  g_flow_stats.last_log_seq = sample_seq;
  osMutexRelease(snapshot_mutex);
}

static void UsbCmd_Ping(void)
{
  const char *msg = "pong\r\n";
  UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
}

static void UsbCmd_Help(void)
{
  const char *msg = "Commands: ping, help, status, snapshot, flowstat, dmastat, stat, acq_start, acq_stop, acq_status, msc\r\n";
  UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
}

static void UsbCmd_Status(void)
{
  char line[128];
  int len;
  uint32_t tick = osKernelGetTickCount();
  AppFlowStats_t stats;
  AppAcqControl_t acq;
  uint32_t elapsed_ms = 0U;
  uint32_t remaining_ms = 0U;

  AppAcqCheckAutoStop();

  if (snapshot_mutex != NULL)
  {
    osMutexAcquire(snapshot_mutex, osWaitForever);
    stats = g_flow_stats;
    osMutexRelease(snapshot_mutex);
  }
  else
  {
    memset(&stats, 0, sizeof(stats));
  }

  AppAcqGetCopy(&acq);
  if (acq.start_tick_ms != 0U)
  {
    uint32_t end_tick = (acq.running != 0U) ? tick : acq.last_stop_tick_ms;
    elapsed_ms = end_tick - acq.start_tick_ms;
  }
  if ((acq.running != 0U) && (acq.duration_ms > elapsed_ms))
  {
    remaining_ms = acq.duration_ms - elapsed_ms;
  }

#define USB_STATUS_LINE(fmt, ...) \
  do { \
    len = snprintf(line, sizeof(line), fmt, ##__VA_ARGS__); \
    if (len > 0 && (uint32_t)len < sizeof(line)) { \
      UsbCdcService_Write((const uint8_t *)line, (uint32_t)len); \
    } \
  } while (0)

  USB_STATUS_LINE("tick=%lu heap=%lu\r\n",
                  (unsigned long)tick,
                  (unsigned long)xPortGetFreeHeapSize());
  USB_STATUS_LINE("flow frames=%lu write_fail=%lu stale=%lu mixed=%lu coherent=%lu\r\n",
                  (unsigned long)stats.logger_rows,
                  (unsigned long)stats.logger_write_failures,
                  (unsigned long)stats.stale_rows,
                  (unsigned long)stats.mixed_rows,
                  (unsigned long)stats.coherent_rows);
  USB_STATUS_LINE("sensor updates lsm=%lu h3=%lu qma=%lu\r\n",
                  (unsigned long)stats.lsm_updates,
                  (unsigned long)stats.h3_updates,
                  (unsigned long)stats.qma_updates);
  USB_STATUS_LINE("frame io sd_written=%lu usb_sent=%lu last_frame=%lu\r\n",
                  (unsigned long)stats.sd_frames_written,
                  (unsigned long)stats.usb_frames_sent,
                  (unsigned long)stats.frame_id);
  USB_STATUS_LINE("frame queue depth=%lu dropped=%lu high=%lu\r\n",
                  (unsigned long)stats.frame_buffer_depth,
                  (unsigned long)stats.frame_dropped,
                  (unsigned long)stats.frame_high_watermark);
  USB_STATUS_LINE("mode usb=%u sd=%s\r\n",
                  (unsigned int)stats.usb_streaming_active,
                  (stats.sd_logging_active != 0U) ? "active" : "paused");
  USB_STATUS_LINE("acq state=%s sink=%s req_hz=%lu eff_period_ms=%lu eff_hz=%lu duration_ms=%lu elapsed_ms=%lu remaining_ms=%lu\r\n",
                  (acq.running != 0U) ? "running" : "stopped",
                  AppAcqSinkToString(acq.sink),
                  (unsigned long)acq.requested_hz,
                  (unsigned long)acq.period_ms,
                  (unsigned long)acq.effective_hz,
                  (unsigned long)acq.duration_ms,
                  (unsigned long)elapsed_ms,
                  (unsigned long)remaining_ms);
  USB_STATUS_LINE("cfg lsm=+-4g/833Hz gyro=+-2000dps/833Hz h3=+-100g/400Hz qma=+-4g/1600Hz\r\n");

#undef USB_STATUS_LINE
}

static void UsbCmd_Snapshot(void)
{
  AppSensorSnapshot_t snap;
  char line[160];
  int len;
  uint32_t tick_ms = osKernelGetTickCount();
  uint32_t fresh_mask;
  uint32_t max_delta_ms;

  AppSnapshotCopy(&snap);
  fresh_mask = AppSnapshotComputeFreshMask(&snap, tick_ms);
  max_delta_ms = AppSnapshotComputeMaxDeltaMs(&snap);

#define USB_SNAPSHOT_LINE(fmt, ...) \
  do { \
    len = snprintf(line, sizeof(line), fmt, ##__VA_ARGS__); \
    if (len > 0 && (uint32_t)len < sizeof(line)) { \
      UsbCdcService_Write((const uint8_t *)line, (uint32_t)len); \
    } \
  } while (0)

  USB_SNAPSHOT_LINE("SNAP fresh=0x%lX max_delta=%lums\r\n",
                    (unsigned long)fresh_mask,
                    (unsigned long)max_delta_ms);
  USB_SNAPSHOT_LINE("LSM v=%u seq=%lu age=%lums acc=[%.1f %.1f %.1f]\r\n",
                    (unsigned int)snap.lsm6dsox.valid,
                    (unsigned long)snap.lsm6dsox.sample_seq,
                    (unsigned long)AppSnapshotComputeAgeMs(tick_ms, snap.lsm6dsox.last_update_ms),
                    snap.lsm6dsox.data.acc.x,
                    snap.lsm6dsox.data.acc.y,
                    snap.lsm6dsox.data.acc.z);
  USB_SNAPSHOT_LINE("H3  v=%u seq=%lu age=%lums acc=[%.1f %.1f %.1f]\r\n",
                    (unsigned int)snap.h3lis100dl.valid,
                    (unsigned long)snap.h3lis100dl.sample_seq,
                    (unsigned long)AppSnapshotComputeAgeMs(tick_ms, snap.h3lis100dl.last_update_ms),
                    snap.h3lis100dl.data.acc_mg[0],
                    snap.h3lis100dl.data.acc_mg[1],
                    snap.h3lis100dl.data.acc_mg[2]);
  USB_SNAPSHOT_LINE("QMA v=%u seq=%lu age=%lums acc=[%.1f %.1f %.1f]\r\n",
                    (unsigned int)snap.qma6100p.valid,
                    (unsigned long)snap.qma6100p.sample_seq,
                    (unsigned long)AppSnapshotComputeAgeMs(tick_ms, snap.qma6100p.last_update_ms),
                    snap.qma6100p.data.acc_mg[0],
                    snap.qma6100p.data.acc_mg[1],
                    snap.qma6100p.data.acc_mg[2]);

#undef USB_SNAPSHOT_LINE
}

static void UsbCmd_FlowStat(void)
{
  char line[128];
  int len;
  AppFlowStats_t stats;
  AppAcqControl_t acq;
  uint32_t now_ms = osKernelGetTickCount();
  uint32_t elapsed_ms = 0U;
  uint32_t remaining_ms = 0U;

  AppAcqCheckAutoStop();

  if (snapshot_mutex != NULL)
  {
    osMutexAcquire(snapshot_mutex, osWaitForever);
    stats = g_flow_stats;
    osMutexRelease(snapshot_mutex);
  }
  else
  {
    memset(&stats, 0, sizeof(stats));
  }

  AppAcqGetCopy(&acq);
  if (acq.start_tick_ms != 0U)
  {
    uint32_t end_tick = (acq.running != 0U) ? now_ms : acq.last_stop_tick_ms;
    elapsed_ms = end_tick - acq.start_tick_ms;
  }
  if ((acq.running != 0U) && (acq.duration_ms > elapsed_ms))
  {
    remaining_ms = acq.duration_ms - elapsed_ms;
  }

#define USB_FLOW_LINE(fmt, ...) \
  do { \
    len = snprintf(line, sizeof(line), fmt, ##__VA_ARGS__); \
    if (len > 0 && (uint32_t)len < sizeof(line)) { \
      UsbCdcService_Write((const uint8_t *)line, (uint32_t)len); \
    } \
  } while (0)

  USB_FLOW_LINE("FLOW frames=%lu write_fail=%lu stale=%lu mixed=%lu coherent=%lu\r\n",
                (unsigned long)stats.logger_rows,
                (unsigned long)stats.logger_write_failures,
                (unsigned long)stats.stale_rows,
                (unsigned long)stats.mixed_rows,
                (unsigned long)stats.coherent_rows);
  USB_FLOW_LINE("FLOW sensor_updates lsm=%lu h3=%lu qma=%lu\r\n",
                (unsigned long)stats.lsm_updates,
                (unsigned long)stats.h3_updates,
                (unsigned long)stats.qma_updates);
  USB_FLOW_LINE("FLOW frame_io sd_written=%lu usb_sent=%lu last_frame=%lu\r\n",
                (unsigned long)stats.sd_frames_written,
                (unsigned long)stats.usb_frames_sent,
                (unsigned long)stats.frame_id);
  USB_FLOW_LINE("FLOW frame_queue depth=%lu dropped=%lu high=%lu\r\n",
                (unsigned long)stats.frame_buffer_depth,
                (unsigned long)stats.frame_dropped,
                (unsigned long)stats.frame_high_watermark);
  USB_FLOW_LINE("FLOW mode usb=%u sd=%s\r\n",
                (unsigned int)stats.usb_streaming_active,
                (stats.sd_logging_active != 0U) ? "active" : "paused");
  USB_FLOW_LINE("FLOW acq state=%s sink=%s req_hz=%lu eff_period_ms=%lu eff_hz=%lu duration_ms=%lu elapsed_ms=%lu remaining_ms=%lu\r\n",
                (acq.running != 0U) ? "running" : "stopped",
                AppAcqSinkToString(acq.sink),
                (unsigned long)acq.requested_hz,
                (unsigned long)acq.period_ms,
                (unsigned long)acq.effective_hz,
                (unsigned long)acq.duration_ms,
                (unsigned long)elapsed_ms,
                (unsigned long)remaining_ms);
  USB_FLOW_LINE("FLOW cfg lsm=+-4g/833Hz gyro=+-2000dps/833Hz h3=+-100g/400Hz qma=+-4g/1600Hz\r\n");

#undef USB_FLOW_LINE
}

static void UsbCmd_DmaStat(void)
{
  char line[128];
  int len;
  uint32_t lsm_calls = LSM6DSOX_GetDmaCallCount();
  uint32_t spi1_ok = DmaSampling_GetTransferCount();
  uint32_t spi1_err = DmaSampling_GetErrorCount();
  uint32_t spi1_start_fail = DmaSampling_GetStartFailCount();
  uint32_t spi1_timeout = DmaSampling_GetTimeoutCount();
  uint32_t spi1_rx_ok = DmaSampling_GetDmaRxCpltCount();
  uint32_t spi1_tx_ok = DmaSampling_GetDmaTxCpltCount();
  uint32_t spi1_spi_err = DmaSampling_GetSpiErrorCode();
  uint32_t spi2_ok = DmaSampling_GetSpi2TransferCount();
  uint32_t spi2_err = DmaSampling_GetSpi2ErrorCount();
  uint32_t spi2_start_fail = DmaSampling_GetSpi2StartFailCount();
  uint32_t spi2_timeout = DmaSampling_GetSpi2TimeoutCount();
  uint32_t spi2_rx_ok = DmaSampling_GetSpi2DmaRxCpltCount();
  uint32_t spi2_tx_ok = DmaSampling_GetSpi2DmaTxCpltCount();
  uint32_t spi2_spi_err = DmaSampling_GetSpi2ErrorCode();
  uint32_t ch0_csr = 0U, ch1_csr = 0U, ch2_csr = 0U, ch3_csr = 0U;

  DmaSampling_GetChannelStatus(&ch0_csr, &ch1_csr);
  DmaSampling_GetSpi2ChannelStatus(&ch2_csr, &ch3_csr);

#define USB_WRITE_LINE(fmt, ...) \
  do { \
    len = snprintf(line, sizeof(line), fmt, ##__VA_ARGS__); \
    if (len > 0 && (uint32_t)len < sizeof(line)) { \
      UsbCdcService_Write((const uint8_t *)line, (uint32_t)len); \
    } \
  } while (0)

  USB_WRITE_LINE("[DS] LSM calls=%lu\r\n", (unsigned long)lsm_calls);
  USB_WRITE_LINE("[DS] SPI1 ok=%lu err=%lu fail=%lu to=%lu rx=%lu tx=%lu serr=0x%lX\r\n",
                 (unsigned long)spi1_ok, (unsigned long)spi1_err,
                 (unsigned long)spi1_start_fail, (unsigned long)spi1_timeout,
                 (unsigned long)spi1_rx_ok, (unsigned long)spi1_tx_ok,
                 (unsigned long)spi1_spi_err);
  USB_WRITE_LINE("[DS] SPI1 ch0=0x%lX ch1=0x%lX\r\n",
                 (unsigned long)ch0_csr, (unsigned long)ch1_csr);
  USB_WRITE_LINE("[DS] SPI2 ok=%lu err=%lu fail=%lu to=%lu rx=%lu tx=%lu serr=0x%lX\r\n",
                 (unsigned long)spi2_ok, (unsigned long)spi2_err,
                 (unsigned long)spi2_start_fail, (unsigned long)spi2_timeout,
                 (unsigned long)spi2_rx_ok, (unsigned long)spi2_tx_ok,
                 (unsigned long)spi2_spi_err);
  USB_WRITE_LINE("[DS] SPI2 ch2=0x%lX ch3=0x%lX\r\n",
                 (unsigned long)ch2_csr, (unsigned long)ch3_csr);

#undef USB_WRITE_LINE
}

static void UsbCmd_Process(const char *cmd)
{
  if (strcmp(cmd, "ping") == 0)
  {
    UsbCmd_Ping();
  }
  else if (strcmp(cmd, "help") == 0)
  {
    UsbCmd_Help();
  }
  else if (strcmp(cmd, "status") == 0)
  {
    UsbCmd_Status();
  }
  else if (strcmp(cmd, "snapshot") == 0)
  {
    UsbCmd_Snapshot();
  }
  else if (strcmp(cmd, "flowstat") == 0)
  {
    UsbCmd_FlowStat();
  }
  else if (strcmp(cmd, "dmastat") == 0)
  {
    UsbCmd_DmaStat();
  }
  else if (strcmp(cmd, "stat") == 0)
  {
    UsbCmd_DmaStat();
  }
  else if (strncmp(cmd, "acq_start ", 10U) == 0)
  {
    UsbCmd_AcqStart(cmd);
  }
  else if (strcmp(cmd, "acq_stop") == 0)
  {
    UsbCmd_AcqStop();
  }
  else if (strcmp(cmd, "acq_status") == 0)
  {
    UsbCmd_AcqStatus();
  }
  else if (strcmp(cmd, "msc") == 0)
  {
    const char *msg = "Switching to USB MSC mode...\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
    UsbCmd_AcqStop();
    osDelay(500U);
    BootMode_Write(BOOT_MODE_USB_MSC);
    /* Verify write before reset */
    boot_mode_t verify = BootMode_Read();
    char buf[64];
    snprintf(buf, sizeof(buf), "BootMode flag written, readback=%d, resetting...\r\n", (int)verify);
    UsbCdcService_Write((const uint8_t *)buf, strlen(buf));
    osDelay(200U);
    NVIC_SystemReset();
  }
  else
  {
    const char *msg = "Unknown cmd\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
  }
}


void StartUsbCdcTask(void *argument)
{
  uint8_t usb_init_ok;
  char tx_buffer[USB_CDC_TX_BUFFER_SIZE];
  uint8_t rx_buffer[USB_CDC_RX_BUFFER_SIZE];
  char cmd_buffer[USB_CDC_CMD_BUFFER_SIZE];
  uint32_t cmd_len = 0U;

  (void)argument;

  memset(tx_buffer, 0, sizeof(tx_buffer));
  memset(rx_buffer, 0, sizeof(rx_buffer));
  memset(cmd_buffer, 0, sizeof(cmd_buffer));

  usb_init_ok = UsbCdcService_Init();
  printf("[CDC] init %s\r\n", usb_init_ok ? "ok" : "FAIL");

  for (;;)
  {
    UsbCdcService_Poll();

    if (usb_init_ok != 0U && UsbCdcService_IsReady() != 0U)
    {
      /* 读取接收数据 */
      uint32_t rx_len = UsbCdcService_Read(rx_buffer, sizeof(rx_buffer));

      for (uint32_t i = 0U; i < rx_len; i++)
      {
        char ch = (char)rx_buffer[i];

        if (ch == '\r' || ch == '\n')
        {
          if (cmd_len > 0U)
          {
            cmd_buffer[cmd_len] = '\0';
            printf("[CDC] cmd: %s\r\n", cmd_buffer);
            UsbCmd_Process(cmd_buffer);
            cmd_len = 0U;
          }
        }
        else if (cmd_len < (USB_CDC_CMD_BUFFER_SIZE - 1U))
        {
          cmd_buffer[cmd_len++] = ch;
        }
      }
    }

    osDelay(USB_CDC_POLL_DELAY_MS);
  }
}

void StartUsbUploadTask(void *argument)
{
  AppSensorFrame_t frame;
  uint8_t header_sent = 0U;

  (void)argument;

  printf("[UsbUpload] task started\r\n");

  for (;;)
  {
    uint8_t usb_sink_active;

    AppAcqCheckAutoStop();
    usb_sink_active = AppAcqIsUsbSinkActive();

    if (usb_sink_active != 0U)
    {
      AppFlowStatsSetMode(1U, 0U);

      if (header_sent == 0U)
      {
        const char *header =
            "frame_id,tick_ms,enabled_mask,present_mask,"
            "lsm_sample_seq,lsm_valid,lsm_acc_x_mg,lsm_acc_y_mg,lsm_acc_z_mg,lsm_gyro_x_mdps,lsm_gyro_y_mdps,lsm_gyro_z_mdps,lsm_temp_c,"
            "h3_sample_seq,h3_valid,h3_raw_x,h3_raw_y,h3_raw_z,h3_acc_x_mg,h3_acc_y_mg,h3_acc_z_mg,"
            "qma_sample_seq,qma_valid,qma_raw_x,qma_raw_y,qma_raw_z,qma_acc_x_mg,qma_acc_y_mg,qma_acc_z_mg\r\n";
        UsbCdcService_Write((const uint8_t *)header, strlen(header));
        header_sent = 1U;
        printf("[UsbUpload] frame CSV header sent\r\n");
      }

      while ((AppAcqIsUsbSinkActive() != 0U) && (AppFrameBufferPop(&frame) != 0U))
      {
        char csv_line[512];
        int len = snprintf(
            csv_line,
            sizeof(csv_line),
            "%lu,%lu,0x%02lX,0x%02lX,"
            "%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
            "%lu,%u,%d,%d,%d,%.1f,%.1f,%.1f,"
            "%lu,%u,%d,%d,%d,%.1f,%.1f,%.1f\r\n",
            (unsigned long)frame.frame_id,
            (unsigned long)frame.tick_ms,
            (unsigned long)frame.enabled_mask,
            (unsigned long)frame.present_mask,
            (unsigned long)frame.lsm6dsox.sample_seq,
            (unsigned int)frame.lsm6dsox.valid,
            frame.lsm6dsox.data.acc.x,
            frame.lsm6dsox.data.acc.y,
            frame.lsm6dsox.data.acc.z,
            frame.lsm6dsox.data.gyro.x,
            frame.lsm6dsox.data.gyro.y,
            frame.lsm6dsox.data.gyro.z,
            frame.lsm6dsox.data.temp_C,
            (unsigned long)frame.h3lis100dl.sample_seq,
            (unsigned int)frame.h3lis100dl.valid,
            (int)frame.h3lis100dl.data.raw[0],
            (int)frame.h3lis100dl.data.raw[1],
            (int)frame.h3lis100dl.data.raw[2],
            frame.h3lis100dl.data.acc_mg[0],
            frame.h3lis100dl.data.acc_mg[1],
            frame.h3lis100dl.data.acc_mg[2],
            (unsigned long)frame.qma6100p.sample_seq,
            (unsigned int)frame.qma6100p.valid,
            (int)frame.qma6100p.data.raw[0],
            (int)frame.qma6100p.data.raw[1],
            (int)frame.qma6100p.data.raw[2],
            frame.qma6100p.data.acc_mg[0],
            frame.qma6100p.data.acc_mg[1],
            frame.qma6100p.data.acc_mg[2]);
        if ((len > 0) && ((uint32_t)len < sizeof(csv_line)))
        {
          UsbCdcService_Write((const uint8_t *)csv_line, (uint32_t)len);
          AppFlowStatsRecordUsbSent(frame.frame_id);
        }
      }
    }
    else
    {
      AppFlowStatsSetMode(0U, AppAcqIsSdSinkActive());
      if (header_sent != 0U)
      {
        header_sent = 0U;
        printf("[UsbUpload] USB frame stream idle, will resend header next session\r\n");
      }
    }

    osDelay(USB_UPLOAD_PERIOD_MS);
  }
}

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

  AppSnapshotReset();

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_LSM6DSOX
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
    printf("[LSM6DSOX] init failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }

  /* Configure LSM FIFO: ACC+GYRO batched at 6664 Hz (max ODR), watermark
   * at 256 samples (~19ms latency: 256/(6664*2)=19.2ms). INT1 (PB1)
   * triggers EXTI when FIFO fill reaches the watermark. */
  if (LSM6DSOX_FIFO_Config(256U, LSM6DSOX_BDR_6667Hz, LSM6DSOX_BDR_6667Hz) != HAL_OK)
  {
    printf("[LSM6DSOX] FIFO config failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }

  /* Local FIFO read scratch: 256 words × 7 bytes */
  static uint8_t fifo_buf[256U * 7U];

  for (;;)
  {
    AppAcqCheckAutoStop();
    if (AppAcqIsRunning() == 0U)
    {
      osDelay(APP_ACQ_IDLE_DELAY_MS);
      continue;
    }

    /* Block on FIFO watermark interrupt (released by EXTI1 ISR on PB1).
     * Use 100ms timeout so we still poll the FIFO level periodically — this
     * also covers any interrupt we might miss during heavy SPI bus traffic. */
    if (s_lsm_fifo_sem == NULL ||
        osSemaphoreAcquire(s_lsm_fifo_sem, 100U) != osOK)
    {
      /* Timeout: still check FIFO — sensor may have produced data without us
       * catching the rising edge. */
    }

    /* Drain FIFO completely — keep reading 256-word chunks until below wtm.
     * Each chunk is parsed into a single AppLsmBatch_t (up to 128 sample
     * pairs) and pushed to the batch ring buffer for the logger to consume. */
    uint16_t fifo_level = 0;
    while (1)
    {
      if (LSM6DSOX_FIFO_GetLevel(&fifo_level) != HAL_OK || fifo_level == 0U)
      {
        break;
      }
      uint16_t to_read = (fifo_level > 256U) ? 256U : fifo_level;

      if (LSM6DSOX_FIFO_ReadBlock(fifo_buf, to_read) != HAL_OK)
      {
        break;
      }

      /* Parse FIFO words into a batch. ACC and GYR samples are interleaved
       * at the same BDR; we accumulate them as pairs. */
      AppLsmBatch_t batch;
      memset(&batch, 0, sizeof(batch));
      batch.base_tick_ms = osKernelGetTickCount();
      batch.period_us = 150U;  /* 1/6664Hz ≈ 150us */
      batch.acc_sensitivity  = LSM6DSOX_GetAccSensitivity();
      batch.gyro_sensitivity = LSM6DSOX_GetGyroSensitivity();

      uint8_t cur_has_acc = 0, cur_has_gyr = 0;
      int16_t cur_acc[3] = {0}, cur_gyr[3] = {0};

      for (uint16_t i = 0; i < to_read; i++)
      {
        uint8_t *w = &fifo_buf[i * 7U];
        uint8_t tag_id = (uint8_t)(w[0] >> 3);
        int16_t rx = (int16_t)((uint16_t)w[2] << 8 | w[1]);
        int16_t ry = (int16_t)((uint16_t)w[4] << 8 | w[3]);
        int16_t rz = (int16_t)((uint16_t)w[6] << 8 | w[5]);

        if (tag_id == LSM6DSOX_TAG_ACC_NC)
        {
          cur_acc[0] = rx; cur_acc[1] = ry; cur_acc[2] = rz;
          cur_has_acc = 1;
        }
        else if (tag_id == LSM6DSOX_TAG_GYRO_NC)
        {
          cur_gyr[0] = rx; cur_gyr[1] = ry; cur_gyr[2] = rz;
          cur_has_gyr = 1;
        }
        else continue;

        if (cur_has_acc && cur_has_gyr)
        {
          if (batch.n_pairs < APP_LSM_BATCH_MAX_PAIRS)
          {
            batch.acc[batch.n_pairs][0] = cur_acc[0];
            batch.acc[batch.n_pairs][1] = cur_acc[1];
            batch.acc[batch.n_pairs][2] = cur_acc[2];
            batch.gyro[batch.n_pairs][0] = cur_gyr[0];
            batch.gyro[batch.n_pairs][1] = cur_gyr[1];
            batch.gyro[batch.n_pairs][2] = cur_gyr[2];
            batch.n_pairs++;
          }
          cur_has_acc = 0;
          cur_has_gyr = 0;
        }
      }

      if (batch.n_pairs > 0U)
      {
        if (snapshot_mutex != NULL)
        {
          osMutexAcquire(snapshot_mutex, osWaitForever);
          g_flow_stats.frame_id += batch.n_pairs;
          batch.base_frame_id = g_flow_stats.frame_id;
          osMutexRelease(snapshot_mutex);
        }
        (void)AppLsmBatchPush(&batch);
      }

      /* Stop once FIFO is back below the high-water mark; otherwise continue
       * reading the next chunk in this same iteration. */
      if (to_read < 256U) break;
    }

    /* Once per FIFO drain, also sample H3/QMA so they keep flowing */
    {
      AppSensorFrame_t frame;
      int h3_ret;
      HAL_StatusTypeDef qma_ret;
      uint32_t now_ms = osKernelGetTickCount();

      memset(&frame, 0, sizeof(frame));
      frame.tick_ms = now_ms;
      frame.enabled_mask = APP_SENSOR_MASK_ALL;

      if (snapshot_mutex != NULL)
      {
        osMutexAcquire(snapshot_mutex, osWaitForever);
        frame.frame_id = ++g_flow_stats.frame_id;
        osMutexRelease(snapshot_mutex);
      }

      osMutexAcquire(spi2_mutex, osWaitForever);
      h3_ret = H3LIS100DL_ReadAccXYZ(&frame.h3lis100dl.data);
      if (h3_ret == 0)
      {
        AppFramePopulateH3lis100dl(&frame, &frame.h3lis100dl.data, now_ms);
      }
      qma_ret = QMA6100P_ReadAccXYZ(&frame.qma6100p.data);
      if (qma_ret == HAL_OK)
      {
        AppFramePopulateQma6100p(&frame, &frame.qma6100p.data, now_ms);
      }
      osMutexRelease(spi2_mutex);

      if (frame.present_mask != 0U)
      {
        (void)AppFrameBufferPush(&frame);
      }
    }
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
    printf("[H3LIS100DL] init failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }
  osMutexRelease(spi2_mutex);

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL
  printf("[H3LIS100DL TEST] started, print every %lu ms\r\n", (unsigned long)SAMPLE_PERIOD_MS);
#endif

  for (;;)
  {
#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL
    int ret;
    uint32_t delay_ms = AppAcqCurrentPeriodMs();

    AppAcqCheckAutoStop();
    if (AppAcqIsRunning() == 0U)
    {
      osDelay(APP_ACQ_IDLE_DELAY_MS);
      continue;
    }

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
      printf("[H3LIS100DL TEST] read failed (ret=%d)\r\n", ret);
    }

    osDelay(delay_ms);
#else
    osDelay(APP_ACQ_IDLE_DELAY_MS);
#endif
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
    printf("[QMA6100P] init failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }
  osMutexRelease(spi2_mutex);

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P
  printf("[QMA6100P TEST] started, print every %lu ms\r\n", (unsigned long)SAMPLE_PERIOD_MS);
#endif

  for (;;)
  {
#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P
    HAL_StatusTypeDef ret;
    uint32_t delay_ms = AppAcqCurrentPeriodMs();

    AppAcqCheckAutoStop();
    if (AppAcqIsRunning() == 0U)
    {
      osDelay(APP_ACQ_IDLE_DELAY_MS);
      continue;
    }

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
      printf("[QMA6100P TEST] read failed\r\n");
    }

    osDelay(delay_ms);
#else
    osDelay(APP_ACQ_IDLE_DELAY_MS);
#endif
  }
}

void StartLoggerTask(void *argument)
{
  FRESULT result;
  uint32_t rows_since_sync = 0U;
  uint8_t sd_file_open = 0U;
  AppSensorFrame_t frame;

  (void)argument;

  AppFlowStatsSetMode(0U, 0U);
  printf("[Logger] task started, SD card mode\r\n");

  /* 从SD卡读取 DeviceConfig.json 并应用到运行时配置；
   * 文件不存在时写入默认模板，两种情况均不中断启动流程 */
  printf("[Logger] loading config from SD...\r\n");
  (void)DeviceCfg_LoadFromSD();
  printf("[Logger] config loaded, entering main loop\r\n");

  for (;;)
  {
    AppAcqControl_t acq;
    uint8_t sd_session_active;
    uint8_t acq_running;

    AppAcqCheckAutoStop();
    AppAcqGetCopy(&acq);
    acq_running = acq.running;
    sd_session_active = (uint8_t)((acq_running != 0U) && (acq.sink == APP_ACQ_SINK_SD));

    if (sd_session_active == 0U)
    {
      AppFlowStatsSetMode(AppAcqIsUsbSinkActive(), 0U);
      AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
      osDelay(10U);
      continue;
    }

    if (sd_file_open == 0U)
    {
      printf("[Logger] starting SD session (acq running=%u sink=%u)...\r\n",
             (unsigned int)acq.running, (unsigned int)acq.sink);
      result = FatFs_SD_LoggerStart();
      if (result == FR_OK)
      {
        /* 将当前配置快照写入会话目录 */
        (void)DeviceCfg_WriteConfigToDir(FatFs_SD_GetSessionDir());

        sd_file_open = 1U;
        rows_since_sync = 0U;
        AppFlowStatsSetMode(0U, 1U);
        printf("[Logger] SD logging resumed\r\n");
      }
      else
      {
        printf("[Logger] SD start failed: %s (%d)\r\n",
               FatFs_SD_ResultToString(result),
               (int)result);
        AppFlowStatsSetMode(0U, 0U);
        osDelay(LOGGER_RETRY_DELAY_MS);
        continue;
      }
    }

    /* Drain LSM batch buffer first — these are bulk-write batches that contain
     * up to 128 sample pairs each and should be processed before the per-sample
     * H3/QMA frame buffer to keep up with high-rate FIFO output. */
    {
      AppLsmBatch_t batch;
      while ((sd_file_open != 0U) && (AppAcqIsSdSessionActive() != 0U) && (AppLsmBatchPop(&batch) != 0U))
      {
        result = FatFs_SD_LoggerAppendLsmBatch(&batch);
        if (result != FR_OK)
        {
          printf("[Logger] batch fail err=%s (%d)\r\n",
                 FatFs_SD_ResultToString(result), (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        rows_since_sync += batch.n_pairs;
      }
    }

    while ((sd_file_open != 0U) && (AppAcqIsSdSessionActive() != 0U) && (AppFrameBufferPop(&frame) != 0U))
    {
      result = FatFs_SD_LoggerAppendFrame(&frame);
      if (result != FR_OK)
      {
        printf("[Logger] append fail frame=%lu err=%s (%d)\r\n",
               (unsigned long)frame.frame_id,
               FatFs_SD_ResultToString(result),
               (int)result);
        AppFlowStatsRecordWriteFailure();
        AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
        AppFlowStatsSetMode(0U, 0U);
        osDelay(LOGGER_RETRY_DELAY_MS);
        break;
      }
      rows_since_sync++;
      AppFlowStatsRecordFrameWrite(&frame);
    }

    if ((sd_file_open != 0U) && (AppAcqDrainPendingStop() != 0U))
    {
      while (AppFrameBufferPop(&frame) != 0U)
      {
        result = FatFs_SD_LoggerAppendFrame(&frame);
        if (result != FR_OK)
        {
          printf("[Logger] drain fail frame=%lu err=%s (%d)\r\n",
                 (unsigned long)frame.frame_id,
                 FatFs_SD_ResultToString(result),
                 (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        rows_since_sync++;
        AppFlowStatsRecordFrameWrite(&frame);
      }

      result = FatFs_SD_LoggerSync();
      if (result != FR_OK)
      {
        printf("[Logger] final sync fail rows=%lu err=%s (%d)\r\n",
               (unsigned long)rows_since_sync,
               FatFs_SD_ResultToString(result),
               (int)result);
        AppFlowStatsRecordWriteFailure();
      }

      AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
      AppFlowStatsSetMode(AppAcqIsUsbSinkActive(), 0U);
      printf("[Logger] SD logging paused\r\n");
      osDelay(10U);
      continue;
    }

    if ((sd_file_open != 0U) && (AppAcqIsSdSessionActive() == 0U))
    {
      AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
      AppFlowStatsSetMode(AppAcqIsUsbSinkActive(), 0U);
      printf("[Logger] SD logging paused\r\n");
      osDelay(10U);
      continue;
    }

    if ((sd_file_open != 0U) && (rows_since_sync >= LOGGER_SYNC_EVERY_ROWS))
    {
      result = FatFs_SD_LoggerSync();
      if (result != FR_OK)
      {
        printf("[Logger] sync fail rows=%lu err=%s (%d)\r\n",
               (unsigned long)rows_since_sync,
               FatFs_SD_ResultToString(result),
               (int)result);
        AppFlowStatsRecordWriteFailure();
        AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
        AppFlowStatsSetMode(0U, 0U);
        osDelay(LOGGER_RETRY_DELAY_MS);
        rows_since_sync = 0U;
        continue;
      }
      rows_since_sync = 0U;
    }

    AppFlowStatsSetMode(0U, (uint8_t)(sd_file_open != 0U));
    osDelay(10U);
  }
}
/* USER CODE END Application */
