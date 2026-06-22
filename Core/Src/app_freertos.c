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
#include "semphr.h"
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
#include "aht20.h"
#include "lis2mdl.h"
#include "acq_config.h"
#include "device_config.h"
#include "boot_mode.h"
#include "sd_diskio.h"
#include "sdmmc.h"
#include "usbd_wcid_app.h"
#include "usbd_wcid_streaming.h"
#include "usbd_core.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usb_otg.h"
#include "app_acq.h"
#include "user_ctrl.h"
#include "board_io.h"
#include "app_time.h"
#include "rtc_pcf85063.h"
#include "i2c.h"
#include "mic_capture.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_PERIOD_MS         APP_SENSOR_SAMPLE_PERIOD_MS
#define LOGGER_RETRY_DELAY_MS    1000U

/* Route command responses to USB EP4 IN via UsbWcidApp_Write. */
#define UsbCdcService_Write(buf, len)  UsbWcidApp_Write((buf), (len))
#define UsbCdcService_IsReady()        (1U)
#define USB_UPLOAD_PERIOD_MS     5U
#define APP_ACQ_IDLE_DELAY_MS    10U
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
static volatile uint8_t s_usb_done_armed;
static osMutexId_t frame_buffer_mutex;
static osMutexId_t acq_ctrl_mutex;
SemaphoreHandle_t s_sdmmc_dma_sem;  /* signaled by HAL SD DMA completion ISR */
static osSemaphoreId_t s_lsm_fifo_sem;  /* released by EXTI0 ISR on PB0 rising edge (HW-v2) */
static osSemaphoreId_t s_qma_fifo_sem;  /* released by EXTI4 ISR on PC4 rising edge (HW-v2) */
static osSemaphoreId_t s_h3_drdy_sem;   /* released by EXTI1 ISR on PA1 rising edge (HW-v2) */
static osSemaphoreId_t s_mag_drdy_sem;  /* released by EXTI13 ISR on PC13 rising edge (LIS2MDL DRDY) */
static osMutexId_t     i2c1_mutex;      /* AHT20 + LIS2MDL 共享 I2C1 互斥 */
static AppSensorSnapshot_t g_sensor_snapshot;

typedef struct
{
  uint8_t running;
  uint8_t sink;
  uint8_t stop_pending;
  uint8_t timer_armed;   /* 0=计时未开始，1=计时中；SD session 由 logger 在 SD 就绪后 arm */
  uint32_t requested_hz;
  uint32_t period_ms;
  uint32_t effective_hz;
  uint32_t duration_ms;
  uint32_t start_tick_ms;
  uint32_t stop_tick_ms;
  uint32_t last_stop_tick_ms;
  uint64_t start_us;     /* auto-stop 计时基准用 apptime（真实时钟）：RTOS tick 比真实
                          * 时间慢 ~19%（44000 tick=52.5s 实测），不能用 tick 计采集时长 */
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
static AppSensorFrame_t g_composite_frame;  /* shared composite for USB upload */

/* Static data backing for the SPSC ring buffers (.bss, no heap pressure). */
static uint8_t s_lsm_imu_ringbuf[APP_RING_LSM_IMU_SIZE];
static uint8_t s_qma_acc_ringbuf[APP_RING_QMA_ACC_SIZE];
static uint8_t s_h3_acc_ringbuf[APP_RING_H3_ACC_SIZE];
static uint8_t s_aht_env_ringbuf[APP_RING_AHT_ENV_SIZE];
static uint8_t s_mag_ringbuf[APP_RING_MAG_SIZE];
static AppRingBuffer_t g_ring_lsm_imu;
static AppRingBuffer_t g_ring_qma_acc;
static AppRingBuffer_t g_ring_h3_acc;
static AppRingBuffer_t g_ring_aht_env;
static AppRingBuffer_t g_ring_mag;
static uint8_t s_mic_ringbuf[APP_RING_MIC_SIZE];
AppRingBuffer_t g_ring_mic;                     /* 非 static：mic_capture.c extern 引用 */
static volatile uint32_t g_lsm_frame_id_counter;
static volatile uint32_t g_qma_frame_id_counter;
static volatile uint32_t g_h3_frame_id_counter;
static volatile uint32_t g_aht_frame_id_counter;
static volatile uint32_t g_mag_frame_id_counter;
/* H3 DRDY 中断测试统计：中断触发 vs 超时(中断没来)次数，判断 PA1/EXTI1 DRDY 是否可靠。 */
static volatile uint32_t g_h3_irq_count;
static volatile uint32_t g_h3_timeout_count;
static uint64_t s_session_start_us;   /* AppTime µs at SD-session start (route-2 real-ODR diag) */
/* QMA FIFO over-read dedup: last emitted raw sample, to drop byte-identical stale repeats. */
static int16_t s_qma_prev_x, s_qma_prev_y, s_qma_prev_z;
static uint8_t s_qma_prev_valid;
static uint32_t s_qma_odr_interval_us = 625U;  /* 1/1600Hz, updated by QMA task init */
static uint32_t s_lsm_odr_interval_us = 150U;  /* 1/6667Hz, updated by LSM task init */
static uint32_t s_h3_odr_interval_us  = 2500U; /* 1/400Hz, updated by H3 task init */
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
static const osMutexAttr_t i2c1_mutex_attr = {
  .name      = "i2c1Mutex",
  .attr_bits = osMutexPrioInherit,
};

/* USER CODE END Variables */

/* ======================== Sensor threads ================================== */
osThreadId_t lsm6dsoxTaskHandle;
const osThreadAttr_t lsm6dsoxTask_attributes = {
  .name = "lsm6dsoxTask",
  /* High（> logger 的 AboveNormal）：6664Hz 硬件 FIFO 仅 ~38ms 余量，必须能抢占
   * logger 排空 FIFO，否则 FIFO 层丢 ~2.5%。早期单独提优先级失败(ring 溢出)是因
   * 当时 ring 小;现在 192KB ring + 攒批写、ring 峰值才 19%，足以兜住 logger 被
   * 抢占后的 drain 延迟。LSM 仅 ~7% CPU，不会饿死 logger。 */
  .priority = (osPriority_t)osPriorityHigh,
  .stack_size = 2048 * 4  /* 8KB — holds fifo_buf (1.8KB) + locals */
};

osThreadId_t qma6100pTaskHandle;
const osThreadAttr_t qma6100pTask_attributes = {
  .name = "qma6100pTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 2048 * 4  /* 8KB — fifo_buf(static)+rowbuf+局部变量；4KB曾导致栈溢出破坏frame_id计数器 */
};

osThreadId_t h3lis100dlTaskHandle;
const osThreadAttr_t h3lis100dlTask_attributes = {
  .name = "h3lis100dlTask",
  .priority = (osPriority_t)osPriorityHigh,   /* 提优先级：无FIFO+不锁存DRDY，满速时被LSM抢占会丢边沿；与LSM同High抢SPI2读 */
  .stack_size = 2048 * 4  /* 8KB — 与QMA同步扩容，防止低概率栈溢出 */
};

osThreadId_t loggerTaskHandle;
const osThreadAttr_t loggerTask_attributes = {
  .name = "loggerTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 1024 * 4  /* 4KB — logger only buffers small line[128] + locals */
};

osThreadId_t micTaskHandle;
const osThreadAttr_t micTask_attributes = {
  .name = "micTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 1024 * 1  /* 1KB — codec/SAI start-stop supervisor + small locals */
};

osThreadId_t aht20TaskHandle;
const osThreadAttr_t aht20Task_attributes = {
  .name = "aht20Task",
  .priority = (osPriority_t)osPriorityLow,    /* 1Hz 慢传感器 */
  .stack_size = 1024 * 2  /* 2KB */
};

osThreadId_t lis2mdlTaskHandle;
const osThreadAttr_t lis2mdlTask_attributes = {
  .name = "lis2mdlTask",
  .priority = (osPriority_t)osPriorityAboveNormal,  /* 100Hz DRDY 驱动 */
  .stack_size = 1024 * 2  /* 2KB */
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
static void     RingBuf_Init(AppRingBuffer_t *rb, uint8_t *data, uint32_t size);
static void     RingBuf_Reset(AppRingBuffer_t *rb);
static uint32_t RingBuf_Available(const AppRingBuffer_t *rb);
static uint32_t RingBuf_Write(AppRingBuffer_t *rb, const uint8_t *src, uint32_t len);
static uint32_t RingBuf_PeekContiguous(AppRingBuffer_t *rb, const uint8_t **out_ptr, uint32_t *out_len);
static void     RingBuf_Consume(AppRingBuffer_t *rb, uint32_t len);
static int      LoggerDrainRing(AppRingBuffer_t *rb, uint8_t file_idx, uint32_t min_flush,
                                uint32_t *rows_since_sync, FRESULT *out_res);
static inline uint32_t AppU32ToDec(char *out, uint32_t v);
static inline uint32_t AppI32ToDec(char *out, int32_t v);
static inline uint32_t AppU64ToDec(char *out, uint64_t v);
static void AppFramePopulateLsm6dsox(AppSensorFrame_t *frame, const LSM6DSOX_AllData_t *data, uint32_t tick_ms);
static void AppFramePopulateH3lis100dl(AppSensorFrame_t *frame, const H3LIS100DL_Data_t *data, uint32_t tick_ms);
static void AppFramePopulateQma6100p(AppSensorFrame_t *frame, const QMA6100P_Data_t *data, uint32_t tick_ms);
static QMA6100P_Bandwidth_t AppQmaBwToEnum(uint32_t odr_hz);
static float AppLsmXlSensitivity(uint16_t range_g);
static float AppLsmGyrSensitivity(uint16_t range_dps);
static const char *AppAcqSinkToString(uint8_t sink);
static uint8_t AppAcqParseSink(const char *text, uint8_t *sink_out);
static void AppAcqGetCopy(AppAcqControl_t *ctrl);
uint32_t AppAcqIsRunning(void);
static uint32_t AppAcqCurrentPeriodMs(void);
static uint8_t AppAcqIsUsbSinkActive(void);
static uint8_t AppAcqIsSdSessionActive(void);
static uint8_t AppUsbRawStreamingActive(void);
static void AppFlowStatsSetMode(uint8_t usb_active, uint8_t sd_active);
static void AppLoggerStopSdSession(uint8_t *sd_file_open, uint32_t *rows_since_sync);
static void AppAcqStopInternal(uint32_t now_ms);
static void AppAcqCheckAutoStop(void);
static void AppAcqResetSessionTimer(void);
uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms);
uint32_t AppAcqStop(void);
static uint32_t AppAcqDrainPendingStop(void);
static void UsbCmd_AcqStatus(void);
static void UsbCmd_AcqStart(const char *cmd);
static void UsbCmd_AcqStop(void);


void StartLsm6dsoxTask(void *argument);
void StartQma6100pTask(void *argument);
void StartH3lis100dlTask(void *argument);
void StartLoggerTask(void *argument);
void StartMicTask(void *argument);
void StartAht20Task(void *argument);
void StartLis2mdlTask(void *argument);
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
  i2c1_mutex     = osMutexNew(&i2c1_mutex_attr);
  UsbWcidApp_InitRtos();   /* 串行化 0x85 响应端点（命令响应 + AHT + MAG 共用） */
#if (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_NONE) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL) || \
    (APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P)
  spi2_mutex = osMutexNew(&spi2_mutex_attr);
#endif
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  s_lsm_fifo_sem = osSemaphoreNew(1, 0, NULL);  /* binary semaphore, init=0 */
  s_qma_fifo_sem = osSemaphoreNew(1, 0, NULL);
  s_h3_drdy_sem  = osSemaphoreNew(1, 0, NULL);
  s_mag_drdy_sem = osSemaphoreNew(1, 0, NULL);
  s_sdmmc_dma_sem = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* Load device config from SD card (DEVCFG.JSN) before tasks start,
   * so sensor tasks read the correct ODR/range/enabled values. */
  (void)DeviceCfg_LoadFromSD();

  /* Initialise SPSC ring buffers (data arrays are static, no allocation). */
  RingBuf_Init(&g_ring_lsm_imu, s_lsm_imu_ringbuf, APP_RING_LSM_IMU_SIZE);
  RingBuf_Init(&g_ring_qma_acc, s_qma_acc_ringbuf, APP_RING_QMA_ACC_SIZE);
  RingBuf_Init(&g_ring_h3_acc,  s_h3_acc_ringbuf,  APP_RING_H3_ACC_SIZE);
  RingBuf_Init(&g_ring_mic,     s_mic_ringbuf,     APP_RING_MIC_SIZE);
  RingBuf_Init(&g_ring_aht_env, s_aht_env_ringbuf, APP_RING_AHT_ENV_SIZE);
  RingBuf_Init(&g_ring_mag,     s_mag_ringbuf,     APP_RING_MAG_SIZE);

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
  /* HW-v2: CS/INT pins remapped — sensor tasks re-enabled */
  lsm6dsoxTaskHandle  = osThreadNew(StartLsm6dsoxTask,  NULL, &lsm6dsoxTask_attributes);
  h3lis100dlTaskHandle = osThreadNew(StartH3lis100dlTask, NULL, &h3lis100dlTask_attributes);
  qma6100pTaskHandle  = osThreadNew(StartQma6100pTask,  NULL, &qma6100pTask_attributes);
  loggerTaskHandle    = osThreadNew(StartLoggerTask,    NULL, &loggerTask_attributes);
  micTaskHandle       = osThreadNew(StartMicTask,       NULL, &micTask_attributes);
  aht20TaskHandle   = osThreadNew(StartAht20Task,   NULL, &aht20Task_attributes);
  lis2mdlTaskHandle = osThreadNew(StartLis2mdlTask, NULL, &lis2mdlTask_attributes);
  usbCdcTaskHandle = osThreadNew(StartUsbCdcTask, NULL, &usbCdcTask_attributes);
  usbUploadTaskHandle = osThreadNew(StartUsbUploadTask, NULL, &usbUploadTask_attributes);
  printf("[RTOS] lsm6dsoxTask created: %s\r\n", (lsm6dsoxTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] h3lis100dlTask created: %s\r\n", (h3lis100dlTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] qma6100pTask created: %s\r\n", (qma6100pTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] loggerTask created: %s\r\n", (loggerTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] micTask created: %s\r\n", (micTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] usbCdcTask created: %s\r\n", (usbCdcTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] usbUploadTask created: %s\r\n", (usbUploadTaskHandle != NULL) ? "ok" : "FAILED");
#endif
  /* UserCtrl task: button polling + LED + power-off.
   * Priority AboveNormal (= sensor tasks) so the power button is still polled
   * during high-rate SD acquisition — at BelowNormal it was starved and the
   * 3s power-off never triggered while logging. The task does negligible work
   * (read 2 GPIOs then osDelay 20ms), so it does not disturb sensor timing. */
  static const osThreadAttr_t userCtrlTask_attributes = {
    .name       = "userCtrlTask",
    .priority   = (osPriority_t)osPriorityAboveNormal,
    .stack_size = 512 * 4
  };
  osThreadNew(StartUserCtrlTask, NULL, &userCtrlTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* EXTI rising-edge callback. HW-v2:
 * PB0 = LSM6DSOX INT1 (FIFO watermark);
 * PA1 = H3LIS100DL DRDY;
 * PC4 = QMA6100P INT1 (FIFO watermark). */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)       /* LSM6DSOX INT1 → PB0 */
  {
    if (s_lsm_fifo_sem != NULL) { osSemaphoreRelease(s_lsm_fifo_sem); }
  }
  else if (GPIO_Pin == GPIO_PIN_1)  /* H3LIS100DL DRDY → PA1 */
  {
    if (s_h3_drdy_sem != NULL)  { osSemaphoreRelease(s_h3_drdy_sem);  }
  }
  else if (GPIO_Pin == GPIO_PIN_4)  /* QMA6100P INT1 → PC4 */
  {
    if (s_qma_fifo_sem != NULL) { osSemaphoreRelease(s_qma_fifo_sem); }
  }
  else if (GPIO_Pin == GPIO_PIN_13) /* LIS2MDL DRDY → PC13 */
  {
    if (s_mag_drdy_sem != NULL) { osSemaphoreRelease(s_mag_drdy_sem); }
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
    memset(&g_composite_frame, 0, sizeof(g_composite_frame));
    osMutexRelease(frame_buffer_mutex);
  }
}

/* QMA6100P register-0x10 BW code -> nominal ODR, using the QST datasheet
 * working points (low-power 80/200/400/800, low-noise 12.5/25/50, + 1600).
 * The datasheet's BW<4:0>->ODR table itself is missing, but the 2026-06-16
 * calibration sweep confirmed these labels: code 0x00 measured ~81Hz = the
 * datasheet's 80Hz point (NOT 100 as the old firmware mislabelled it), 0x03
 * ~803=800, 0x04 ~1601=1600. The actual rate has a small chip offset on some
 * codes (ODR = f(MCLK_SEL,BW); ~10-20% on the low/mid steps) — acceptable, like
 * the LSM/H3 nameplate-vs-real offsets. Thresholds are nearest-boundary. */
static QMA6100P_Bandwidth_t AppQmaBwToEnum(uint32_t odr_hz)
{
  if (odr_hz >= 1200U) return QMA6100P_BW_1600;  /* 1600 */
  if (odr_hz >= 600U)  return QMA6100P_BW_800;   /* 800  */
  if (odr_hz >= 300U)  return QMA6100P_BW_400;   /* 400  */
  if (odr_hz >= 140U)  return QMA6100P_BW_200;   /* 200  */
  if (odr_hz >= 65U)   return QMA6100P_BW_100;   /* 80   */
  if (odr_hz >= 37U)   return QMA6100P_BW_50;    /* 50   */
  if (odr_hz >= 18U)   return QMA6100P_BW_25;    /* 25   */
  return QMA6100P_BW_12_5;                        /* 12.5 */
}

/* Nominal ODR (Hz) for a BW code — datasheet working points (see AppQmaBwToEnum). */
static uint32_t AppQmaBwToOdrHz(QMA6100P_Bandwidth_t bw)
{
  switch (bw)
  {
    case QMA6100P_BW_1600: return 1600U;
    case QMA6100P_BW_800:  return 800U;
    case QMA6100P_BW_400:  return 400U;
    case QMA6100P_BW_200:  return 200U;
    case QMA6100P_BW_100:  return 80U;   /* datasheet low-power point (was mislabelled 100) */
    case QMA6100P_BW_50:   return 50U;
    case QMA6100P_BW_25:   return 25U;
    default:               return 12U;   /* 12.5 Hz */
  }
}

static float AppLsmXlSensitivity(uint16_t range_g)
{
  switch (range_g)
  {
    case 2:  return LSM6DSOX_XL_SENSITIVITY_2G;
    case 8:  return LSM6DSOX_XL_SENSITIVITY_8G;
    case 16: return LSM6DSOX_XL_SENSITIVITY_16G;
    default: return LSM6DSOX_XL_SENSITIVITY_4G;
  }
}

static float AppLsmGyrSensitivity(uint16_t range_dps)
{
  switch (range_dps)
  {
    case 250:  return LSM6DSOX_G_SENSITIVITY_250;
    case 500:  return LSM6DSOX_G_SENSITIVITY_500;
    case 1000: return LSM6DSOX_G_SENSITIVITY_1000;
    default:   return LSM6DSOX_G_SENSITIVITY_2000;
  }
}

/* DWT cycle counter → microseconds (160MHz, wraps at ~26.8s) */
static inline uint32_t AppDwtUs(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static uint8_t AppLsmBdrToEnum(uint16_t odr_hz)
{
  if (odr_hz >= 6664U) return LSM6DSOX_BDR_6667Hz;
  if (odr_hz >= 3332U) return LSM6DSOX_BDR_3333Hz;
  if (odr_hz >= 1666U) return LSM6DSOX_BDR_1667Hz;
  if (odr_hz >= 833U)  return LSM6DSOX_BDR_833Hz;
  if (odr_hz >= 416U)  return LSM6DSOX_BDR_417Hz;
  if (odr_hz >= 208U)  return LSM6DSOX_BDR_208Hz;
  if (odr_hz >= 104U)  return LSM6DSOX_BDR_104Hz;
  if (odr_hz >= 52U)   return LSM6DSOX_BDR_52Hz;
  if (odr_hz >= 26U)   return LSM6DSOX_BDR_26Hz;
  return LSM6DSOX_BDR_12_5Hz;
}

static uint32_t AppLsmBdrToIntervalUs(uint8_t bdr)
{
  switch (bdr)
  {
    case LSM6DSOX_BDR_6667Hz: return 150U;
    case LSM6DSOX_BDR_3333Hz: return 300U;
    case LSM6DSOX_BDR_1667Hz: return 600U;
    case LSM6DSOX_BDR_833Hz:  return 1200U;
    case LSM6DSOX_BDR_417Hz:  return 2400U;
    case LSM6DSOX_BDR_208Hz:  return 4808U;
    case LSM6DSOX_BDR_104Hz:  return 9615U;
    case LSM6DSOX_BDR_52Hz:   return 19231U;
    case LSM6DSOX_BDR_26Hz:   return 38462U;
    default:                  return 80000U;  /* 12.5Hz */
  }
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

/* 由 StartMicTask 在 ES8311 SAI DMA 真正开始采集后置 1、停采时清 0。
 * 让"诚实采集灯"在麦克风真的开录后才亮，使用户"灯亮=在录"的判断不会
 * 早于真实音频 → 不再切掉开头那句话。 */
static volatile uint8_t s_mic_capturing = 0U;

uint32_t AppAcqIsRunning(void)
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

/* 诚实采集指示（供采集灯用）：仅当采集在运行、且(麦克风未启用 或 麦克风已开录)
 * 时返回 1。灯亮 ⟹ 所有启用的源(传感器+麦克风)都已在采，用户一见灯亮即可开口，
 * 不会切掉录音开头。注意:若麦克风启用但启动失败,本函数持续返回 0(灯不亮),
 * 这本身就是"麦克风没起来"的提示。 */
uint8_t AppCaptureActive(void)
{
  if (AppAcqIsRunning() == 0U) { return 0U; }
  AcqConfig_t c;
  AcqConfig_GetCopy(&c);
  if ((c.es8311.enabled != 0U) && (s_mic_capturing == 0U)) { return 0U; }
  return 1U;
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

static uint8_t AppAcqIsSdSessionActive(void)
{
  AppAcqControl_t ctrl;

  AppAcqGetCopy(&ctrl);
  return (uint8_t)((ctrl.running != 0U) && (ctrl.sink == APP_ACQ_SINK_SD));
}

/* Route-2 RAM-corruption diagnostics, printed at SD session stop.
 *  [Stack] = per-task minimum free stack (bytes) since boot. A value near 0 on
 *            any task means its stack overflowed into adjacent .bss — the prime
 *            suspect for the stray write that corrupts the ring buffers.
 *  [Ring]  = per-session dropped bytes + peak fill vs size. hwm≈size means the
 *            ring ran full (producer overran the logger) — explains fid GAPS,
 *            and a near-full wrap is where any index/overlap bug would surface. */
static void AppPrintRuntimeDiag(void)
{
  printf("[Stack] lsm=%lu h3=%lu qma=%lu log=%lu mic=%lu cdc=%lu up=%lu (min free bytes)\r\n",
         (unsigned long)osThreadGetStackSpace(lsm6dsoxTaskHandle),
         (unsigned long)osThreadGetStackSpace(h3lis100dlTaskHandle),
         (unsigned long)osThreadGetStackSpace(qma6100pTaskHandle),
         (unsigned long)osThreadGetStackSpace(loggerTaskHandle),
         (unsigned long)osThreadGetStackSpace(micTaskHandle),
         (unsigned long)osThreadGetStackSpace(usbCdcTaskHandle),
         (unsigned long)osThreadGetStackSpace(usbUploadTaskHandle));
  printf("[Ring] lsm drop=%lu hwm=%lu/%lu | h3 drop=%lu hwm=%lu/%lu | qma drop=%lu hwm=%lu/%lu | mic drop=%lu hwm=%lu/%lu\r\n",
         (unsigned long)g_ring_lsm_imu.dropped, (unsigned long)g_ring_lsm_imu.high_watermark, (unsigned long)g_ring_lsm_imu.size,
         (unsigned long)g_ring_h3_acc.dropped,  (unsigned long)g_ring_h3_acc.high_watermark,  (unsigned long)g_ring_h3_acc.size,
         (unsigned long)g_ring_qma_acc.dropped, (unsigned long)g_ring_qma_acc.high_watermark, (unsigned long)g_ring_qma_acc.size,
         (unsigned long)g_ring_mic.dropped,     (unsigned long)g_ring_mic.high_watermark,     (unsigned long)g_ring_mic.size);

  /* Route-2 real-ODR measurement: per-sensor session sample counts (the frame_id
   * counters, reset to 0 at session start) + AppTime-measured session seconds.
   * True ODR = samples / (MIC.WAV bytes / 192000) — the mic is the hardware
   * reference clock. Compare AppTime seconds vs that to expose timestamp-clock
   * drift; compare true ODR vs configured to expose ODR-label / FIFO-loss. */
  uint64_t now_us = AppTime_GetEpochUs();
  uint32_t apptime_ms = (s_session_start_us != 0ULL && now_us > s_session_start_us)
                        ? (uint32_t)((now_us - s_session_start_us) / 1000ULL) : 0U;
  printf("[ODR] lsm=%lu qma=%lu h3=%lu samples | apptime_session=%lu ms (true ODR = samples / MIC.WAV_seconds)\r\n",
         (unsigned long)g_lsm_frame_id_counter,
         (unsigned long)g_qma_frame_id_counter,
         (unsigned long)g_h3_frame_id_counter,
         (unsigned long)apptime_ms);
  printf("[H3diag] drdy_irq=%lu timeout=%lu (irq 占比高=DRDY中断可靠)\r\n",
         (unsigned long)g_h3_irq_count, (unsigned long)g_h3_timeout_count);
}

static void AppLoggerStopSdSession(uint8_t *sd_file_open, uint32_t *rows_since_sync)
{
  if (sd_file_open == NULL)
  {
    return;
  }

  if (*sd_file_open != 0U)
  {
    /* Batching (LoggerDrainRing min_flush) can leave up to ~half a ring of
     * sub-threshold tail data unwritten. Force-flush every ring (min_flush=0)
     * before closing files, else the session tail is lost. */
    {
      uint32_t dummy_rss = 0U;
      FRESULT fr;
      int any;
      do {
        any = 0;
        if (LoggerDrainRing(&g_ring_lsm_imu, 0U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_qma_acc, 3U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_h3_acc,  2U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_mic, FATFS_SD_FILE_MIC_WAV, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_aht_env, 4U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_mag,     5U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
      } while (any != 0);
    }
    (void)DeviceCfg_WriteCurrentToSD();
    FatFs_SD_LoggerStop();
    SD_PrintWriteStats();   /* route-2: per-session SDMMC write-path health */
    AppPrintRuntimeDiag();  /* route-2: per-task stack margin + ring overrun */
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

  /* WCID Bulk: stop USB streaming. */
  if (g_boot_mode == BOOT_MODE_WCID_BULK)
  {
    UsbWcidApp_StopStreaming();
  }
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
  if ((ctrl.running == 0U) || (ctrl.duration_ms == 0U) || (ctrl.timer_armed == 0U))
  {
    return;
  }

  /* auto-stop 计时必须用真实时钟(apptime)——RTOS tick 比真实时间慢 ~19%（实测 44000
   * tick=52.5s），用 tick 会让 44s 跑成 ~52s。但 AppTime_GetEpochUs 带 __disable_irq +
   * DWT wrap 维护，被各采集任务高频调用会拖垮 SD 吞吐（实测 ring 大量溢出 CKBX0304）。
   * 故 tick 限流：每 ~250 tick 才真正取一次 apptime（auto-stop 不需要 ms 精度）。 */
  static uint32_t s_autostop_last_tick = 0U;
  if ((now_ms - s_autostop_last_tick) < 250U) { return; }
  s_autostop_last_tick = now_ms;

  uint64_t now_us = AppTime_GetEpochUs();
  uint32_t elapsed_ms = (now_us > ctrl.start_us) ? (uint32_t)((now_us - ctrl.start_us) / 1000ULL) : 0U;
  if (elapsed_ms >= ctrl.duration_ms)
  {
    printf("[Acq] auto-stop: elapsed_apptime=%lu ms dur=%lu ms (tick_elapsed=%lu)\r\n",
           (unsigned long)elapsed_ms, (unsigned long)ctrl.duration_ms,
           (unsigned long)(now_ms - ctrl.start_tick_ms));
    AppAcqStopInternal(now_ms);
  }
}
/* Arm the session countdown from now — called when SD files are open and data
 * actually starts flowing, so duration_ms is measured from SD-ready, not from
 * the acq_start command (which includes SD init overhead). Safe to call even
 * if another task already triggered stop_pending: we re-arm unconditionally
 * so the logger gets the full duration for actual data capture. */
static void AppAcqResetSessionTimer(void)
{
  if (acq_ctrl_mutex == NULL) { return; }
  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  if (g_acq_ctrl.duration_ms != 0U)
  {
    g_acq_ctrl.running       = 1U;
    g_acq_ctrl.stop_pending  = 0U;
    g_acq_ctrl.timer_armed   = 1U;
    g_acq_ctrl.start_tick_ms = osKernelGetTickCount();
    g_acq_ctrl.start_us      = AppTime_GetEpochUs();  /* 真实时钟基准（tick 慢 19% 不可用） */
  }
  uint8_t  dbg_armed = g_acq_ctrl.timer_armed;
  uint32_t dbg_start = g_acq_ctrl.start_tick_ms;
  uint32_t dbg_dur   = g_acq_ctrl.duration_ms;
  osMutexRelease(acq_ctrl_mutex);
  printf("[Acq] timer armed=%u start_tick=%lu duration_ms=%lu\r\n",
         (unsigned)dbg_armed, (unsigned long)dbg_start, (unsigned long)dbg_dur);
}

/* Apply current AcqConfig to all three sensors. Called on each acq_start so
 * set_sensor changes take effect without restarting the tasks. */
static void AppApplySensorConfig(void)
{
  AcqConfig_t cfg;
  AcqConfig_GetCopy(&cfg);

  /* --- LSM6DSOX (SPI1, no mutex needed) --- */
  if (cfg.lsm6dsox.enabled != 0U)
  {
    uint8_t bdr = AppLsmBdrToEnum(cfg.lsm6dsox.odr_hz);
    s_lsm_odr_interval_us = AppLsmBdrToIntervalUs(bdr);
    (void)LSM6DSOX_FIFO_Config(256U, bdr, bdr);
  }
  else
  {
    printf("[LSM6DSOX] disabled, skip reconfig\r\n");
  }

  /* --- QMA6100P + H3LIS100DL (SPI2, need mutex) --- */
  osMutexAcquire(spi2_mutex, osWaitForever);

  /* QMA */
  if (cfg.qma6100p.enabled != 0U)
  {
    QMA6100P_Bandwidth_t bw = AppQmaBwToEnum((uint32_t)cfg.qma6100p.odr_hz);
    QMA6100P_Config_t qcfg = { .range = QMA6100P_RANGE_4G, .bw = bw };
    if (QMA6100P_Configure(&qcfg) == HAL_OK)
    {
      uint32_t actual_odr = AppQmaBwToOdrHz(bw);
      s_qma_odr_interval_us = 1000000U / actual_odr;
    }
    (void)QMA6100P_FIFO_Config(16U);
  }
  else
  {
    printf("[QMA6100P] disabled, skip reconfig\r\n");
  }

  /* H3: write CTRL_REG1 DR bits (else odr_hz is ignored) + update fallback
   * interval. Same per-apply path as LSM/QMA above, so "s h3 odr N" takes
   * effect immediately without a reboot. spi2_mutex already held here.
   * Datasheet Table 20: DR 00/01/10 = 50/100/400 Hz. */
  if (cfg.h3lis100dl.enabled != 0U)
  {
    uint32_t h3_odr = (cfg.h3lis100dl.odr_hz > 0U) ? (uint32_t)cfg.h3lis100dl.odr_hz : 400U;
    H3LIS100DL_Config_t h3_cfg;
    if (h3_odr <= 75U)       { h3_cfg.odr = H3LIS100DL_ODR_50HZ;  h3_odr = 50U;  }
    else if (h3_odr <= 250U) { h3_cfg.odr = H3LIS100DL_ODR_100HZ; h3_odr = 100U; }
    else                     { h3_cfg.odr = H3LIS100DL_ODR_400HZ; h3_odr = 400U; }
    (void)H3LIS100DL_Configure(&h3_cfg);
    s_h3_odr_interval_us = 1000000U / h3_odr;
  }
  else
  {
    printf("[H3LIS100DL] disabled, skip reconfig\r\n");
  }

  osMutexRelease(spi2_mutex);

  /* --- LIS2MDL 磁力计 (I2C1, 需 mutex) ---
   * 与 LSM/QMA/H3 一样每次 acq_start 重配,使 "s mag odr N" 改了即生效(原先 odr 只在
   * LIS2MDL 任务开机时锁定一次,必须重启才生效)。仅在使能时重配;禁用时不动(与其它
   * 传感器一致:停一个已在跑的传感器仍需重启)。AHT20 无需重配——它是命令触发、固定
   * ~1Hz、无 ODR 寄存器,改不了采样率。 */
  if (cfg.lis2mdl.enabled != 0U)
  {
    osMutexAcquire(i2c1_mutex, osWaitForever);
    (void)LIS2MDL_Init(cfg.lis2mdl.odr_hz);
    osMutexRelease(i2c1_mutex);
  }
  else
  {
    printf("[LIS2MDL] disabled, skip reconfig\r\n");
  }
}

uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms)
{
  /* 每次采集启动时重新同步 RTC 锚点 */
  AppTime_Sync();
  uint32_t now_ms = osKernelGetTickCount();

  if ((sink != APP_ACQ_SINK_USB) && (sink != APP_ACQ_SINK_SD))
  {
    return 0U;
  }

  if (acq_ctrl_mutex == NULL)
  {
    return 0U;
  }

  /* Apply latest sensor config before starting acquisition */
  AppApplySensorConfig();

  AppSnapshotReset();

  osMutexAcquire(acq_ctrl_mutex, osWaitForever);
  g_acq_ctrl.running = 1U;
  g_acq_ctrl.sink = sink;
  g_acq_ctrl.requested_hz = 0U;
  g_acq_ctrl.period_ms = 0U;
  g_acq_ctrl.effective_hz = 0U;
  g_acq_ctrl.duration_ms = duration_ms;
  g_acq_ctrl.start_tick_ms = now_ms;
  g_acq_ctrl.stop_pending = 0U;
  /* SD session: timer starts only after SD files are open (logger arms it via
   * AppAcqResetSessionTimer, which also sets start_us). USB session: no init
   * delay — arm immediately AND set the apptime base here, otherwise start_us
   * stays stale/0 and the auto-stop computes a huge elapsed → instant stop. */
  g_acq_ctrl.timer_armed = (sink == APP_ACQ_SINK_USB) ? 1U : 0U;
  if (sink == APP_ACQ_SINK_USB)
  {
    g_acq_ctrl.start_us = AppTime_GetEpochUs();  /* real-clock base (tick is 19% slow) */
  }
  osMutexRelease(acq_ctrl_mutex);

  /* 按 sink 选 LSM FIFO 读法:USB→每字节(碎读不丢 USB 帧),SD→7字节(快读满采)。
   * 二分实测确诊:7字节读 SD满采但USB丢~6%;每字节读 USB好但SD欠采~13%。两者需求
   * 相反,按 sink 动态切才两全。配合 USB sink 时 LSM 不被高优先级抢占。 */
  LSM6DSOX_SetFifoReadPerByte((sink == APP_ACQ_SINK_USB) ? 1U : 0U);

  AppFlowStatsSetMode((uint8_t)((sink == APP_ACQ_SINK_USB) ? 1U : 0U),
                      (uint8_t)((sink == APP_ACQ_SINK_SD) ? 1U : 0U));

  /* WCID Bulk: start USB streaming when sink is USB.
   * Pass per-channel ODR so each USB half-buffer is sized to its data rate
   * (high-ODR LSM needs a big half to avoid overrun; low-ODR H3 needs a small
   * half so it still fills/flushes within the capture). */
  if (g_boot_mode == BOOT_MODE_WCID_BULK && sink == APP_ACQ_SINK_USB)
  {
    AcqConfig_t scfg;
    AcqConfig_GetCopy(&scfg);
    uint32_t h3_odr = (scfg.h3lis100dl.odr_hz > 0U) ? (uint32_t)scfg.h3lis100dl.odr_hz : 400U;
    UsbWcidApp_StartStreaming((uint32_t)scfg.lsm6dsox.odr_hz, h3_odr,
                              (uint32_t)scfg.qma6100p.odr_hz);
  }

  return 1U;
}

uint32_t AppAcqStop(void)
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

  if (ctrl.running != 0U)
  {
    if (ctrl.duration_ms == 0U)
      USB_ACQ_LINE("running  sink=%s\r\n", AppAcqSinkToString(ctrl.sink));
    else
      USB_ACQ_LINE("running  sink=%s  elapsed=%lums  remaining=%lums\r\n",
                   AppAcqSinkToString(ctrl.sink),
                   (unsigned long)elapsed_ms,
                   (unsigned long)remaining_ms);
  }
  else
  {
    USB_ACQ_LINE("stopped\r\n");
  }

#undef USB_ACQ_LINE
}

static void UsbCmd_AcqStart(const char *cmd)
{
  char sink_text[8] = {0};
  unsigned long duration_ms = 0UL;
  uint8_t sink = 0U;

  /* 手动解析 —— 不能用 sscanf 的 %lu：本工程用 Keil microLIB，其精简版 sscanf
   * 不支持 %lu，会把 "acq_start sd 44000" 的 44000 丢掉、duration_ms 恒为 0，
   * 导致定时采集失效（变成无限手动停）。改用 token 切分 + strtoul（microLIB 支持）。
   * 调用前已 strncmp("acq_start ",10) 命中，故 cmd+10 起为参数。 */
  {
    const char *p = cmd + 10;                 /* 跳过 "acq_start " */
    int i = 0;
    while (*p == ' ') { p++; }                /* 跳过前导空格 */
    while ((*p != '\0') && (*p != ' ') && (i < 7)) { sink_text[i++] = *p++; }  /* sink */
    sink_text[i] = '\0';
    while (*p == ' ') { p++; }                /* 跳到 duration */
    if ((*p >= '0') && (*p <= '9')) { duration_ms = strtoul(p, NULL, 10); }
  }

  if (sink_text[0] == '\0')
  {
    const char *msg = "Usage: acq_start <usb|sd> [duration_ms]\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
    return;
  }

  printf("[Acq] acq_start sink=%s duration_ms=%lu\r\n", sink_text, duration_ms);

  if (AppAcqParseSink(sink_text, &sink) == 0U)
  {
    const char *msg = "ERR invalid sink (use usb or sd)\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
    return;
  }

  if (AppAcqStart(sink, (uint32_t)duration_ms) == 0U)
  {
    const char *msg = "ERR acq_start failed\r\n";
    UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
    return;
  }

  if ((sink == APP_ACQ_SINK_USB) && (duration_ms > 0UL))
    s_usb_done_armed = 1U;

  UsbCdcService_Write((const uint8_t *)"start\r\n", 7U);
}

static void UsbCmd_AcqStop(void)
{
  const char *msg;

  AppAcqStop();
  s_usb_done_armed = 0U;
  msg = "stop\r\n";
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

/* === Lock-free SPSC ring buffer ============================================ */
static inline uint32_t AppU32ToDec(char *out, uint32_t v)
{
  /* Fast unsigned->decimal — replaces snprintf in hot paths. newlib's
   * snprintf isn't safe to call from multiple FreeRTOS tasks at high rates
   * because it uses static reentrancy state without __malloc_lock here. */
  char tmp[10];
  uint32_t n = 0;
  if (v == 0U) { out[0] = '0'; return 1U; }
  while (v > 0U) { tmp[n++] = (char)('0' + (v % 10U)); v /= 10U; }
  for (uint32_t i = 0; i < n; i++) out[i] = tmp[n - 1U - i];
  return n;
}

static inline uint32_t AppI32ToDec(char *out, int32_t v)
{
  if (v < 0) { out[0] = '-'; return 1U + AppU32ToDec(out + 1, (uint32_t)(-v)); }
  return AppU32ToDec(out, (uint32_t)v);
}

static inline uint32_t AppU64ToDec(char *out, uint64_t v)
{
  if (v == 0ULL) { out[0] = '0'; return 1U; }
  char tmp[20];
  uint32_t n = 0U;
  while (v > 0ULL) { tmp[n++] = (char)('0' + (uint8_t)(v % 10ULL)); v /= 10ULL; }
  for (uint32_t i = 0U; i < n; i++) { out[i] = tmp[n - 1U - i]; }
  return n;
}

/* 把 Unix 纪元秒格式化为 12 位 YYMMDDHHMMSS（定长，无分隔符，UTC 无时区）。
 * 返回写入字节数（固定 12）。秒级精度：同一秒内的多个样本时间戳相同，
 * 秒内顺序靠连续递增的 frame_id 区分。 */
static uint32_t AppFmtDateTime12(char *out, uint32_t epoch_s)
{
  Pcf85063_Time_t t;
  Pcf85063_FromEpochSeconds(epoch_s, &t);
  out[0]  = (char)('0' + t.year   / 10U); out[1]  = (char)('0' + t.year   % 10U);
  out[2]  = (char)('0' + t.month  / 10U); out[3]  = (char)('0' + t.month  % 10U);
  out[4]  = (char)('0' + t.day    / 10U); out[5]  = (char)('0' + t.day    % 10U);
  out[6]  = (char)('0' + t.hour   / 10U); out[7]  = (char)('0' + t.hour   % 10U);
  out[8]  = (char)('0' + t.minute / 10U); out[9]  = (char)('0' + t.minute % 10U);
  out[10] = (char)('0' + t.second / 10U); out[11] = (char)('0' + t.second % 10U);
  return 12U;
}

/* Write float as "dd.d" (one decimal place) into out. Returns bytes written. */
static inline uint32_t AppF1ToDec(char *out, float v)
{
  int32_t scaled = (int32_t)(v >= 0.0f ? v * 10.0f + 0.5f : v * 10.0f - 0.5f);
  int32_t int_part = scaled / 10;
  uint32_t frac = (uint32_t)(scaled < 0 ? -(scaled % 10) : scaled % 10);
  uint32_t n;
  if (int_part == 0 && scaled < 0) {
    /* scaled in [-9,-1]: v in (-0.9,-0.1); integer division truncates to 0,
     * losing the sign. Write "-0.X" directly. */
    out[0] = '-'; out[1] = '0'; out[2] = '.'; out[3] = (char)('0' + frac);
    return 4U;
  }
  n = AppI32ToDec(out, int_part);
  out[n++] = '.';
  out[n++] = (char)('0' + frac);
  return n;
}

static void RingBuf_Init(AppRingBuffer_t *rb, uint8_t *data, uint32_t size)
{
  rb->data = data;
  rb->size = size;
  rb->wr_idx = 0U;
  rb->rd_idx = 0U;
  rb->dropped = 0U;
  rb->high_watermark = 0U;
}

/* Discard all buffered bytes. Called by the logger when a new SD session
 * starts so stale data produced between sessions does not leak into the
 * new file. Safe because the producer only appends and the consumer is the
 * caller; setting rd_idx = wr_idx empties the buffer atomically enough for
 * the SPSC pattern (a concurrent producer write just adds fresh data). */
static void RingBuf_Reset(AppRingBuffer_t *rb)
{
  rb->rd_idx = rb->wr_idx;
  rb->dropped = 0U;
  rb->high_watermark = 0U;   /* per-session peak fill for diagnostics */
}

static uint32_t RingBuf_Available(const AppRingBuffer_t *rb)
{
  uint32_t wr = rb->wr_idx;
  uint32_t rd = rb->rd_idx;
  if (wr >= rd) return wr - rd;
  return rb->size - rd + wr;
}

static uint32_t RingBuf_FreeSpace(const AppRingBuffer_t *rb)
{
  return rb->size - 1U - RingBuf_Available(rb);
}

static uint32_t RingBuf_Write(AppRingBuffer_t *rb, const uint8_t *src, uint32_t len)
{
  if (rb == NULL || rb->data == NULL || src == NULL || len == 0U) return 0U;
  if (RingBuf_FreeSpace(rb) < len)
  {
    rb->dropped += len;
    return 0U;
  }
  uint32_t wr = rb->wr_idx;
  uint32_t first = rb->size - wr;
  if (first >= len)
  {
    memcpy(&rb->data[wr], src, len);
    wr = (wr + len) % rb->size;
  }
  else
  {
    memcpy(&rb->data[wr], src, first);
    memcpy(&rb->data[0], src + first, len - first);
    wr = len - first;
  }
  rb->wr_idx = wr;
  uint32_t avail = RingBuf_Available(rb);
  if (avail > rb->high_watermark) rb->high_watermark = avail;
  return len;
}

/* Non-static producer wrapper for the mic ring (SPSC). Sole writer is the SAI
 * DMA ISR (HAL_SAI_RxHalf/CpltCallback in mic_capture.c). Resolves the extern
 * declared in mic_capture.c. */
uint32_t AppRing_WriteMic(const uint8_t *src, uint32_t len)
{
  return RingBuf_Write(&g_ring_mic, src, len);
}

/* Return pointer to next contiguous chunk of unread bytes (no wrap), capped
 * at APP_RING_FLUSH_CHUNK. Caller calls RingBuf_Consume after a successful
 * f_write to advance rd_idx. */
static uint32_t RingBuf_PeekContiguous(AppRingBuffer_t *rb, const uint8_t **out_ptr, uint32_t *out_len)
{
  if (rb == NULL || rb->data == NULL || out_ptr == NULL || out_len == NULL) return 0U;
  uint32_t wr = rb->wr_idx;
  uint32_t rd = rb->rd_idx;
  if (wr == rd) return 0U;

  uint32_t span = (wr > rd) ? (wr - rd) : (rb->size - rd);
  if (span > APP_RING_FLUSH_CHUNK) span = APP_RING_FLUSH_CHUNK;
  *out_ptr = &rb->data[rd];
  *out_len = span;
  return span;
}

static void RingBuf_Consume(AppRingBuffer_t *rb, uint32_t len)
{
  if (rb == NULL || len == 0U) return;
  rb->rd_idx = (rb->rd_idx + len) % rb->size;
}

/* Private bounce buffer for SD flushes. The logger copies each peeked ring
 * chunk here BEFORE f_write so the SDMMC IDMA never reads directly from the
 * live ring. This decouples SD timing (PROGRAMMING stalls, abort/retry, the
 * DeInit/re-init recovery path) from the producer: the DMA source stays
 * perfectly static for the whole transfer even if the write is retried.
 * One buffer is reused across all files — the logger drains serially and each
 * f_write blocks on the SD DMA-complete semaphore before the next starts. */
static uint8_t s_sd_bounce[APP_RING_FLUSH_CHUNK];

/* Drain one contiguous chunk from a ring to its file via s_sd_bounce.
 * min_flush = batching threshold: if fewer than min_flush bytes are buffered AND
 * the ring is below half full, skip the write so data accumulates into a large
 * block first. Each SD write carries a big fixed cost (command + card
 * PROGRAMMING + IRQ-off entry/exit); the old greedy drain emitted ~1.5KB writes
 * at ~446/s, so that fixed cost dominated. Accumulating to ~FLUSH_CHUNK cuts the
 * write count ~10x → far higher effective throughput and far less total IRQ-off
 * time, all in polling mode (zero corruption). Pass min_flush = 0 to force a
 * flush regardless (session stop). The half-full override still bounds latency
 * and guarantees no ring overflow.
 * Returns:  1 = wrote a chunk (caller keeps looping),
 *           0 = nothing to do / accumulating / audio dropped (no progress),
 *          -1 = write error (caller breaks; *out_res holds the FRESULT). */
static int LoggerDrainRing(AppRingBuffer_t *rb, uint8_t file_idx, uint32_t min_flush,
                           uint32_t *rows_since_sync, FRESULT *out_res)
{
  const uint8_t *p; uint32_t n;
  /* Batching gate: hold back small writes unless the ring is past half full
   * (then drain to bound latency / prevent overflow). min_flush==0 bypasses. */
  if (min_flush != 0U)
  {
    uint32_t avail = RingBuf_Available(rb);
    if (avail < min_flush && avail < (rb->size / 2U)) return 0;
  }
  if (RingBuf_PeekContiguous(rb, &p, &n) == 0U) return 0;

  memcpy(s_sd_bounce, p, n);
  FRESULT r = FatFs_SD_LoggerWriteFileIndex(file_idx, s_sd_bounce, n);
  *out_res = r;

  if (r == FR_OK)
  {
    RingBuf_Consume(rb, n);
    *rows_since_sync += n;
    return 1;
  }
  if ((r == FR_NOT_ENABLED) && (file_idx == FATFS_SD_FILE_MIC_WAV))
  {
    /* WAV not open but the SAI ISR keeps filling g_ring_mic — drop the audio
     * and keep the drain loop healthy (matches prior behaviour). */
    RingBuf_Consume(rb, n);
    return 0;
  }
  return -1;
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
  const char *msg = "Commands: ping, help, status, acq_start, acq_stop, s <lsm|h3|qma|aht|mag> <odr|range|en> <val>, s mic <gain|sr|en> <val>, set_time YYYY-MM-DDTHH:MM:SS, boot_msc\r\n";
  UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
}

static void UsbCmd_SetSensor(const char *cmd)
{
  char sensor[8] = {0};
  char param[8]  = {0};
  unsigned long val = 0UL;
  char line[80];
  int len;
  int updated = 0;

  /* 手动解析 —— microLIB 的 sscanf 不支持 %lu(同 acq_start 的坑)：原 sscanf 会把
   * value 丢掉、val 为乱值/0，导致 s 命令实际改不动配置。格式 "s <sensor> <param> <value>"。 */
  {
    const char *p = cmd + 1;                  /* 跳过 's' */
    int i;
    while (*p == ' ') { p++; }
    i = 0; while ((*p != '\0') && (*p != ' ') && (i < 7)) { sensor[i++] = *p++; } sensor[i] = '\0';
    while (*p == ' ') { p++; }
    i = 0; while ((*p != '\0') && (*p != ' ') && (i < 7)) { param[i++] = *p++; } param[i] = '\0';
    while (*p == ' ') { p++; }
    if ((sensor[0] == '\0') || (param[0] == '\0') || (*p < '0') || (*p > '9'))
    {
      const char *msg = "Usage: s <lsm|h3|qma|aht|mag> <odr|range|en> <value> | s mic <gain|sr|en> <value>\r\n";
      UsbCdcService_Write((const uint8_t *)msg, strlen(msg));
      return;
    }
    val = strtoul(p, NULL, 10);
  }
  printf("[Acq] set sensor=%s param=%s val=%lu\r\n", sensor, param, val);

  /* 麦克风(ES8311)配置：s mic <gain|sr|en> <val>。参数与传感器的 odr/range/en 不同，
   * 单独处理后提前返回。AcqConfig_SetMic 一次写全部三项，故先读当前值、只改目标项；
   * sr 传 0 表示保持当前采样率。配置在下次会话 Mic_Start 时生效（与 acq_start 配合）。 */
  if (strcmp(sensor, "mic") == 0)
  {
    AcqConfig_t mcfg;
    AcqConfig_GetCopy(&mcfg);
    uint8_t  m_en   = mcfg.es8311.enabled;
    uint32_t m_sr   = 0U;                 /* 0 = 保持当前采样率 */
    uint16_t m_gain = mcfg.es8311.gain_db;
    int ok = 1;

    if      (strcmp(param, "gain") == 0) { m_gain = (uint16_t)val; }       /* 0..42 */
    else if (strcmp(param, "sr")   == 0) { m_sr   = (uint32_t)val; }       /* 8000/16000/48000/96000 */
    else if (strcmp(param, "en")   == 0) { m_en   = (val != 0U) ? 1U : 0U; }
    else                                 { ok = 0; }

    if (!ok)
    {
      len = snprintf(line, sizeof(line), "ERR: use gain, sr, or en\r\n");
    }
    else if (AcqConfig_SetMic(m_en, m_sr, m_gain) == 0)
    {
      AcqConfig_GetCopy(&mcfg);          /* 读回实际生效值 */
      len = snprintf(line, sizeof(line), "OK mic en=%u sr=%lu gain=%u\r\n",
                     (unsigned)mcfg.es8311.enabled,
                     (unsigned long)mcfg.es8311.sample_rate_hz,
                     (unsigned)mcfg.es8311.gain_db);
      (void)DeviceCfg_WriteCurrentToSD();
    }
    else
    {
      len = snprintf(line, sizeof(line), "ERR: sr=8000/16000/48000/96000, gain<=42\r\n");
    }
    UsbCdcService_Write((const uint8_t *)line, (uint32_t)len);
    return;
  }

  uint8_t which;
  if      (strcmp(sensor, "lsm") == 0) which = 0U;
  else if (strcmp(sensor, "h3")  == 0) which = 1U;
  else if (strcmp(sensor, "qma") == 0) which = 2U;
  else if (strcmp(sensor, "aht") == 0) which = 3U;
  else if (strcmp(sensor, "mag") == 0) which = 4U;
  else
  {
    len = snprintf(line, sizeof(line), "ERR: use lsm/h3/qma/aht/mag\r\n");
    UsbCdcService_Write((const uint8_t *)line, (uint32_t)len);
    return;
  }

  AcqConfig_t cfg;
  AcqConfig_GetCopy(&cfg);

  if (strcmp(param, "odr") == 0)
  {
    /* Snap to nearest supported ODR so CONFIG.JSN matches actual hardware */
    uint16_t snapped = (uint16_t)val;
    switch (which)
    {
      case 0U: {
        static const uint16_t lsm_odrs[] = {12,26,52,104,208,416,833,1666,3332,6664};
        snapped = lsm_odrs[0];
        for (unsigned i = 1; i < sizeof(lsm_odrs)/sizeof(lsm_odrs[0]); i++)
          if (abs((int)val - (int)lsm_odrs[i]) < abs((int)val - (int)snapped))
            snapped = lsm_odrs[i];
        cfg.lsm6dsox.odr_hz = snapped;
        break;
      }
      case 1U: {
        static const uint16_t h3_odrs[] = {50,100,400};
        snapped = h3_odrs[0];
        for (unsigned i = 1; i < sizeof(h3_odrs)/sizeof(h3_odrs[0]); i++)
          if (abs((int)val - (int)h3_odrs[i]) < abs((int)val - (int)snapped))
            snapped = h3_odrs[i];
        cfg.h3lis100dl.odr_hz = snapped;
        break;
      }
      case 2U: {
        /* QST datasheet working points (12=12.5Hz); all 8 BW codes 0x00-0x07. */
        static const uint16_t qma_odrs[] = {12,25,50,80,200,400,800,1600};
        snapped = qma_odrs[0];
        for (unsigned i = 1; i < sizeof(qma_odrs)/sizeof(qma_odrs[0]); i++)
          if (abs((int)val - (int)qma_odrs[i]) < abs((int)val - (int)snapped))
            snapped = qma_odrs[i];
        cfg.qma6100p.odr_hz = snapped;
        break;
      }
      case 3U: {
        cfg.aht20.odr_hz = 1U;   /* AHT20 固定 1Hz */
        snapped = 1U;
        break;
      }
      case 4U: {
        static const uint16_t mag_odrs[] = {10,20,50,100};
        snapped = mag_odrs[0];
        for (unsigned i = 1; i < sizeof(mag_odrs)/sizeof(mag_odrs[0]); i++)
          if (abs((int)val - (int)mag_odrs[i]) < abs((int)val - (int)snapped))
            snapped = mag_odrs[i];
        cfg.lis2mdl.odr_hz = snapped;
        break;
      }
    }
    if (AcqConfig_Set(&cfg) == 0)
    {
      len = snprintf(line, sizeof(line), "OK %s odr=%u\r\n", sensor, (unsigned)snapped);
      updated = 1;
    }
    else
      len = snprintf(line, sizeof(line), "ERR\r\n");
  }
  else if (strcmp(param, "range") == 0)
  {
    switch (which)
    {
      case 0U: cfg.lsm6dsox.range   = (uint16_t)val; break;
      case 1U: cfg.h3lis100dl.range = (uint16_t)val; break;
      case 2U: cfg.qma6100p.range   = (uint16_t)val; break;
    }
    if (AcqConfig_Set(&cfg) == 0)
    {
      len = snprintf(line, sizeof(line), "OK %s range=%lu\r\n", sensor, val);
      updated = 1;
    }
    else
      len = snprintf(line, sizeof(line), "ERR\r\n");
  }
  else if (strcmp(param, "en") == 0)
  {
    uint8_t en = (val != 0U) ? 1U : 0U;
    switch (which)
    {
      case 0U: cfg.lsm6dsox.enabled   = en; break;
      case 1U: cfg.h3lis100dl.enabled = en; break;
      case 2U: cfg.qma6100p.enabled   = en; break;
      case 3U: cfg.aht20.enabled      = en; break;
      case 4U: cfg.lis2mdl.enabled    = en; break;
    }
    if (AcqConfig_Set(&cfg) == 0)
    {
      len = snprintf(line, sizeof(line), "OK %s en=%u\r\n", sensor, (unsigned)en);
      updated = 1;
    }
    else
      len = snprintf(line, sizeof(line), "ERR\r\n");
  }
  else
  {
    len = snprintf(line, sizeof(line), "ERR: use odr, range, or en\r\n");
  }
  UsbCdcService_Write((const uint8_t *)line, (uint32_t)len);

  /* 同步写回 SD 卡 + 立即配置传感器硬件 */
  if (updated)
  {
    (void)DeviceCfg_WriteCurrentToSD();
    AppApplySensorConfig();
  }
}

static void UsbCmd_Status(void)
{
  char line[128];
  int len;
  AcqConfig_t cfg;
  AcqConfig_GetCopy(&cfg);

#define STATUS_LINE(fmt, ...) \
  do { \
    len = snprintf(line, sizeof(line), fmt, ##__VA_ARGS__); \
    if (len > 0 && (uint32_t)len < sizeof(line)) \
      UsbCdcService_Write((const uint8_t *)line, (uint32_t)len); \
  } while (0)

  STATUS_LINE("lsm  en=%u  odr=%uHz  xl=+-%ug  gyro=+-%udps\r\n",
              (unsigned)cfg.lsm6dsox.enabled,
              (unsigned)cfg.lsm6dsox.odr_hz,
              (unsigned)cfg.lsm6dsox.range,
              (unsigned)cfg.lsm6dsox.range2);
  STATUS_LINE("h3   en=%u  odr=%uHz  range=+-%ug\r\n",
              (unsigned)cfg.h3lis100dl.enabled,
              (unsigned)cfg.h3lis100dl.odr_hz,
              (unsigned)cfg.h3lis100dl.range);
  STATUS_LINE("qma  en=%u  odr=%uHz  range=+-%ug\r\n",
              (unsigned)cfg.qma6100p.enabled,
              (unsigned)cfg.qma6100p.odr_hz,
              (unsigned)cfg.qma6100p.range);
  STATUS_LINE("mic  en=%u  sr=%luHz  gain=%udB\r\n",
              (unsigned)cfg.es8311.enabled,
              (unsigned long)cfg.es8311.sample_rate_hz,
              (unsigned)cfg.es8311.gain_db);
  STATUS_LINE("aht  en=%u  odr=%uHz\r\n",
              (unsigned)cfg.aht20.enabled,
              (unsigned)cfg.aht20.odr_hz);
  STATUS_LINE("mag  en=%u  odr=%uHz  range=+-%ugauss\r\n",
              (unsigned)cfg.lis2mdl.enabled,
              (unsigned)cfg.lis2mdl.odr_hz,
              (unsigned)cfg.lis2mdl.range);

#undef STATUS_LINE
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
  else if (cmd[0] == 's' && cmd[1] == ' ')
  {
    UsbCmd_SetSensor(cmd);
  }
  else if (strcmp(cmd, "boot_msc") == 0)
  {
    printf("Switching to USB MSC mode...\r\n");
    UsbCmd_AcqStop();
    osDelay(500U);
    (void)DeviceCfg_WriteCurrentToSD();
    BootMode_Write(BOOT_MODE_USB_MSC);
    boot_mode_t verify = BootMode_Read();
    printf("BootMode flag written, readback=%d, resetting...\r\n", (int)verify);
    osDelay(200U);
    NVIC_SystemReset();
  }
  else if (strncmp(cmd, "set_time ", 9U) == 0)
  {
    /* 格式: set_time YYYY-MM-DDTHH:MM:SS（从偏移 9 开始，共 19 字符）
     * 位置：0123456789012345678
     *       YYYY-MM-DDTHH:MM:SS
     * p[0..3]=年, p[5..6]=月, p[8..9]=日, p[11..12]=时, p[14..15]=分, p[17..18]=秒 */
    const char *p = cmd + 9U;
    if (strlen(p) < 19U)
    {
      UsbCdcService_Write((const uint8_t *)"ERR set_time: parse fail\r\n", 26U);
    }
    else
    {
      uint32_t yr = (uint32_t)(p[2] - '0') * 10U + (uint32_t)(p[3] - '0'); /* 年后两位 */
      uint32_t mo = (uint32_t)(p[5] - '0') * 10U + (uint32_t)(p[6] - '0');
      uint32_t dy = (uint32_t)(p[8] - '0') * 10U + (uint32_t)(p[9] - '0');
      uint32_t hr = (uint32_t)(p[11] - '0') * 10U + (uint32_t)(p[12] - '0');
      uint32_t mn = (uint32_t)(p[14] - '0') * 10U + (uint32_t)(p[15] - '0');
      uint32_t sc = (uint32_t)(p[17] - '0') * 10U + (uint32_t)(p[18] - '0');
      if (mo >= 1U && mo <= 12U && dy >= 1U && dy <= 31U &&
          hr <= 23U && mn <= 59U && sc <= 59U)
      {
        Pcf85063_Time_t t;
        t.year = (uint8_t)yr;  t.month  = (uint8_t)mo; t.day    = (uint8_t)dy;
        t.hour = (uint8_t)hr;  t.minute = (uint8_t)mn; t.second = (uint8_t)sc;
        if (Pcf85063_SetTime(&t) == PCF85063_OK)
        {
          AppTime_Sync();
          char _line[48];
          int _n = snprintf(_line, sizeof(_line), "OK set_time %04u-%02u-%02uT%02u:%02u:%02u\r\n",
                            2000U + yr, mo, dy, hr, mn, sc);
          UsbCdcService_Write((const uint8_t *)_line, (uint32_t)_n);
        }
        else
        {
          UsbCdcService_Write((const uint8_t *)"ERR set_time: RTC write fail\r\n", 30U);
        }
      }
      else { UsbCdcService_Write((const uint8_t *)"ERR set_time: parse fail\r\n", 26U); }
    }
  }
  else
  {
    UsbCdcService_Write((const uint8_t *)"ERR: unknown cmd\r\n", 18U);
  }
}


/* Deferred command state: ISR writes here, WCID task reads in task context. */
static volatile uint8_t s_wcid_cmd_pending;
static char             s_wcid_cmd_buf[64];

/* Command callback for WCID Bulk mode — called from USB class driver Receive (ISR context).
 * Must NOT call any RTOS blocking function. Only copy + flag; task loop processes it. */
static void WcidCmdCallback(const char *cmd, uint32_t len)
{
  uint32_t n = (len < sizeof(s_wcid_cmd_buf) - 1U) ? len : sizeof(s_wcid_cmd_buf) - 1U;
  memcpy(s_wcid_cmd_buf, cmd, n);
  s_wcid_cmd_buf[n] = '\0';
  while (n > 0U && (s_wcid_cmd_buf[n - 1U] == '\r' ||
                    s_wcid_cmd_buf[n - 1U] == '\n' ||
                    s_wcid_cmd_buf[n - 1U] == ' '))
  {
    s_wcid_cmd_buf[--n] = '\0';
  }
  if (n > 0U)
  {
    s_wcid_cmd_pending = 1U;
  }
}

extern USBD_HandleTypeDef hUSB_Device;

void StartUsbCdcTask(void *argument)
{
  (void)argument;

  printf("[WCID] task entered, boot_mode=%d\r\n", (int)g_boot_mode);

  if (g_boot_mode == BOOT_MODE_WCID_BULK)
  {
    /* WCID Bulk mode: PCD already init'd in main, just start Classic USBD core. */
    printf("[WCID] calling USBD_Init...\r\n");
    USBD_StatusTypeDef st = USBD_Init(&hUSB_Device, &MSC_Desc, 0U);
    printf("[WCID] USBD_Init returned %d\r\n", (int)st);
    if (st == USBD_OK)
    {
      printf("[WCID] calling USBD_RegisterClass...\r\n");
      st = USBD_RegisterClass(&hUSB_Device, USBD_WCID_STREAMING_CLASS);
      printf("[WCID] USBD_RegisterClass returned %d\r\n", (int)st);
      if (st == USBD_OK)
      {
        printf("[WCID] calling UsbWcidApp_Init...\r\n");
        UsbWcidApp_Init(&hUSB_Device);
        printf("[WCID] calling UsbWcidApp_SetCmdHandler...\r\n");
        UsbWcidApp_SetCmdHandler(WcidCmdCallback);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn); /* stack ready, re-enable USB ISR */
        printf("[WCID] calling USBD_Start...\r\n");
        st = USBD_Start(&hUSB_Device);
        printf("[WCID] USBD_Start returned %d\r\n", (int)st);
        if (st == USBD_OK)
        {
          printf("[WCID] init ok — 4 data IN (LSM/H3/MIC/QMA) + resp IN + 1 OUT (cmd)\r\n");
        }
        else { printf("[WCID] USBD_Start FAIL\r\n"); }
      }
      else { printf("[WCID] RegisterClass FAIL\r\n"); }
    }
    else { printf("[WCID] USBD_Init FAIL\r\n"); }

    /* WCID mode: WcidCmdCallback (ISR) sets s_wcid_cmd_pending.
     * Process commands here in task context where RTOS mutexes are safe. */
    /* USB connect management via PC7 (USB_DET, VBUS divider) — battery boot only.
     * USB-first boot: USBD_Start already connected D+ to the present host, leave
     * it. Battery boot: keep D+ disconnected until the cable is detected (PC7
     * HIGH), then connect so the host sees a clean insertion and enumerates.
     * This is the board's intended VBUS-detect design. */
    uint8_t  s_usb_present_prev = 0xFFU;   /* 0xFF forces initial sync */
    uint32_t s_usb_connect_tick = 0U;
    for (;;)
    {
      osDelay(20U);

      uint32_t now = osKernelGetTickCount();

      /* Battery boot: drive D+ connect/disconnect from the USB_DET pin (PC7). */
      if (BoardIO_IsBatteryLatched())
      {
        uint8_t usb_now = UsbDet_IsPresent();
        if (usb_now != s_usb_present_prev)
        {
          s_usb_present_prev = usb_now;
          if (usb_now != 0U)
          {
            HAL_PCD_DevConnect(&hpcd_USB_OTG_FS);    /* cable in: advertise */
            s_usb_connect_tick = now;
            printf("[USB] cable detected (PC7 HIGH) -> connect D+\r\n");
          }
          else
          {
            HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS); /* cable out: stop adv */
            printf("[USB] cable removed (PC7 LOW) -> disconnect D+\r\n");
          }
        }
        else if ((usb_now != 0U) &&
                 (hUSB_Device.dev_state != USBD_STATE_CONFIGURED) &&
                 ((int32_t)(now - (s_usb_connect_tick + 1500U)) >= 0))
        {
          /* Cable present but enumeration stalled: re-pulse D+ once. */
          HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
          osDelay(80U);
          HAL_PCD_DevConnect(&hpcd_USB_OTG_FS);
          s_usb_connect_tick = now;
          printf("[USB] re-advertise (PC7 HIGH, not configured)\r\n");
        }
      }

      if (s_usb_done_armed != 0U)
      {
        AppAcqControl_t acq;
        AppAcqGetCopy(&acq);
        if (acq.running == 0U)
        {
          s_usb_done_armed = 0U;
          osDelay(500U);
          UsbCdcService_Write((const uint8_t *)"DONE\r\n", 6U);
        }
      }

      if (s_wcid_cmd_pending != 0U)
      {
        char local_cmd[64];
        s_wcid_cmd_pending = 0U;
        memcpy(local_cmd, s_wcid_cmd_buf, sizeof(local_cmd));
        printf("[WCID] cmd: %s\r\n", local_cmd);
        UsbCmd_Process(local_cmd);
      }
    }
  }
}

void StartUsbUploadTask(void *argument)
{
  /* CDC removed — in WCID mode sensors write directly via UsbWcidApp_SendCsv. */
  (void)argument;
  for (;;) { osDelay(10000U); }
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
  /* en 即时生效:芯片一律初始化(无论 enabled),使能与否由主循环每轮门控。禁用的传感器
   * 开机也初始化(多一点功耗,可接受),这样停采时 "s lsm en 1" + acq_start 即可起采,不必
   * 重启。FIFO 清空/ODR 重配由 acq_start 与 s 命令时的 AppApplySensorConfig 负责。 */
  if (LSM6DSOX_Init() != HAL_OK)
  {
    printf("[LSM6DSOX] init failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }

  /* Configure LSM FIFO: ACC+GYRO batched at configured ODR, watermark
   * at 256 samples. INT1 (PB1) triggers EXTI when FIFO fill reaches
   * the watermark. */
  {
    AcqConfig_t acfg;
    AcqConfig_GetCopy(&acfg);
    uint8_t bdr = AppLsmBdrToEnum(acfg.lsm6dsox.odr_hz);
    s_lsm_odr_interval_us = AppLsmBdrToIntervalUs(bdr);
    printf("[LSM6DSOX] ODR config: %u Hz -> BDR=0x%02X (interval=%lu us)\r\n",
           (unsigned)acfg.lsm6dsox.odr_hz, (unsigned)bdr,
           (unsigned long)s_lsm_odr_interval_us);
    if (LSM6DSOX_FIFO_Config(256U, bdr, bdr) != HAL_OK)
    {
      printf("[LSM6DSOX] FIFO config failed, task exit\r\n");
      osThreadTerminate(NULL);
      return;
    }
  }

  /* Local FIFO read scratch: 256 words × 7 bytes */
  static uint8_t fifo_buf[256U * 7U];

  for (;;)
  {
    AppAcqCheckAutoStop();
    /* 门控 = 采集运行中 && 本传感器使能。enabled 每批(约38ms一次水位)读一次,
     * 不进 6664Hz 内层逐样本循环。改 en 停采时改、下次 acq_start 即生效。 */
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.lsm6dsox.enabled == 0U))
      {
        osDelay(APP_ACQ_IDLE_DELAY_MS);
        continue;
      }
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
     * Each chunk is parsed into CSV rows and written to the LSM_IMU ring
     * buffer for the logger to flush to SD. */
    int16_t last_acc[3] = {0}, last_gyr[3] = {0};
    uint8_t last_has_pair = 0;
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

      /* Two-pass parse:
       *   pass 1: count ACC/GYR pairs
       *   pass 2: emit one CSV row per pair into the LSM_IMU ring buffer.
       * Each row carries 6 raw int16 values (ACC + GYR). PC-side scales by
       * the sensitivity constants to recover physical units. */
      static uint32_t lsm_ts_us = 0U;
      static uint8_t lsm_ts_init = 0U;
      if (lsm_ts_init == 0U) { lsm_ts_us = AppDwtUs(); lsm_ts_init = 1U; }
      uint8_t cur_has_acc = 0, cur_has_gyr = 0;
      int16_t cur_acc[3] = {0}, cur_gyr[3] = {0};
      uint16_t n_pairs = 0;
      for (uint16_t i = 0; i < to_read; i++)
      {
        uint8_t tag_id = fifo_buf[i * 7U] >> 3;
        if (tag_id == LSM6DSOX_TAG_ACC_NC)
        {
          if (cur_has_gyr) { n_pairs++; cur_has_gyr = 0; } else cur_has_acc = 1;
        }
        else if (tag_id == LSM6DSOX_TAG_GYRO_NC)
        {
          if (cur_has_acc) { n_pairs++; cur_has_acc = 0; } else cur_has_gyr = 1;
        }
      }
      cur_has_acc = cur_has_gyr = 0;

      /* Fetch sensitivity once per FIFO batch — avoids 6000+/s mutex acquires. */
      AcqConfig_t lcfg;
      AcqConfig_GetCopy(&lcfg);
      float xl_s = AppLsmXlSensitivity(lcfg.lsm6dsox.range);
      float g_s  = AppLsmGyrSensitivity(lcfg.lsm6dsox.range2);

      /* Fetch timestamp once per batch: epoch_s is 1-second resolution,
       * same value for all ~256 samples. Avoids concurrent GetEpochUs()
       * calls from multiple tasks racing on s_dwt_prev / s_wrap_count. */
      uint32_t epoch_s_batch = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
      /* Format the datetime ONCE per FIFO batch — the epoch is 1-second
       * resolution so it is identical for every row in the batch, and the
       * epoch→Y/M/D/H/M/S conversion is the heaviest per-row op at 6664 Hz.
       * AppFmtDateTime12 always writes exactly 12 chars (YYMMDDHHMMSS). */
      char dt_batch[12];
      (void)AppFmtDateTime12(dt_batch, epoch_s_batch);

      char rowbuf[160];
      for (uint16_t i = 0; i < to_read; i++)
      {
        uint8_t *w = &fifo_buf[i * 7U];
        uint8_t tag_id = w[0] >> 3;
        int16_t rx = (int16_t)((uint16_t)w[2] << 8 | w[1]);
        int16_t ry = (int16_t)((uint16_t)w[4] << 8 | w[3]);
        int16_t rz = (int16_t)((uint16_t)w[6] << 8 | w[5]);
        if (tag_id == LSM6DSOX_TAG_ACC_NC) { cur_acc[0]=rx; cur_acc[1]=ry; cur_acc[2]=rz; cur_has_acc = 1; }
        else if (tag_id == LSM6DSOX_TAG_GYRO_NC) { cur_gyr[0]=rx; cur_gyr[1]=ry; cur_gyr[2]=rz; cur_has_gyr = 1; }
        else continue;

        if (cur_has_acc && cur_has_gyr)
        {
          lsm_ts_us += s_lsm_odr_interval_us;
          uint32_t fid = ++g_lsm_frame_id_counter;

          uint32_t off = 0;
          off += AppU32ToDec(&rowbuf[off], fid);
          rowbuf[off++] = ',';
          memcpy(&rowbuf[off], dt_batch, 12U); off += 12U;
          rowbuf[off++] = ',';
          off += AppF1ToDec(&rowbuf[off], (float)cur_acc[0] * xl_s);
          rowbuf[off++] = ',';
          off += AppF1ToDec(&rowbuf[off], (float)cur_acc[1] * xl_s);
          rowbuf[off++] = ',';
          off += AppF1ToDec(&rowbuf[off], (float)cur_acc[2] * xl_s);
          rowbuf[off++] = ',';
          off += AppF1ToDec(&rowbuf[off], (float)cur_gyr[0] * g_s);
          rowbuf[off++] = ',';
          off += AppF1ToDec(&rowbuf[off], (float)cur_gyr[1] * g_s);
          rowbuf[off++] = ',';
          off += AppF1ToDec(&rowbuf[off], (float)cur_gyr[2] * g_s);
          rowbuf[off++] = '\r';
          rowbuf[off++] = '\n';
          /* Guard: verify exactly 7 commas (8 columns) before writing.
           * ~0.5% of rows have missing/extra commas from an unknown cause;
           * silently dropping them is better than corrupting the CSV file. */
          {
            uint32_t nc = 0U;
            for (uint32_t k = 0U; k < off - 2U; k++) { if (rowbuf[k] == ',') nc++; }
            if (nc == 7U)
            {
              RingBuf_Write(&g_ring_lsm_imu, (const uint8_t *)rowbuf, off);
              /* WCID Bulk: write CSV directly to USB endpoint double-buffer. */
              if (g_boot_mode == BOOT_MODE_WCID_BULK && AppAcqIsUsbSinkActive() != 0U)
              {
                UsbWcidApp_SendCsv(WCID_CH_LSM_IMU, rowbuf, off);
              }
            }
          }

          last_acc[0] = cur_acc[0]; last_acc[1] = cur_acc[1]; last_acc[2] = cur_acc[2];
          last_gyr[0] = cur_gyr[0]; last_gyr[1] = cur_gyr[1]; last_gyr[2] = cur_gyr[2];
          last_has_pair = 1;

          cur_has_acc = 0;
          cur_has_gyr = 0;
        }
      }

      /* Stop once FIFO is back below the high-water mark; otherwise continue
       * reading the next chunk in this same iteration. */
      if (to_read < 256U) break;
    }

    /* Once per FIFO drain, update composite frame with LSM ACC+GYR+TEMP
     * and push a copy to the frame buffer for USB upload. */
    {
      AppSensorFrame_t tmp;
      uint32_t now_ms = osKernelGetTickCount();

      LSM6DSOX_AllData_t lsm_data;
      memset(&lsm_data, 0, sizeof(lsm_data));
      if (last_has_pair)
      {
        AcqConfig_t lcfg;
        AcqConfig_GetCopy(&lcfg);
        float xl_s = AppLsmXlSensitivity(lcfg.lsm6dsox.range);
        float g_s  = AppLsmGyrSensitivity(lcfg.lsm6dsox.range2);
        lsm_data.acc.x  = (float)last_acc[0] * xl_s;
        lsm_data.acc.y  = (float)last_acc[1] * xl_s;
        lsm_data.acc.z  = (float)last_acc[2] * xl_s;
        lsm_data.gyro.x = (float)last_gyr[0] * g_s;
        lsm_data.gyro.y = (float)last_gyr[1] * g_s;
        lsm_data.gyro.z = (float)last_gyr[2] * g_s;
      }
      (void)LSM6DSOX_ReadTemp(&lsm_data.temp_C);

      memset(&tmp, 0, sizeof(tmp));
      AppFramePopulateLsm6dsox(&tmp, &lsm_data, now_ms);

      {
        AppSensorFrame_t push_frame;
        osMutexAcquire(frame_buffer_mutex, osWaitForever);
        g_composite_frame.lsm6dsox = tmp.lsm6dsox;
        g_composite_frame.present_mask |= APP_SENSOR_MASK_LSM6DSOX;
        g_composite_frame.enabled_mask = APP_SENSOR_MASK_ALL;
        g_composite_frame.tick_ms = now_ms;
        g_composite_frame.frame_id = ++g_flow_stats.frame_id;
        push_frame = g_composite_frame;
        osMutexRelease(frame_buffer_mutex);
        (void)AppFrameBufferPush(&push_frame);
      }
    }
  }
#endif
}

/* Apply the configured H3 ODR to the chip's CTRL_REG1 DR bits and update the
 * DRDY-fallback interval. Init() hard-codes 400Hz, so without this the odr_hz
 * config was ignored entirely. Called once at boot, same as LSM/QMA configure
 * their chips at task start (ODR change needs a reboot to take effect).
 * Datasheet Table 20: DR 00/01/10 = 50/100/400 Hz. */
static void AppH3ApplyOdr(void)
{
  AcqConfig_t acfg;
  AcqConfig_GetCopy(&acfg);
  uint32_t h3_odr = (acfg.h3lis100dl.odr_hz > 0U) ? (uint32_t)acfg.h3lis100dl.odr_hz : 400U;
  H3LIS100DL_Config_t h3_cfg;
  if (h3_odr <= 75U)       { h3_cfg.odr = H3LIS100DL_ODR_50HZ;  h3_odr = 50U;  }
  else if (h3_odr <= 250U) { h3_cfg.odr = H3LIS100DL_ODR_100HZ; h3_odr = 100U; }
  else                     { h3_cfg.odr = H3LIS100DL_ODR_400HZ; h3_odr = 400U; }
  osMutexAcquire(spi2_mutex, osWaitForever);
  int h3_cfg_ret = H3LIS100DL_Configure(&h3_cfg);
  osMutexRelease(spi2_mutex);
  s_h3_odr_interval_us = 1000000U / h3_odr;
  printf("[H3LIS100DL] DRDY-IRQ mode (PA1/EXTI1), ODR %lu Hz (interval=%lu us, cfg_ret=%d)\r\n",
         (unsigned long)h3_odr, (unsigned long)s_h3_odr_interval_us, h3_cfg_ret);
}

void StartH3lis100dlTask(void *argument)
{
  (void)argument;

  osMutexAcquire(spi2_mutex, osWaitForever);
  int init_ret = H3LIS100DL_Init();
  osMutexRelease(spi2_mutex);
  if (init_ret != 0)
  {
    printf("[H3LIS100DL] init failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_H3LIS100DL
  printf("[H3LIS100DL TEST] started\r\n");
  for (;;)
  {
    H3LIS100DL_Data_t data;
    AppAcqCheckAutoStop();
    if (AppAcqIsRunning() == 0U) { osDelay(APP_ACQ_IDLE_DELAY_MS); continue; }
    osMutexAcquire(spi2_mutex, osWaitForever);
    int ret = H3LIS100DL_ReadAccXYZ(&data);
    osMutexRelease(spi2_mutex);
    if (ret == 0)
    {
      printf("H3LIS100DL: raw[%4d,%4d,%4d]  acc(mg)[%7.1f,%7.1f,%7.1f]\r\n",
             data.raw[0], data.raw[1], data.raw[2],
             data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);
    }
    osDelay(AppAcqCurrentPeriodMs());
  }
#else
  /* en 即时生效:H3 芯片已无条件 Init;不再因 disabled 永久 idle,使能由主循环每轮门控。 */

  /* DRDY-interrupt mode (test): DRDY routed to PA1/EXTI1 (CTRL_REG3=0x02), s_h3_drdy_sem
   * released by HAL_GPIO_EXTI_Rising_Callback. Was unreliable on the old board → polling;
   * HW reportedly fixed, re-testing. Timeout = period+10ms fallback so a missed/absent IRQ
   * still reads (data stays continuous); irq vs timeout counts tell if DRDY is reliable. */
  AppH3ApplyOdr();   /* 开机无条件应用 ODR(改 odr 在 acq_start 经 AppApplySensorConfig 重配) */
  for (;;)
  {
    AppAcqCheckAutoStop();
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.h3lis100dl.enabled == 0U))
      {
        osDelay(APP_ACQ_IDLE_DELAY_MS);
        continue;
      }
    }

    /* Wait for DRDY interrupt; fall back after period+10ms if the IRQ doesn't arrive. */
    {
      uint32_t h3_to = (s_h3_odr_interval_us / 1000U) + 10U;
      if (h3_to < 2U) { h3_to = 2U; }
      if ((s_h3_drdy_sem != NULL) && (osSemaphoreAcquire(s_h3_drdy_sem, h3_to) == osOK))
      { g_h3_irq_count++; }
      else { g_h3_timeout_count++; }
    }

    H3LIS100DL_Data_t data;
    osMutexAcquire(spi2_mutex, osWaitForever);
    int ret = H3LIS100DL_ReadAccXYZ(&data);
    osMutexRelease(spi2_mutex);

    if (ret == 0)
    {
      /* Monotonic timestamp: seed with DWT on first sample, then +interval per sample. */
      static uint32_t h3_ts_us = 0U;
      static uint8_t h3_ts_init = 0U;
      if (h3_ts_init == 0U) { h3_ts_us = AppDwtUs(); h3_ts_init = 1U; }
      h3_ts_us += s_h3_odr_interval_us;

      uint32_t fid = ++g_h3_frame_id_counter;
      uint32_t epoch_s_i = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
      char rowbuf[96];
      uint32_t off = 0;
      off += AppU32ToDec(&rowbuf[off], fid);
      rowbuf[off++] = ',';
      off += AppFmtDateTime12(&rowbuf[off], epoch_s_i);
      rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], data.acc_mg[0]);
      rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], data.acc_mg[1]);
      rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], data.acc_mg[2]);
      rowbuf[off++] = '\r';
      rowbuf[off++] = '\n';
      RingBuf_Write(&g_ring_h3_acc, (const uint8_t *)rowbuf, off);

      /* WCID Bulk: write CSV directly to USB endpoint double-buffer. */
      if (g_boot_mode == BOOT_MODE_WCID_BULK && AppAcqIsUsbSinkActive() != 0U)
      {
        UsbWcidApp_SendCsv(WCID_CH_H3_ACCEL, rowbuf, off);
      }

      if (AppAcqIsUsbSinkActive() != 0U)
      {
        AppSensorFrame_t tmp;
        uint32_t now_ms = osKernelGetTickCount();
        memset(&tmp, 0, sizeof(tmp));
        AppFramePopulateH3lis100dl(&tmp, &data, now_ms);

        AppSensorFrame_t push_frame;
        osMutexAcquire(frame_buffer_mutex, osWaitForever);
        g_composite_frame.h3lis100dl = tmp.h3lis100dl;
        g_composite_frame.present_mask |= APP_SENSOR_MASK_H3LIS100DL;
        g_composite_frame.enabled_mask = APP_SENSOR_MASK_ALL;
        g_composite_frame.tick_ms = now_ms;
        g_composite_frame.frame_id = ++g_flow_stats.frame_id;
        push_frame = g_composite_frame;
        osMutexRelease(frame_buffer_mutex);
        (void)AppFrameBufferPush(&push_frame);
      }
    }

    /* No pacing delay — loop is paced by the DRDY interrupt (or its timeout fallback). */
  }
#endif
}

void StartQma6100pTask(void *argument)
{
  (void)argument;

  osMutexAcquire(spi2_mutex, osWaitForever);
  if (QMA6100P_Init() != HAL_OK)
  {
    osMutexRelease(spi2_mutex);
    printf("[QMA6100P] init failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }

  /* Apply configured ODR (overrides the hardcoded 1600Hz in Init).
   * QMA6100P_Configure goes Standby→BW→Active so the ODR change takes effect. */
  {
    AcqConfig_t acfg;
    AcqConfig_GetCopy(&acfg);
    QMA6100P_Bandwidth_t bw = AppQmaBwToEnum((uint32_t)acfg.qma6100p.odr_hz);
    QMA6100P_Config_t qcfg = { .range = QMA6100P_RANGE_4G, .bw = bw };
    if (QMA6100P_Configure(&qcfg) == HAL_OK)
    {
      uint32_t actual_odr = AppQmaBwToOdrHz(bw);
      s_qma_odr_interval_us = 1000000U / actual_odr;
      printf("[QMA6100P] ODR configured: %lu Hz (interval=%lu us)\r\n",
             (unsigned long)actual_odr, (unsigned long)s_qma_odr_interval_us);
    }
  }
  osMutexRelease(spi2_mutex);

#if APP_SENSOR_TEST_TARGET == APP_SENSOR_TEST_QMA6100P
  printf("[QMA6100P TEST] started\r\n");
  for (;;)
  {
    QMA6100P_Data_t data;
    AppAcqCheckAutoStop();
    if (AppAcqIsRunning() == 0U) { osDelay(APP_ACQ_IDLE_DELAY_MS); continue; }
    osMutexAcquire(spi2_mutex, osWaitForever);
    HAL_StatusTypeDef ret = QMA6100P_ReadAccXYZ(&data);
    osMutexRelease(spi2_mutex);
    if (ret == HAL_OK)
    {
      printf("QMA6100P: acc(mg) X:%7.1f Y:%7.1f Z:%7.1f\r\n",
             data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);
    }
    osDelay(AppAcqCurrentPeriodMs());
  }
#else
  /* en 即时生效:QMA 芯片/ODR 已无条件 Init+Configure;FIFO 也无条件配置;使能由主循环每轮门控。 */

  /* FIFO + watermark 16 frames -> ~10ms cadence per IRQ. */
  osMutexAcquire(spi2_mutex, osWaitForever);
  if (QMA6100P_FIFO_Config(16U) != HAL_OK)
  {
    osMutexRelease(spi2_mutex);
    printf("[QMA6100P] FIFO config failed, task exit\r\n");
    osThreadTerminate(NULL);
    return;
  }
  osMutexRelease(spi2_mutex);

  static uint8_t fifo_buf[63U * 6U];
  uint32_t qma_ts_us = 0U;  /* running µs counter, strictly monotonic */
  uint8_t qma_ts_init = 0U;

  for (;;)
  {
    AppAcqCheckAutoStop();
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.qma6100p.enabled == 0U))
      {
        osDelay(APP_ACQ_IDLE_DELAY_MS);
        continue;
      }
    }

    /* Wait for the watermark EXTI, 2ms timeout for self-healing. NOTE: this
     * captures the full 1600 Hz of REAL frames (unique ≈ 1600) but the QMA6100P
     * FIFO_CNT over-reports, so ~31% of emitted rows are stale tail copies of
     * the last frame. Lengthening the timeout to read less often loses real
     * frames (unique dropped to ~1075 at 10ms) — worse. So keep the fast poll;
     * the duplicate tail is a QMA6100P quirk, filtered/ignored downstream. */
    if (s_qma_fifo_sem != NULL)
    {
      (void)osSemaphoreAcquire(s_qma_fifo_sem, 2U);
    }

    osMutexAcquire(spi2_mutex, osWaitForever);

    uint8_t fifo_level = 0;
    if (QMA6100P_FIFO_GetLevel(&fifo_level) != HAL_OK || fifo_level == 0U)
    {
      osMutexRelease(spi2_mutex);
      continue;
    }
    if (fifo_level > 63U) fifo_level = 63U;

    if (QMA6100P_FIFO_ReadBlock(fifo_buf, fifo_level) != HAL_OK)
    {
      osMutexRelease(spi2_mutex);
      continue;
    }

    /* STREAM mode: no Rearm needed. The FIFO is circular and re-triggers
     * the watermark interrupt automatically on wrap. */

    osMutexRelease(spi2_mutex);

    /* Strictly monotonic timestamps: each sample = previous + interval.
     * Seed with DWT on first batch; after that the counter never regresses.
     * The FIFO delivers oldest-first, so sample[0] is the oldest. */
    if (qma_ts_init == 0U)
    {
      qma_ts_us = AppDwtUs();
      qma_ts_init = 1U;
    }

    AcqConfig_t qcfg;
    AcqConfig_GetCopy(&qcfg);
    uint32_t qma_lsb1g;
    switch (qcfg.qma6100p.range)
    {
      case 4:  qma_lsb1g = 2048U; break;
      case 8:  qma_lsb1g = 1024U; break;
      case 16: qma_lsb1g = 512U;  break;
      case 32: qma_lsb1g = 256U;  break;
      default: qma_lsb1g = 4096U; break;
    }
    float qma_scale = 1000.0f / (float)qma_lsb1g;
    uint32_t epoch_s_batch_qma = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
    /* Datetime formatted once per batch (1-second resolution) — see LSM. */
    char dt_batch_qma[12];
    (void)AppFmtDateTime12(dt_batch_qma, epoch_s_batch_qma);

    char rowbuf[96];
    for (uint8_t i = 0; i < fifo_level; i++)
    {
      uint8_t *p = &fifo_buf[i * 6U];
      int16_t rx = (int16_t)(((int16_t)((uint16_t)p[1] << 8 | p[0])) >> 2);
      int16_t ry = (int16_t)(((int16_t)((uint16_t)p[3] << 8 | p[2])) >> 2);
      int16_t rz = (int16_t)(((int16_t)((uint16_t)p[5] << 8 | p[4])) >> 2);

      /* Drop FIFO over-read duplicates: the QMA6100P FIFO read does not advance
       * its read pointer correctly, so each read re-returns the last frame(s)
       * verbatim (~35-44% byte-identical stale rows, confirmed by before/after
       * FIFO_CNT diag). They cost CPU/SD/storage and drag LSM. Under vibration
       * the real samples always differ (0.625ms @ ±4g), so a byte-identical
       * frame is a stale repeat → skip it before formatting. Cost: during true
       * standstill (no vibration = no signal) the stream becomes change-only. */
      if (s_qma_prev_valid && rx == s_qma_prev_x && ry == s_qma_prev_y && rz == s_qma_prev_z)
      {
        continue;
      }
      s_qma_prev_x = rx; s_qma_prev_y = ry; s_qma_prev_z = rz; s_qma_prev_valid = 1U;

      qma_ts_us += s_qma_odr_interval_us;
      uint32_t fid = ++g_qma_frame_id_counter;

      uint32_t off = 0;
      off += AppU32ToDec(&rowbuf[off], fid);
      rowbuf[off++] = ',';
      memcpy(&rowbuf[off], dt_batch_qma, 12U); off += 12U;
      rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], (float)rx * qma_scale);
      rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], (float)ry * qma_scale);
      rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], (float)rz * qma_scale);
      rowbuf[off++] = '\r';
      rowbuf[off++] = '\n';
      RingBuf_Write(&g_ring_qma_acc, (const uint8_t *)rowbuf, off);

      /* WCID Bulk: write CSV directly to USB endpoint double-buffer. */
      if (g_boot_mode == BOOT_MODE_WCID_BULK && AppAcqIsUsbSinkActive() != 0U)
      {
        UsbWcidApp_SendCsv(WCID_CH_QMA_ACCEL, rowbuf, off);
      }
    }

    if (AppAcqIsUsbSinkActive() != 0U && fifo_level > 0U)
    {
      uint8_t *last = &fifo_buf[(fifo_level - 1U) * 6U];
      int16_t lrx = (int16_t)(((int16_t)((uint16_t)last[1] << 8 | last[0])) >> 2);
      int16_t lry = (int16_t)(((int16_t)((uint16_t)last[3] << 8 | last[2])) >> 2);
      int16_t lrz = (int16_t)(((int16_t)((uint16_t)last[5] << 8 | last[4])) >> 2);

      AcqConfig_t qcfg;
      AcqConfig_GetCopy(&qcfg);
      uint32_t qma_lsb1g;
      switch (qcfg.qma6100p.range)
      {
        case 4:  qma_lsb1g = 2048U; break;
        case 8:  qma_lsb1g = 1024U; break;
        case 16: qma_lsb1g = 512U;  break;
        case 32: qma_lsb1g = 256U;  break;
        default: qma_lsb1g = 4096U; break;
      }
      float qma_scale = 1000.0f / (float)qma_lsb1g;

      QMA6100P_Data_t qdata;
      qdata.raw[0] = lrx; qdata.raw[1] = lry; qdata.raw[2] = lrz;
      qdata.acc_mg[0] = (float)lrx * qma_scale;
      qdata.acc_mg[1] = (float)lry * qma_scale;
      qdata.acc_mg[2] = (float)lrz * qma_scale;

      AppSensorFrame_t tmp;
      uint32_t now_ms = osKernelGetTickCount();
      memset(&tmp, 0, sizeof(tmp));
      AppFramePopulateQma6100p(&tmp, &qdata, now_ms);

      {
        AppSensorFrame_t push_frame;
        osMutexAcquire(frame_buffer_mutex, osWaitForever);
        g_composite_frame.qma6100p = tmp.qma6100p;
        g_composite_frame.present_mask |= APP_SENSOR_MASK_QMA6100P;
        g_composite_frame.enabled_mask = APP_SENSOR_MASK_ALL;
        g_composite_frame.tick_ms = now_ms;
        g_composite_frame.frame_id = ++g_flow_stats.frame_id;
        push_frame = g_composite_frame;
        osMutexRelease(frame_buffer_mutex);
        (void)AppFrameBufferPush(&push_frame);
      }
    }
  }
#endif
}

/* ===== AHT20 温湿度任务（I2C1，~1Hz，仅 SD ring） ===== */
void StartAht20Task(void *argument)
{
  (void)argument;

  /* en 即时生效:AHT20 一律初始化(无论 enabled);使能由主循环每轮门控。 */
  osMutexAcquire(i2c1_mutex, osWaitForever);
  HAL_StatusTypeDef ir = AHT20_Init();
  osMutexRelease(i2c1_mutex);
  if (ir != HAL_OK)
  {
    printf("[AHT20] init failed, task idle\r\n");
    for (;;) { osDelay(1000U); }
  }

  char rowbuf[48];
  for (;;)
  {
    AppAcqCheckAutoStop();
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.aht20.enabled == 0U)) { osDelay(APP_ACQ_IDLE_DELAY_MS); continue; }
    }

    osMutexAcquire(i2c1_mutex, osWaitForever);
    HAL_StatusTypeDef tr = AHT20_TriggerMeasure();
    osMutexRelease(i2c1_mutex);
    if (tr != HAL_OK) { osDelay(1000U); continue; }

    osDelay(80U);   /* 手册要求测量等待 ≥80ms */

    float temp_c = 0.0f, hum = 0.0f;
    osMutexAcquire(i2c1_mutex, osWaitForever);
    HAL_StatusTypeDef rr = AHT20_ReadResult(&temp_c, &hum);
    osMutexRelease(i2c1_mutex);
    if (rr != HAL_OK) { osDelay(1000U); continue; }

    uint32_t epoch_s = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
    char dt[12];
    (void)AppFmtDateTime12(dt, epoch_s);
    uint32_t fid = ++g_aht_frame_id_counter;

    uint32_t off = 0;
    off += AppU32ToDec(&rowbuf[off], fid);          rowbuf[off++] = ',';
    memcpy(&rowbuf[off], dt, 12U); off += 12U;       rowbuf[off++] = ',';
    off += AppF1ToDec(&rowbuf[off], temp_c);         rowbuf[off++] = ',';
    off += AppF1ToDec(&rowbuf[off], hum);
    rowbuf[off++] = '\r'; rowbuf[off++] = '\n';
    RingBuf_Write(&g_ring_aht_env, (const uint8_t *)rowbuf, off);

    /* WCID Bulk: AHT 数据没有独立 IN 端点，复用命令响应端点 0x85，行首加 "aht,"
     * 供上位机在该端点上与命令响应文本分流。整行一次发出（≤64B，不截断）。 */
    if (g_boot_mode == BOOT_MODE_WCID_BULK && AppAcqIsUsbSinkActive() != 0U)
    {
      char usbrow[56];
      usbrow[0] = 'a'; usbrow[1] = 'h'; usbrow[2] = 't'; usbrow[3] = ',';
      memcpy(&usbrow[4], rowbuf, off);
      (void)UsbWcidApp_Write((const uint8_t *)usbrow, off + 4U);
    }

    osDelay(1000U);   /* ~1Hz（手册要求采集周期≥1s） */
  }
}

/* ===== LIS2MDL 三轴磁力任务（I2C1，100Hz，DRDY/EXTI13 节拍，仅 SD ring） ===== */
void StartLis2mdlTask(void *argument)
{
  (void)argument;

  /* en 即时生效:LIS2MDL 一律初始化(无论 enabled);使能由主循环每轮门控。开机按当前
   * 配置 odr 初始化(改 odr 在 acq_start 经 AppApplySensorConfig 重配)。 */
  uint16_t odr;
  {
    AcqConfig_t acfg;
    AcqConfig_GetCopy(&acfg);
    odr = acfg.lis2mdl.odr_hz;
  }

  osMutexAcquire(i2c1_mutex, osWaitForever);
  HAL_StatusTypeDef ir = LIS2MDL_Init(odr);
  osMutexRelease(i2c1_mutex);
  if (ir != HAL_OK)
  {
    printf("[LIS2MDL] init failed, task idle\r\n");
    for (;;) { osDelay(1000U); }
  }

  /* rowbuf 容量：fid(≤10) + ',' + dt(12) + ',' + 3×mg(±50000mG→≤8 "-50000.0") + 2×','
   * + "\r\n" = 最坏 52 字节，取 64 留余量（QMA 同类用 96）。 */
  char rowbuf[64];
  char dt[12];
  uint8_t  dt_valid = 0U;
  uint32_t dt_last_tick = 0U;

  for (;;)
  {
    AppAcqCheckAutoStop();
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.lis2mdl.enabled == 0U)) { dt_valid = 0U; osDelay(APP_ACQ_IDLE_DELAY_MS); continue; }
    }

    /* DRDY 硬件节拍（不靠 tick 计时）；20ms 超时自愈。 */
    osStatus_t sem_st = osOK;
    if (s_mag_drdy_sem != NULL)
    {
      sem_st = osSemaphoreAcquire(s_mag_drdy_sem, 20U);
    }

    int16_t raw[3]; float mg[3];
    HAL_StatusTypeDef rr;
    osMutexAcquire(i2c1_mutex, osWaitForever);
    if (sem_st == osOK)
    {
      /* DRDY 中断已确保新数据就绪：跳过冗余的 STATUS 读，直接读 6 字节，
       * 每周期省一次 I2C 往返(~0.3ms)，降低满载下错过 10ms 窗口的概率。
       * (BDU 已置位，保证 H/L 一致，无需再查 Zyxda。) */
      rr = LIS2MDL_ReadMag(raw, mg);
    }
    else
    {
      /* 超时回退(无 DRDY)：先查就绪再读，避免读到陈旧/重复数据。 */
      rr = LIS2MDL_DataReady() ? LIS2MDL_ReadMag(raw, mg) : HAL_BUSY;
    }
    osMutexRelease(i2c1_mutex);
    if (rr != HAL_OK) { continue; }

    /* datetime 限流：AppTime_GetEpochUs 不宜 100Hz 调用；分辨率本就 1s，
     * 每 ~0.5s（500 tick）刷新一次，期间复用缓存字符串。 */
    uint32_t now_tick = osKernelGetTickCount();
    if (dt_valid == 0U || (now_tick - dt_last_tick) >= 500U)
    {
      uint32_t epoch_s = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
      (void)AppFmtDateTime12(dt, epoch_s);
      dt_last_tick = now_tick;
      dt_valid = 1U;
    }

    uint32_t fid = ++g_mag_frame_id_counter;
    uint32_t off = 0;
    off += AppU32ToDec(&rowbuf[off], fid);     rowbuf[off++] = ',';
    memcpy(&rowbuf[off], dt, 12U); off += 12U;  rowbuf[off++] = ',';
    off += AppF1ToDec(&rowbuf[off], mg[0]);    rowbuf[off++] = ',';
    off += AppF1ToDec(&rowbuf[off], mg[1]);    rowbuf[off++] = ',';
    off += AppF1ToDec(&rowbuf[off], mg[2]);
    rowbuf[off++] = '\r'; rowbuf[off++] = '\n';
    RingBuf_Write(&g_ring_mag, (const uint8_t *)rowbuf, off);

    /* WCID Bulk: MAG 数据没有独立 IN 端点，复用命令响应端点 0x85，行首加 "mag,"
     * 供上位机分流。整行 ≤56B，单次发出不截断。100Hz × ~56B ≈ 5.6KB/s，对 0x85
     * 可忽略；与命令响应/AHT 的并发由 UsbWcidApp_Write 内的互斥锁串行化。 */
    if (g_boot_mode == BOOT_MODE_WCID_BULK && AppAcqIsUsbSinkActive() != 0U)
    {
      char usbrow[72];
      usbrow[0] = 'm'; usbrow[1] = 'a'; usbrow[2] = 'g'; usbrow[3] = ',';
      memcpy(&usbrow[4], rowbuf, off);
      (void)UsbWcidApp_Write((const uint8_t *)usbrow, off + 4U);
    }
  }
}

/* Mic supervisor: starts/stops the ES8311 + SAI DMA capture in lock-step with
 * the global acquisition state when the codec is enabled in config. Audio data
 * itself is pushed to g_ring_mic by the SAI DMA ISR; this task only handles the
 * start/stop edges. */
void StartMicTask(void *argument)
{
  (void)argument;
  uint8_t running = 0U;
  for (;;)
  {
    uint8_t want = (uint8_t)(AppAcqIsRunning() != 0U);
    AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
    want = (uint8_t)(want && cfg.es8311.enabled);

    if (want && !running)
    {
      if (Mic_Start(cfg.es8311.sample_rate_hz, cfg.es8311.gain_db) == 0)
      {
        running = 1U;
        s_mic_capturing = 1U;   /* SAI DMA 已开录 → 诚实采集灯此刻才可亮 */
        printf("[Mic] started %luHz gain%u\r\n",
               (unsigned long)cfg.es8311.sample_rate_hz, (unsigned)cfg.es8311.gain_db);
      }
      else
      {
        printf("[Mic] start failed\r\n");
        osDelay(500U);
      }
    }
    else if (!want && running)
    {
      Mic_Stop();
      running = 0U;
      s_mic_capturing = 0U;
      printf("[Mic] stopped (dropped=%lu)\r\n", (unsigned long)Mic_GetDropped());
    }

    /* USB streaming mode: pump raw PCM from g_ring_mic into the USB MIC channel
     * (EP3). In SD mode the logger drains the ring to MIC.WAV instead, so we only
     * consume here when the USB sink is active — keeps the ring single-consumer.
     * Feed in small chunks (<= the SOF half) and poll fast so the 2x2048 double-
     * buffer never overruns the in-flight half; the 64KB ring absorbs jitter. */
    if (running && (AppAcqIsUsbSinkActive() != 0U))
    {
      uint32_t fed = 0U;
      const uint8_t *p;
      uint32_t n;
      while ((fed < 1024U) && (RingBuf_PeekContiguous(&g_ring_mic, &p, &n) != 0U))
      {
        uint32_t chunk = (n > 512U) ? 512U : n;
        if (UsbWcidApp_SendRaw(WCID_CH_MIC, p, chunk) != 0U) { break; }
        RingBuf_Consume(&g_ring_mic, chunk);
        fed += chunk;
      }
      osDelay(1U);
    }
    else
    {
      osDelay(20U);
    }
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

  /* Switch SDMMC to DMA mode now that the kernel is running and the
   * completion semaphore is operational.  Pre-kernel SD operations
   * (DeviceCfg_LoadFromSD) used polling mode via g_sd_use_dma == 0. */
  hsd1.Context = SD_CONTEXT_NONE;
  hsd1.Instance->IDMACTRL = 0U;
  __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_FLAGS);
  /* SD writes use POLLING (PIO), NOT SDMMC IDMA — this is the fix for the
   * long-standing ~0.4% byte-level CSV corruption. Root cause (confirmed by
   * A/B test CKBX0079 DMA=0.40% bad vs CKBX0080/0081 polling=0.00% bad): the
   * SDMMC IDMA, as a second bus master reading SRAM, corrupts the CPU's
   * concurrent writes to the ring buffers (a bus-matrix hazard the SD CRC can't
   * catch — the damage is in source RAM, not on the SD bus). Polling has no
   * IDMA, so no hazard. Full-load test (96k mic + 3 sensors) stayed 100% clean
   * with zero ring drops, so the PIO path sustains throughput here.
   * NOTE: the polling write path masks IRQs for the transfer + PROGRAMMING wait
   * (see SD_disk_write); fine with this card/clock, watch it on slower cards. */
  SD_SetDmaMode(0U);
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  printf("[Logger] SDMMC POLLING mode (corruption-free SD writes)\r\n");

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
      if (sd_file_open != 0U)
      {
        char done_dir[48];
        const char *dir = FatFs_SD_GetSessionDir();
        strncpy(done_dir, (dir != NULL) ? dir : "?", sizeof(done_dir) - 1U);
        done_dir[sizeof(done_dir) - 1U] = '\0';
        AppFlowStatsSetMode(AppAcqIsUsbSinkActive(), 0U);
        AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
        { char _d[64]; int _n = snprintf(_d, sizeof(_d), "DONE dir=%s\r\n", done_dir); \
          UsbWcidApp_Write((const uint8_t*)_d, (uint32_t)_n); }
      }
      else
      {
        AppFlowStatsSetMode(AppAcqIsUsbSinkActive(), 0U);
      }
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

        /* If the mic is enabled, create the MIC.WAV file (placeholder header,
         * size patched on stop inside FatFs_SD_LoggerStop). */
        {
          AcqConfig_t mcfg; AcqConfig_GetCopy(&mcfg);
          if (mcfg.es8311.enabled)
          {
            FRESULT wav_r = FatFs_SD_WavCreate(mcfg.es8311.sample_rate_hz, mcfg.es8311.bits);
            if (wav_r != FR_OK)
            {
              printf("[Logger] MIC.WAV create failed: %s (%d) — audio will be dropped\r\n",
                     FatFs_SD_ResultToString(wav_r), (int)wav_r);
            }
          }
        }

        /* Discard data buffered between sessions so each file starts clean
         * and frame_id/tick_ms in the new session align with real samples. */
        RingBuf_Reset(&g_ring_lsm_imu);
        RingBuf_Reset(&g_ring_qma_acc);
        RingBuf_Reset(&g_ring_h3_acc);
        RingBuf_Reset(&g_ring_mic);
        RingBuf_Reset(&g_ring_aht_env);
        RingBuf_Reset(&g_ring_mag);
        g_lsm_frame_id_counter = 0U;
        g_qma_frame_id_counter = 0U;
        g_h3_frame_id_counter  = 0U;
        g_aht_frame_id_counter = 0U;
        g_mag_frame_id_counter = 0U;
        g_h3_irq_count = 0U;        /* H3 DRDY 中断测试统计，按会话清零 */
        g_h3_timeout_count = 0U;

        sd_file_open = 1U;
        rows_since_sync = 0U;
        SD_ResetWriteStats();   /* route-2: count SDMMC write events per session */
        s_session_start_us = AppTime_GetEpochUs();  /* route-2: real-ODR measurement base */
        s_qma_prev_valid = 0U;  /* reset QMA dedup state for the new session */
        AppAcqResetSessionTimer();
        AppFlowStatsSetMode(0U, 1U);
        printf("[Logger] SD logging resumed\r\n");
      }
      else
      {
        printf("[Logger] SD start failed: %s (%d)%s\r\n",
               FatFs_SD_ResultToString(result),
               (int)result,
               (result == FR_NOT_READY) ? " — SD card must be inserted before power-on" : "");
        AppFlowStatsSetMode(0U, 0U);
        osDelay(LOGGER_RETRY_DELAY_MS);
        continue;
      }
    }

    /* Round-robin drain of the four ring buffers. Each chunk is copied into the
     * private s_sd_bounce buffer (inside LoggerDrainRing) before f_write, so the
     * SDMMC IDMA never reads from a live ring — see the helper's comment. Every
     * ring is checked each cycle so no stream starves under heavy load. */
    {
      uint8_t did_work = 1;
      while (did_work && (sd_file_open != 0U) && (AppAcqIsSdSessionActive() != 0U))
      {
        int dr;
        did_work = 0;

        dr = LoggerDrainRing(&g_ring_lsm_imu, 0U, APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
        if (dr < 0)
        {
          printf("[Logger] LSM_IMU write fail %s (%d)\r\n",
                 FatFs_SD_ResultToString(result), (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        if (dr > 0) did_work = 1;

        dr = LoggerDrainRing(&g_ring_qma_acc, 3U, APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
        if (dr < 0)
        {
          printf("[Logger] QMA_ACC write fail %s (%d)\r\n",
                 FatFs_SD_ResultToString(result), (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        if (dr > 0) did_work = 1;

        dr = LoggerDrainRing(&g_ring_h3_acc, 2U, APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
        if (dr < 0)
        {
          printf("[Logger] H3_ACC write fail %s (%d)\r\n",
                 FatFs_SD_ResultToString(result), (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        if (dr > 0) did_work = 1;

        dr = LoggerDrainRing(&g_ring_mic, FATFS_SD_FILE_MIC_WAV, APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
        if (dr < 0)
        {
          printf("[Logger] MIC.WAV write fail %s (%d)\r\n",
                 FatFs_SD_ResultToString(result), (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        if (dr > 0) did_work = 1;

        dr = LoggerDrainRing(&g_ring_aht_env, 4U, APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
        if (dr < 0)
        {
          printf("[Logger] AHT_ENV write fail %s (%d)\r\n",
                 FatFs_SD_ResultToString(result), (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        if (dr > 0) did_work = 1;

        dr = LoggerDrainRing(&g_ring_mag, 5U, APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
        if (dr < 0)
        {
          printf("[Logger] MAG write fail %s (%d)\r\n",
                 FatFs_SD_ResultToString(result), (int)result);
          AppFlowStatsRecordWriteFailure();
          break;
        }
        if (dr > 0) did_work = 1;
      }

      /* 攒批节奏：本轮已把所有达阈值(或半满)的 ring 写完，其余在攒批。让出 CPU 并
       * 给 ring 时间积累成大块，把每次写的固定开销(命令+PROGRAMMING+关中断)摊薄；
       * 同时避免 logger 空转占满自己的时间片。数轮 2ms 即可攒到 ~16KB，ring 半满
       * override 保证不溢出。会话停止时由 AppLoggerStopSdSession 强制 flush 尾部。 */
      if ((sd_file_open != 0U) && (AppAcqIsSdSessionActive() != 0U))
      {
        osDelay(2U);
      }
    }

    /* Periodic metadata flush: commit FAT + directory entries every ~1 MB of
     * payload so an extreme-load write-integrity failure (the 6664+96k FAT/dir
     * loss — empty session dir) can't orphan the whole session. rows_since_sync
     * accumulates drained bytes (LoggerDrainRing += n); ~1 MB ≈ 2.3 s at full
     * load, rare enough not to add meaningful write overhead. */
    if ((sd_file_open != 0U) && (rows_since_sync >= (1024U * 1024U)))
    {
      (void)FatFs_SD_LoggerSync();
      rows_since_sync = 0U;
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

      {
        char done_dir[48];
        const char *dir = FatFs_SD_GetSessionDir();
        strncpy(done_dir, (dir != NULL) ? dir : "?", sizeof(done_dir) - 1U);
        done_dir[sizeof(done_dir) - 1U] = '\0';
        AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
        AppFlowStatsSetMode(AppAcqIsUsbSinkActive(), 0U);
        { char _d[64]; int _n = snprintf(_d, sizeof(_d), "DONE dir=%s\r\n", done_dir); \
          UsbWcidApp_Write((const uint8_t*)_d, (uint32_t)_n); }
      }
      osDelay(10U);
      continue;
    }

    if ((sd_file_open != 0U) && (AppAcqIsSdSessionActive() == 0U))
    {
      char done_dir[48];
      const char *dir = FatFs_SD_GetSessionDir();
      strncpy(done_dir, (dir != NULL) ? dir : "?", sizeof(done_dir) - 1U);
      done_dir[sizeof(done_dir) - 1U] = '\0';
      AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
      AppFlowStatsSetMode(AppAcqIsUsbSinkActive(), 0U);
      printf("DONE dir=%s\r\n", done_dir);
      osDelay(10U);
      continue;
    }

    AppFlowStatsSetMode(0U, (uint8_t)(sd_file_open != 0U));
    osDelay(10U);
  }
}
/* configASSERT() failure handler (declared extern in FreeRTOSConfig.h). The
 * macro has already disabled interrupts; reset the MCU so a failed assertion
 * self-recovers instead of freezing forever. Kept tiny and printf-free because
 * it may run from any context with interrupts masked. */
void AppAssertReset(void)
{
  NVIC_SystemReset();
  for (;;) {}
}

/* FreeRTOS stack-overflow hook — called when configCHECK_FOR_STACK_OVERFLOW≥1
 * detects a task has overrun its stack. Prints the offending task name and
 * halts so the fault is visible on the debug console. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  printf("[FATAL] stack overflow in task: %s\r\n", pcTaskName);
  taskDISABLE_INTERRUPTS();
  /* Self-recover instead of freezing forever — restart the system after the
   * offending task name has been emitted. Pairs with the planned IWDG watchdog. */
  NVIC_SystemReset();
  for (;;) {}
}
/* USER CODE END Application */
