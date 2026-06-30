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
#include "sensor_bin.h"
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
#include "app_acq.h"
#include "user_ctrl.h"
#include "board_io.h"
#include "app_time.h"
#include "app_locks.h"
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

#define APP_ACQ_IDLE_DELAY_MS    10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint8_t g_system_error = 0U;  /* 1 = 系统异常(SD写失败/传感器初始化失败),LED 常亮告警 */

/* ======================== Bus architecture ================================
 *
 *   SPI1 (PA5/PA6/PA7 + PC4 CS)          -> LSM6DSOX   (dedicated bus)
 *   SPI2 (PB10/PC2/PC1 + PC5/PA4 CS)     -> H3LIS100DL + QMA6100P (shared)
 */

static osMutexId_t spi2_mutex;
static osMutexId_t snapshot_mutex;
static osMutexId_t frame_buffer_mutex;
static osMutexId_t acq_ctrl_mutex;
SemaphoreHandle_t s_sdmmc_dma_sem;  /* signaled by HAL SD DMA completion ISR */
static osSemaphoreId_t s_lsm_fifo_sem;  /* released by EXTI0 ISR on PB0 rising edge (HW-v2) */
static osSemaphoreId_t s_qma_fifo_sem;  /* released by EXTI4 ISR on PC4 rising edge (HW-v2) */
static osSemaphoreId_t s_h3_drdy_sem;   /* released by EXTI1 ISR on PA1 rising edge (HW-v2) */
static osSemaphoreId_t s_mag_drdy_sem;  /* released by EXTI13 ISR on PC13 rising edge (LIS2MDL DRDY) */
static osMutexId_t     i2c1_mutex;      /* AHT20 + LIS2MDL 共享 I2C1 互斥 */
static osMutexId_t     i2c2_mutex;      /* ES8311 codec + PCF85063 RTC 共享 I2C2 互斥 (M1) */
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
/* SPSC ring: single producer = SAI DMA ISR (AppRing_WriteMic); single ACTIVE
 * consumer at a time, guaranteed by sink mutual-exclusion (USB stream XOR SD
 * logger drain — never both at once). Do NOT add a second concurrent consumer
 * without a real lock: PeekContiguous/Consume vs LoggerDrainRing would race the
 * SPSC indices and corrupt audio (L9). 非 static：mic_capture.c extern 引用。 */
AppRingBuffer_t g_ring_mic;
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
static const osMutexAttr_t i2c2_mutex_attr = {
  .name      = "i2c2Mutex",
  .attr_bits = osMutexPrioInherit,
};

/* USER CODE END Variables */

/* ======================== Sensor threads ================================== */
volatile osThreadId_t lsm6dsoxTaskHandle;   /* volatile: nulled by the task on init-fail, read by diag (L1) */
const osThreadAttr_t lsm6dsoxTask_attributes = {
  .name = "lsm6dsoxTask",
  /* High（> logger 的 AboveNormal）：6664Hz 硬件 FIFO 仅 ~38ms 余量，必须能抢占
   * logger 排空 FIFO，否则 FIFO 层丢 ~2.5%。早期单独提优先级失败(ring 溢出)是因
   * 当时 ring 小;现在 192KB ring + 攒批写、ring 峰值才 19%，足以兜住 logger 被
   * 抢占后的 drain 延迟。LSM 仅 ~7% CPU，不会饿死 logger。 */
  .priority = (osPriority_t)osPriorityHigh,
  .stack_size = 2048 * 4  /* 8KB — holds fifo_buf (1.8KB) + locals */
};

volatile osThreadId_t qma6100pTaskHandle;   /* volatile: nulled by the task on init-fail, read by diag (L1) */
const osThreadAttr_t qma6100pTask_attributes = {
  .name = "qma6100pTask",
  .priority = (osPriority_t)osPriorityAboveNormal,
  .stack_size = 2048 * 4  /* 8KB — fifo_buf(static)+rowbuf+局部变量；4KB曾导致栈溢出破坏frame_id计数器 */
};

volatile osThreadId_t h3lis100dlTaskHandle;   /* volatile: nulled by the task on init-fail, read by diag (L1) */
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
  .stack_size = 1024 * 2  /* 2KB — codec/SAI start-stop supervisor; raised from 1KB for printf + Mic_Start deep call-chain margin (M3) */
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
static void     AppEvtH3Sample(float mx, float my, float mz);  /* 阈值检测:定义在 StartLoggerTask 前,调用在 H3 支路(更靠前),需前置声明 */
static uint8_t  AppEvtThresholdMode(void);  /* 同上,供 AppAcqCheckAutoStop/AppCaptureActive 提前调用 */
static uint8_t  AppEvtIsRecording(void);
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
static uint8_t AppAcqIsSdSessionActive(void);
static void AppFlowStatsSetMode(uint8_t usb_active, uint8_t sd_active);
static void AppLoggerStopSdSession(uint8_t *sd_file_open, uint32_t *rows_since_sync);
static void AppAcqStopInternal(uint32_t now_ms);
static void AppAcqCheckAutoStop(void);
static void AppAcqResetSessionTimer(void);
uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms);
uint32_t AppAcqStop(void);
static uint32_t AppAcqDrainPendingStop(void);


void StartLsm6dsoxTask(void *argument);
void StartQma6100pTask(void *argument);
void StartH3lis100dlTask(void *argument);
void StartLoggerTask(void *argument);
void StartMicTask(void *argument);
void StartAht20Task(void *argument);
void StartLis2mdlTask(void *argument);

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
  i2c2_mutex     = osMutexNew(&i2c2_mutex_attr);
  /* Always create spi2_mutex (was guarded by a test-target #if). The QMA/H3
   * SPI2 acquire sites have no NULL guard, so unconditional creation avoids a
   * silent osMutexAcquire(NULL) = no protection in any build (L3). */
  spi2_mutex = osMutexNew(&spi2_mutex_attr);
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
  printf("[RTOS] lsm6dsoxTask created: %s\r\n", (lsm6dsoxTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] h3lis100dlTask created: %s\r\n", (h3lis100dlTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] qma6100pTask created: %s\r\n", (qma6100pTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] loggerTask created: %s\r\n", (loggerTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] micTask created: %s\r\n", (micTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] aht20Task created: %s\r\n", (aht20TaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] lis2mdlTask created: %s\r\n", (lis2mdlTaskHandle != NULL) ? "ok" : "FAILED");
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
  if (osThreadNew(StartUserCtrlTask, NULL, &userCtrlTask_attributes) == NULL)
  {
    printf("[RTOS] userCtrlTask created: FAILED\r\n");
  }
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
  /* 阈值模式:只有真正在录制事件(REC)时才算"采集中"。布防/死区返回 0 → LED 灭,
   * 这样灯灭=守候、灯闪=正在录事件,肉眼可区分。 */
  if (AppEvtThresholdMode() != 0U) { return AppEvtIsRecording(); }
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
  printf("[Stack] lsm=%lu h3=%lu qma=%lu log=%lu mic=%lu (min free bytes)\r\n",
         (unsigned long)osThreadGetStackSpace(lsm6dsoxTaskHandle),
         (unsigned long)osThreadGetStackSpace(h3lis100dlTaskHandle),
         (unsigned long)osThreadGetStackSpace(qma6100pTaskHandle),
         (unsigned long)osThreadGetStackSpace(loggerTaskHandle),
         (unsigned long)osThreadGetStackSpace(micTaskHandle));
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
  /* 阈值模式:不按 duration 自动停(每个事件会重置会话计时器,duration 永不到点;
   * 且语义是"布防守候到手动停/断电")。事件录制时长由 trig_post_sec 控制。 */
  if (AppEvtThresholdMode() != 0U)
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

void AppBootAcquireIfConfigured(void)
{
  AcqConfig_t cfg;
  uint8_t     sink;

  AcqConfig_GetCopy(&cfg);
  if (cfg.boot_acquire == 0U)
  {
    return;            /* 未开启开机即采，保持现有行为 */
  }
  /* 仅电池上电（长按 2s 锁存电池供电）才开机即采；USB 上电不触发。 */
  if (BoardIO_IsBatteryLatched() == 0U)
  {
    printf("[Acq] boot_acquire: USB 上电，跳过开机即采（仅电池上电触发）\r\n");
    return;
  }
  if (AppAcqIsRunning() != 0U)
  {
    return;            /* 已在采集（不应发生），不重复启动 */
  }

  /* sink：配置含 SD 位优先走 SD（无人值守标准用法）；否则走 USB。
   * BOTH 含 SD 位 → 归为 SD（AppAcqStart 不支持同时双 sink）。 */
  sink = ((cfg.sink_mask & ACQ_SINK_SD) != 0U) ? APP_ACQ_SINK_SD : APP_ACQ_SINK_USB;

  printf("[Acq] boot_acquire=1 -> 自动启动 sink=%s duration=%lu ms\r\n",
         (sink == APP_ACQ_SINK_SD) ? "SD" : "USB",
         (unsigned long)cfg.duration_ms);

  (void)AppAcqStart(sink, cfg.duration_ms);
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
  g_system_error = 1U;  /* SD 写失败触发异常告警 */

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
    g_system_error = 1U;  /* 传感器初始化失败触发异常告警 */
    printf("[LSM6DSOX] init failed, task exit\r\n");
    /* Null the handle before self-terminating so AppPrintRuntimeDiag's
     * osThreadGetStackSpace() reads NULL (→0) instead of the freed TCB (L1). */
    lsm6dsoxTaskHandle = NULL;
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
      lsm6dsoxTaskHandle = NULL;   /* avoid dangling handle in diag (L1) */
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
    uint8_t output_format = ACQ_OUTPUT_CSV;  /* 默认 CSV */
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.lsm6dsox.enabled == 0U))
      {
        osDelay(APP_ACQ_IDLE_DELAY_MS);
        continue;
      }
      /* output_format 仅对 SD 存储生效,USB 传输始终用 CSV */
      if (AppAcqIsSdSessionActive() != 0U)
      {
        output_format = cfg.output_format;
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

          if (output_format == ACQ_OUTPUT_BIN)
          {
            /* BIN 模式:构建二进制帧 */
            SensorBinFrame_LSM_t bin_frame;
            bin_frame.frame_id = fid;
            bin_frame.timestamp_us = lsm_ts_us;
            bin_frame.acc_x = cur_acc[0];
            bin_frame.acc_y = cur_acc[1];
            bin_frame.acc_z = cur_acc[2];
            bin_frame.gyr_x = cur_gyr[0];
            bin_frame.gyr_y = cur_gyr[1];
            bin_frame.gyr_z = cur_gyr[2];
            bin_frame.crc32 = SensorBin_CalcCRC32(&bin_frame, sizeof(bin_frame) - 4U);

            RingBuf_Write(&g_ring_lsm_imu, (const uint8_t *)&bin_frame, sizeof(bin_frame));
          }
          else
          {
            /* CSV 模式:构建文本行 */
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
    g_system_error = 1U;  /* 传感器初始化失败触发异常告警 */
    printf("[H3LIS100DL] init failed, task exit\r\n");
    h3lis100dlTaskHandle = NULL;   /* avoid dangling handle in diag (L1) */
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
    uint8_t output_format = ACQ_OUTPUT_CSV;  /* 默认 CSV */
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.h3lis100dl.enabled == 0U))
      {
        osDelay(APP_ACQ_IDLE_DELAY_MS);
        continue;
      }
      /* output_format 仅对 SD 存储生效,USB 传输始终用 CSV */
      if (AppAcqIsSdSessionActive() != 0U)
      {
        output_format = cfg.output_format;
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
      /* 阈值事件门控:每样本判合矢量越限(仅阈值模式生效,非阈值模式立即返回)。 */
      AppEvtH3Sample(data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);

      /* Monotonic timestamp: seed with DWT on first sample, then +interval per sample. */
      static uint32_t h3_ts_us = 0U;
      static uint8_t h3_ts_init = 0U;
      if (h3_ts_init == 0U) { h3_ts_us = AppDwtUs(); h3_ts_init = 1U; }
      h3_ts_us += s_h3_odr_interval_us;

      uint32_t fid = ++g_h3_frame_id_counter;
      uint32_t epoch_s_i = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);

      if (output_format == ACQ_OUTPUT_BIN)
      {
        /* BIN 模式:构建二进制帧(使用原始 int16_t,避免 float 精度损失) */
        SensorBinFrame_H3_t bin_frame;
        bin_frame.frame_id = fid;
        bin_frame.timestamp_us = h3_ts_us;
        bin_frame.acc_x = (int16_t)data.acc_mg[0];  /* mg → int16_t */
        bin_frame.acc_y = (int16_t)data.acc_mg[1];
        bin_frame.acc_z = (int16_t)data.acc_mg[2];
        bin_frame.crc32 = SensorBin_CalcCRC32(&bin_frame, sizeof(bin_frame) - 4U);

        RingBuf_Write(&g_ring_h3_acc, (const uint8_t *)&bin_frame, sizeof(bin_frame));
      }
      else
      {
        /* CSV 模式:构建文本行 */
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
    g_system_error = 1U;  /* 传感器初始化失败触发异常告警 */
    printf("[QMA6100P] init failed, task exit\r\n");
    qma6100pTaskHandle = NULL;   /* avoid dangling handle in diag (L1) */
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
    qma6100pTaskHandle = NULL;   /* avoid dangling handle in diag (L1) */
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
    uint8_t output_format = ACQ_OUTPUT_CSV;  /* 默认 CSV */
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.qma6100p.enabled == 0U))
      {
        osDelay(APP_ACQ_IDLE_DELAY_MS);
        continue;
      }
      /* output_format 仅对 SD 存储生效,USB 传输始终用 CSV */
      if (AppAcqIsSdSessionActive() != 0U)
      {
        output_format = cfg.output_format;
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

      if (output_format == ACQ_OUTPUT_BIN)
      {
        /* BIN 模式:构建二进制帧 */
        SensorBinFrame_QMA_t bin_frame;
        bin_frame.frame_id = fid;
        bin_frame.timestamp_us = qma_ts_us;
        bin_frame.acc_x = rx;
        bin_frame.acc_y = ry;
        bin_frame.acc_z = rz;
        bin_frame.crc32 = SensorBin_CalcCRC32(&bin_frame, sizeof(bin_frame) - 4U);

        RingBuf_Write(&g_ring_qma_acc, (const uint8_t *)&bin_frame, sizeof(bin_frame));
      }
      else
      {
        /* CSV 模式:构建文本行 */
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
    uint8_t output_format = ACQ_OUTPUT_CSV;  /* 默认 CSV */
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.aht20.enabled == 0U)) { osDelay(APP_ACQ_IDLE_DELAY_MS); continue; }
      /* output_format 仅对 SD 存储生效,USB 传输始终用 CSV */
      if (AppAcqIsSdSessionActive() != 0U)
      {
        output_format = cfg.output_format;
      }
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
    uint64_t ts_us = AppTime_GetEpochUs();

    if (output_format == ACQ_OUTPUT_BIN)
    {
      /* BIN 模式:构建二进制帧(temp_raw/humidity_raw 存放 float 的二进制表示) */
      SensorBinFrame_AHT_t bin_frame;
      bin_frame.frame_id = fid;
      bin_frame.timestamp_us = ts_us;
      memcpy(&bin_frame.temp_raw, &temp_c, sizeof(float));  /* float 按位存储 */
      memcpy(&bin_frame.humidity_raw, &hum, sizeof(float));
      bin_frame.crc32 = SensorBin_CalcCRC32(&bin_frame, sizeof(bin_frame) - 4U);

      RingBuf_Write(&g_ring_aht_env, (const uint8_t *)&bin_frame, sizeof(bin_frame));
    }
    else
    {
      /* CSV 模式:构建文本行 */
      uint32_t off = 0;
      off += AppU32ToDec(&rowbuf[off], fid);          rowbuf[off++] = ',';
      memcpy(&rowbuf[off], dt, 12U); off += 12U;       rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], temp_c);         rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], hum);
      rowbuf[off++] = '\r'; rowbuf[off++] = '\n';
      RingBuf_Write(&g_ring_aht_env, (const uint8_t *)rowbuf, off);
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
    uint8_t output_format = ACQ_OUTPUT_CSV;  /* 默认 CSV */
    {
      AcqConfig_t cfg; AcqConfig_GetCopy(&cfg);
      if ((AppAcqIsRunning() == 0U) || (cfg.lis2mdl.enabled == 0U)) { dt_valid = 0U; osDelay(APP_ACQ_IDLE_DELAY_MS); continue; }
      /* output_format 仅对 SD 存储生效,USB 传输始终用 CSV */
      if (AppAcqIsSdSessionActive() != 0U)
      {
        output_format = cfg.output_format;
      }
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
    uint64_t ts_us = AppTime_GetEpochUs();

    if (output_format == ACQ_OUTPUT_BIN)
    {
      /* BIN 模式:构建二进制帧(使用原始 int16_t raw 数据) */
      SensorBinFrame_MAG_t bin_frame;
      bin_frame.frame_id = fid;
      bin_frame.timestamp_us = ts_us;
      bin_frame.mag_x = raw[0];
      bin_frame.mag_y = raw[1];
      bin_frame.mag_z = raw[2];
      bin_frame.crc32 = SensorBin_CalcCRC32(&bin_frame, sizeof(bin_frame) - 4U);

      RingBuf_Write(&g_ring_mag, (const uint8_t *)&bin_frame, sizeof(bin_frame));
    }
    else
    {
      /* CSV 模式:构建文本行 */
      uint32_t off = 0;
      off += AppU32ToDec(&rowbuf[off], fid);     rowbuf[off++] = ',';
      memcpy(&rowbuf[off], dt, 12U); off += 12U;  rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], mg[0]);    rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], mg[1]);    rowbuf[off++] = ',';
      off += AppF1ToDec(&rowbuf[off], mg[2]);
      rowbuf[off++] = '\r'; rowbuf[off++] = '\n';
      RingBuf_Write(&g_ring_mag, (const uint8_t *)rowbuf, off);
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

    /* SAI DMA error (FIFO over/underrun, bus error) silently kills capture —
     * HAL_SAI_ErrorCallback flags it. Tear down so the branch below restarts
     * from scratch this iteration; the 50ms backoff bounds an error storm (L5). */
    if (running && (Mic_SaiErrorPending() != 0U))
    {
      printf("[Mic] SAI error 0x%lX — restarting capture\r\n",
             (unsigned long)Mic_SaiErrorCode());
      Mic_Stop();
      running = 0U;
      s_mic_capturing = 0U;
      Mic_ClearSaiError();
      osDelay(50U);
    }

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

    /* SD path: the logger drains g_ring_mic to MIC.WAV — this task only handles
     * the mic start/stop edges, no USB PCM pump. */
    osDelay(20U);
  }
}

/* ============================================================================
 *  H3 阈值触发事件门控控制器
 *  - H3 采集任务每样本调 AppEvtH3Sample()，越限则自增 g_evt_trig_seq(唯一写者)。
 *  - logger 任务每轮调 AppEvtSdGate() 推进状态机，返回"此刻是否应开 SD 会话"。
 *  - ARMED/HOLDOFF 期 logger 调 AppEvtTrimRings() 在消费者侧裁剪各 ring，只留最近
 *    ~size/2 作预触发(不碰 RingBuf_Write/生产者，SPSC 不破)。
 *  时间基准用 AppTime_GetEpochUs()(RTOS tick 慢~19%，定时不可用 tick)；logger 每轮
 *  仅调用一次，频率低，符合 AppTime 限流要求。
 * ========================================================================= */
typedef enum { EVT_OFF = 0, EVT_ARMED, EVT_REC, EVT_HOLDOFF } AppEvtState_t;

static volatile uint32_t g_evt_trig_seq = 0U;   /* H3 在"上升越过触发阈值"时自增(唯一写者) */
static volatile uint8_t  g_evt_over     = 0U;    /* H3 迟滞态:1=当前在触发阈值之上(唯一写者 H3) */
static uint32_t  g_evt_last_seen_seq = 0U;       /* logger 上次观察到的 seq */
static uint64_t  g_evt_level_mg2     = 0U;       /* 触发阈值平方(mg²) */
static uint64_t  g_evt_release_mg2   = 0U;       /* 释放阈值平方(mg²)=0.8×触发,迟滞防抖 */
static uint32_t  g_evt_post_ms       = 0U;       /* 后触发窗口(ms,用 tick 计时) */
static uint32_t  g_evt_holdoff_ms    = 0U;       /* 死区(ms) */
static uint32_t  g_evt_deadline_ms   = 0U;       /* REC:post 截止 / HOLDOFF:死区截止(tick ms) */
static AppEvtState_t g_evt_state      = EVT_OFF;
static uint8_t   g_evt_mode          = 0U;       /* 1=本次采集为阈值模式 */

/* 采集启动时调用：按配置决定是否进入阈值模式并预算阈值/窗口。 */
static void AppEvtBeginSession(const AcqConfig_t *cfg)
{
  if (cfg != NULL && cfg->trigger_mode == ACQ_TRIGGER_THRESHOLD)
  {
    uint64_t mg = (uint64_t)cfg->trig_level_g * 1000U;   /* g → mg */
    g_evt_level_mg2   = mg * mg;
    uint64_t rmg = (mg * 8U) / 10U;                      /* 释放阈值 = 触发的 0.8 倍 */
    g_evt_release_mg2 = rmg * rmg;
    g_evt_post_ms    = (uint32_t)cfg->trig_post_sec    * 1000U;
    g_evt_holdoff_ms = (uint32_t)cfg->trig_holdoff_sec * 1000U;
    g_evt_last_seen_seq = g_evt_trig_seq;   /* 丢弃布防前的历史触发 */
    g_evt_over  = 0U;
    g_evt_state = EVT_ARMED;
    g_evt_mode  = 1U;
    if (cfg->h3lis100dl.enabled == 0U)
    {
      printf("[Evt] 警告：阈值模式但 H3 未启用，将永不触发！请在 DEVCFG.JSN 启用 h3lis100dl\r\n");
    }
    printf("[Evt] 阈值模式布防: level=%ug post=%us holdoff=%us\r\n",
           (unsigned int)cfg->trig_level_g,
           (unsigned int)cfg->trig_post_sec,
           (unsigned int)cfg->trig_holdoff_sec);
  }
  else
  {
    g_evt_mode  = 0U;
    g_evt_state = EVT_OFF;
  }
}

/* 采集停止时调用。 */
static void AppEvtEndSession(void)
{
  g_evt_mode  = 0U;
  g_evt_state = EVT_OFF;
}

/* 本次采集是否为阈值模式(供 auto-stop / LED 判定)。 */
static uint8_t AppEvtThresholdMode(void) { return g_evt_mode; }
/* 阈值模式下是否正在录制事件(REC)。布防/死区返回 0。 */
static uint8_t AppEvtIsRecording(void)
{
  return (uint8_t)((g_evt_mode != 0U) && (g_evt_state == EVT_REC));
}

/* H3 采集任务每样本调用:带迟滞的合矢量越限检测。整数运算,~400Hz,可忽略。
 * 上升越过"触发阈值"→ 新触发(seq++)且置 over;跌回"释放阈值"(0.8×)以下 → 清 over。
 * 中间带内保持状态,避免在阈值附近抖动反复触发/反复收尾。 */
static void AppEvtH3Sample(float mx, float my, float mz)
{
  if (g_evt_mode == 0U) return;
  int64_t ix = (int64_t)mx, iy = (int64_t)my, iz = (int64_t)mz;
  uint64_t mag2 = (uint64_t)(ix * ix + iy * iy + iz * iz);
  if (g_evt_over == 0U)
  {
    if (mag2 > g_evt_level_mg2)      /* 上升越过触发阈值 → 新触发 */
    {
      g_evt_over     = 1U;
      g_evt_trig_seq++;              /* 唯一写者(H3),logger 只读 */
    }
  }
  else
  {
    if (mag2 < g_evt_release_mg2)    /* 跌回释放阈值之下 → 本次活跃结束 */
    {
      g_evt_over = 0U;
    }
  }
}

/* logger 任务每轮调用:推进状态机,返回 1=此刻应开 SD 会话(REC),0=否(ARMED/HOLDOFF)。
 * 非阈值模式不应调用本函数(返回 1 兜底)。 */
static uint8_t AppEvtSdGate(void)
{
  if (g_evt_mode == 0U) return 1U;

  uint32_t seq      = g_evt_trig_seq;
  uint8_t  new_trig = (uint8_t)(seq != g_evt_last_seen_seq);
  g_evt_last_seen_seq = seq;
  /* 用 tick(ms) 计时,不用 AppTime_GetEpochUs:后者带 __disable_irq+DWT 维护,被本
   * gate 每个 logger 循环高频调用会拖垮吞吐并致 apptime 错乱(实测 CKBX0412)。
   * tick 比真实慢 ~19%,3s post 实际约 3.6s,对事件录制完全够用。 */
  uint32_t now = osKernelGetTickCount();

  switch (g_evt_state)
  {
    case EVT_ARMED:
      if (new_trig)
      {
        g_evt_deadline_ms = now + g_evt_post_ms;
        g_evt_state = EVT_REC;
        printf("[Evt] 触发! 开始录制事件\r\n");
        return 1U;
      }
      return 0U;

    case EVT_REC:
      /* 信号仍在阈值之上(over)或又有新上升触发 → 持续录,刷新 post 截止;
       * 跌回释放阈值之下并安静满 post_ms 才收尾(迟滞,防止抖动无限延长)。 */
      if ((g_evt_over != 0U) || new_trig) { g_evt_deadline_ms = now + g_evt_post_ms; }
      if ((int32_t)(now - g_evt_deadline_ms) >= 0)
      {
        g_evt_deadline_ms = now + g_evt_holdoff_ms;
        g_evt_state = EVT_HOLDOFF;
        printf("[Evt] 事件录制结束,进入死区\r\n");
        return 0U;
      }
      return 1U;

    case EVT_HOLDOFF:
      if ((int32_t)(now - g_evt_deadline_ms) >= 0) { g_evt_state = EVT_ARMED; }
      return 0U;   /* 死区内忽略触发(new_trig 已被吞掉) */

    default:
      return 0U;
  }
}

/* ARMED/HOLDOFF 期消费者侧裁剪:每条 ring 只留最近 ~size/2(预触发),丢弃更旧字节。
 * 仅 logger(消费者)动 rd_idx,生产者不变,SPSC 不破。按字节裁剪可能在头部留半帧,
 * 事件预触发首条记录可能不完整(BIN 由 CRC 跳过/CSV 跳首行),属可接受的边界瑕疵。 */
static void AppEvtTrimRing(AppRingBuffer_t *rb)
{
  uint32_t avail = RingBuf_Available(rb);
  uint32_t keep  = rb->size / 2U;
  if (avail > keep) { RingBuf_Consume(rb, avail - keep); }
}

static void AppEvtTrimRings(void)
{
  AppEvtTrimRing(&g_ring_lsm_imu);
  AppEvtTrimRing(&g_ring_qma_acc);
  AppEvtTrimRing(&g_ring_h3_acc);
  AppEvtTrimRing(&g_ring_mic);
  AppEvtTrimRing(&g_ring_aht_env);
  AppEvtTrimRing(&g_ring_mag);
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

  static uint8_t s_prev_sd_acq = 0U;   /* 上一轮 SD 采集是否活跃,用于检测起止沿 */
  for (;;)
  {
    AppAcqControl_t acq;
    uint8_t sd_session_active;
    uint8_t acq_running;

    AppAcqCheckAutoStop();
    AppAcqGetCopy(&acq);
    acq_running = acq.running;

    uint8_t sd_acq_active = (uint8_t)((acq_running != 0U) && (acq.sink == APP_ACQ_SINK_SD));

    /* 事件门控生命周期:SD 采集起/止沿驱动 BeginSession/EndSession */
    if (sd_acq_active != 0U && s_prev_sd_acq == 0U)
    {
      AcqConfig_t ecfg; AcqConfig_GetCopy(&ecfg);
      AppEvtBeginSession(&ecfg);
    }
    else if (sd_acq_active == 0U && s_prev_sd_acq != 0U)
    {
      AppEvtEndSession();
    }
    s_prev_sd_acq = sd_acq_active;

    if (g_evt_mode != 0U && sd_acq_active != 0U)
    {
      /* 阈值模式:gate 决定此刻是否开 SD 会话;未录时维持预触发裁剪 */
      uint8_t gate = AppEvtSdGate();
      sd_session_active = gate;
      if (gate == 0U) { AppEvtTrimRings(); }
    }
    else
    {
      /* 非阈值模式:维持原行为 */
      sd_session_active = sd_acq_active;
    }

    if (sd_session_active == 0U)
    {
      if (sd_file_open != 0U)
      {
        AppFlowStatsSetMode(0U, 0U);
        AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
      }
      else
      {
        AppFlowStatsSetMode(0U, 0U);
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
         * and frame_id/tick_ms in the new session align with real samples.
         * 阈值模式例外:ring 里是要落盘的预触发段,绝不能清;帧号跨事件保持连续。 */
        if (g_evt_mode == 0U)
        {
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
        }
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
        AppLoggerStopSdSession(&sd_file_open, &rows_since_sync);
        AppFlowStatsSetMode(0U, 0U);
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
      AppFlowStatsSetMode(0U, 0U);
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

/* I2C2 bus lock (ES8311 codec + PCF85063 RTC share the bus). Sequence-level:
 * callers hold it across a whole codec register sequence or a whole RTC read so
 * the two never interleave on the wire (M1). NULL-safe before the mutex is
 * created (early boot) — acquire/release become no-ops. */
void AppI2c2Lock(void)
{
  if (i2c2_mutex != NULL) { (void)osMutexAcquire(i2c2_mutex, osWaitForever); }
}
void AppI2c2Unlock(void)
{
  if (i2c2_mutex != NULL) { (void)osMutexRelease(i2c2_mutex); }
}

/* FreeRTOS malloc-failed hook — called when pvPortMalloc (task/queue/mutex
 * creation, or any heap_4 allocation) returns NULL because the 64KB heap is
 * exhausted. Print then reset so an OOM self-recovers instead of silently
 * leaving a half-initialised system (M6). */
void vApplicationMallocFailedHook(void)
{
  printf("[FATAL] malloc failed (FreeRTOS heap exhausted)\r\n");
  taskDISABLE_INTERRUPTS();
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
