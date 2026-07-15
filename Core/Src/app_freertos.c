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
/* ★2026-07-09 事件驱动 logger(借鉴 DATALOG1 SDM_Thread):生产者(含 mic SAI ISR)在某个 ring
 * 攒过唤醒阈值时 osSemaphoreRelease 唤醒 logger,logger 平时阻塞睡(50ms 超时兜底)不再 osDelay(2)
 * 忙轮询空转。计数信号量(max 大),多次 release 被 logger 一次排空;边沿触发避免淹没。 */
static osSemaphoreId_t s_logger_wake;   /* 数据就绪 → 唤醒 logger 排空 ring */
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
static uint8_t s_lsm_acc_ringbuf[APP_RING_LSM_ACC_SIZE];
static uint8_t s_lsm_gyr_ringbuf[APP_RING_LSM_GYR_SIZE];
static uint8_t s_qma_acc_ringbuf[APP_RING_QMA_ACC_SIZE];
static uint8_t s_h3_acc_ringbuf[APP_RING_H3_ACC_SIZE];
static uint8_t s_aht_env_ringbuf[APP_RING_AHT_ENV_SIZE];
static uint8_t s_mag_ringbuf[APP_RING_MAG_SIZE];
static AppRingBuffer_t g_ring_lsm_acc;   /* LSM 加速度 → LSM_ACC.CSV */
static AppRingBuffer_t g_ring_lsm_gyr;   /* LSM 角速度 → LSM_GYR.CSV */
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

osThreadId_t sdWriterTaskHandle;
const osThreadAttr_t sdWriterTask_attributes = {
  .name = "sdWriterTask",
  .priority = (osPriority_t)osPriorityAboveNormal,  /* = 当前 logger,维持"SD写在传感器(High)之下" */
  .stack_size = 1024 * 4  /* 4KB:f_write/f_sync/WavCheckpoint 调用链 + printf 诊断余量 */
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
static uint32_t RingBuf_CopyToBounce(AppRingBuffer_t *rb, uint8_t *dst, uint32_t cap);
static void     RingBuf_Consume(AppRingBuffer_t *rb, uint32_t len);
static int      LoggerDrainRing(AppRingBuffer_t *rb, uint8_t file_idx, uint32_t min_flush,
                                uint32_t *rows_since_sync, FRESULT *out_res);
/* 阈值控制器(AppEvt*)已移除 */
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
static void AppSdBlockPoolInit(void);
static void StartSdWriterTask(void *argument);
static void AppAcqStopInternal(uint32_t now_ms);
static void AppAcqCheckAutoStop(void);
static void AppAcqResetSessionTimer(void);
uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms);
uint32_t AppAcqStop(void);
static uint32_t AppAcqDrainPendingStop(void);
static void AppFlowStatsRecordWriteFailure(void);

/* ===== 双缓冲块池 + 双队列(2026-07-14) — 类型/句柄前置(供 AppLoggerStopSdSession 等更早的
 * 函数引用;存储数组 s_block_pool 与 AppSdBlockPoolInit 定义仍在下方 SD 写区)。 =====
 * WriteQ 消息类型:写线程是 FatFs 唯一访问者,所有落盘动作都经此队列串行化。 */
typedef enum {
  APP_SDMSG_DATA = 0,   /* 写一个数据块到 file_idx,写完归还 block_idx 到 FreeQ */
  APP_SDMSG_SYNC,       /* FatFs_SD_LoggerSync() */
  APP_SDMSG_WAVCKPT,    /* FatFs_SD_WavCheckpoint() */
  APP_SDMSG_STOP,       /* 收尾:FatFs_SD_LoggerStop(),然后 release s_sd_writer_done */
  APP_SDMSG_APPENDFRAME,/* 逐帧写(LSM温度行到TMP_LOW),携带 msg.frame */
  APP_SDMSG_DEVCFG      /* 写线程调 DeviceCfg_WriteCurrentToSD(收尾配置快照,FIFO 保证在 STOP 前) */
} AppSdMsgType_t;

typedef struct {
  uint8_t  type;        /* AppSdMsgType_t */
  uint8_t  block_idx;   /* DATA:块索引 0..N-1;其他消息忽略 */
  uint8_t  file_idx;    /* DATA:目标文件索引;其他忽略 */
  uint32_t len;         /* DATA:块内有效字节(扇区对齐,末块可非对齐);其他忽略 */
  AppSensorFrame_t frame;   /* 仅 APP_SDMSG_APPENDFRAME 使用 */
} AppSdWriteMsg_t;

static osMessageQueueId_t s_free_q;    /* 装空闲 block_idx(uint8_t),容量 N */
static osMessageQueueId_t s_write_q;   /* 装 AppSdWriteMsg_t,容量 APP_SD_WRITEQ_LEN */
static osSemaphoreId_t    s_sd_writer_done;  /* 收尾屏障:STOP 处理完后 release */


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
  s_logger_wake = osSemaphoreNew(16, 0, NULL);  /* 计数,max16/init0:生产者唤醒 logger */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Load device config from SD card (DEVCFG.JSN) before tasks start,
   * so sensor tasks read the correct ODR/range/enabled values. */
  (void)DeviceCfg_LoadFromSD();

  /* Initialise SPSC ring buffers (data arrays are static, no allocation). */
  RingBuf_Init(&g_ring_lsm_acc, s_lsm_acc_ringbuf, APP_RING_LSM_ACC_SIZE);
  RingBuf_Init(&g_ring_lsm_gyr, s_lsm_gyr_ringbuf, APP_RING_LSM_GYR_SIZE);
  RingBuf_Init(&g_ring_qma_acc, s_qma_acc_ringbuf, APP_RING_QMA_ACC_SIZE);
  RingBuf_Init(&g_ring_h3_acc,  s_h3_acc_ringbuf,  APP_RING_H3_ACC_SIZE);
  RingBuf_Init(&g_ring_mic,     s_mic_ringbuf,     APP_RING_MIC_SIZE);
  RingBuf_Init(&g_ring_aht_env, s_aht_env_ringbuf, APP_RING_AHT_ENV_SIZE);
  RingBuf_Init(&g_ring_mag,     s_mag_ringbuf,     APP_RING_MAG_SIZE);

  /* 双缓冲块池 + 双队列 + 收尾信号量(必须在写线程/生产者跑之前建好) */
  AppSdBlockPoolInit();

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
  sdWriterTaskHandle  = osThreadNew(StartSdWriterTask,  NULL, &sdWriterTask_attributes);
  micTaskHandle       = osThreadNew(StartMicTask,       NULL, &micTask_attributes);
  aht20TaskHandle   = osThreadNew(StartAht20Task,   NULL, &aht20Task_attributes);
  lis2mdlTaskHandle = osThreadNew(StartLis2mdlTask, NULL, &lis2mdlTask_attributes);
  printf("[RTOS] lsm6dsoxTask created: %s\r\n", (lsm6dsoxTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] h3lis100dlTask created: %s\r\n", (h3lis100dlTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] qma6100pTask created: %s\r\n", (qma6100pTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] loggerTask created: %s\r\n", (loggerTaskHandle != NULL) ? "ok" : "FAILED");
  printf("[RTOS] sdWriterTask created: %s\r\n", (sdWriterTaskHandle != NULL) ? "ok" : "FAILED");
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
  printf("[Ring] lsmA drop=%lu hwm=%lu/%lu | lsmG drop=%lu hwm=%lu/%lu | h3 drop=%lu hwm=%lu/%lu | qma drop=%lu hwm=%lu/%lu | mic drop=%lu hwm=%lu/%lu\r\n",
         (unsigned long)g_ring_lsm_acc.dropped, (unsigned long)g_ring_lsm_acc.high_watermark, (unsigned long)g_ring_lsm_acc.size,
         (unsigned long)g_ring_lsm_gyr.dropped, (unsigned long)g_ring_lsm_gyr.high_watermark, (unsigned long)g_ring_lsm_gyr.size,
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
    /* 1) 强排空每个环的末块(min_flush=0 → 全排空,允许非对齐末块),全部压入 WriteQ。
     *    LoggerDrainRing 现在是"凑块入队",do/while 直到所有环再无数据可入队。 */
    {
      uint32_t dummy_rss = 0U;
      FRESULT fr;
      int any;
      do {
        any = 0;
        if (LoggerDrainRing(&g_ring_lsm_acc, 0U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_lsm_gyr, FATFS_SD_FILE_LSM_GYR, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_qma_acc, 3U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_h3_acc,  2U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_mic, FATFS_SD_FILE_MIC_WAV, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_aht_env, 4U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_mag,     5U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
      } while (any != 0);
    }

    /* 2) 收尾全部经写线程串行执行(FIFO):先前入队的 DATA/SYNC/APPENDFRAME 都排在前面,
     *    这里再压 DEVCFG(配置快照)+STOP。写线程按序处理,STOP 时所有数据已落盘、配置已写,
     *    再 finalize WAV + f_close + unmount,然后 release 信号量。STOP 完成 = 真正收尾完成,
     *    无需单独的 FreeQ 屏障(块归还是 DATA 处理的副作用,STOP 在其后必然已归还)。 */
    {
      AppSdWriteMsg_t m = { 0 };
      m.type = (uint8_t)APP_SDMSG_DEVCFG;
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);

      m.type = (uint8_t)APP_SDMSG_STOP;
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);

      /* 等写线程处理完 STOP(= 数据/配置全落盘 + 文件 close + 卷 unmount)。超时兜底防卡死。
       * 最坏:队列里若干 DATA 块 × 单次写<1s + DevCfg + finalize,给足 8s。 */
      (void)osSemaphoreAcquire(s_sd_writer_done, 8000U);
    }

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
  rb->last_write_tick = 0U;
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
  rb->last_write_tick = 0U;  /* 写时间封顶基准,按会话清零 */
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

  /* ★事件驱动唤醒(边沿触发):本环攒过唤醒阈值时唤醒 logger 排空。阈值=min(WRITE_BLOCK,
   * 3/4环),与 LoggerDrainRing 的 gate 一致 → logger 醒来即有满块可写。无状态边沿:写前
   * avail=(avail-len),仅在 未过→过 的那一次 release,避免每次写都 release 淹没信号量。
   * 计数信号量多余 release 也被 logger 一次排空,故偶发重复无害。s_logger_wake==NULL(初始化
   * 前)或非会话期的 release 也无害(logger 醒来发现无会话/无数据即继续睡)。mic 走此函数,在
   * SAI ISR 中 release —— osSemaphoreRelease 是 ISR 安全的(项目 EXTI ISR 已用同款)。 */
  {
    uint32_t wake_gate = (rb->size * 3U) / 4U;
    if (wake_gate > APP_SD_WRITE_BLOCK) wake_gate = APP_SD_WRITE_BLOCK;
    uint32_t prev = avail - len;
    if ((prev < wake_gate) && (avail >= wake_gate) && (s_logger_wake != NULL))
    {
      (void)osSemaphoreRelease(s_logger_wake);
    }
  }
  return len;
}

/* Non-static producer wrapper for the mic ring (SPSC). Sole writer is the SAI
 * DMA ISR (HAL_SAI_RxHalf/CpltCallback in mic_capture.c). Resolves the extern
 * declared in mic_capture.c. */
uint32_t AppRing_WriteMic(const uint8_t *src, uint32_t len)
{
  return RingBuf_Write(&g_ring_mic, src, len);
}

/* F1(消除环绕截断,降 PROGRAMMING 次数):把最多 cap 字节从环拷进 dst,**跨越环绕**
 * (rd 接近环尾时分两段 memcpy 拼成一块),返回实际拷贝字节数。旧 PeekContiguous 只到
 * 环尾就截断 → wrap 处被迫小写 → 平均每次写仅 11KB(远小于 64KB 上限)→ 写次数暴涨 →
 * 91.7% 时间耗在每次写的固定 PROGRAMMING 等待(实测诊断,见
 * docs/.../2026-07-08-dropframe-opportunistic-write.md §8)。跨环绕拷贝后单次写接近满
 * cap → 写次数大降 → PROGRAMMING 总时间降 → 掉帧降。拷进私有 bounce,IDMA 源静态、
 * 与生产者解耦(同旧 memcpy 语义)。返回 0 = 环空。 */
static uint32_t RingBuf_CopyToBounce(AppRingBuffer_t *rb, uint8_t *dst, uint32_t cap)
{
  if (rb == NULL || rb->data == NULL || dst == NULL || cap == 0U) return 0U;
  uint32_t wr = rb->wr_idx;
  uint32_t rd = rb->rd_idx;
  if (wr == rd) return 0U;

  uint32_t avail = (wr > rd) ? (wr - rd) : (rb->size - rd + wr);
  /* cap = 调用方(LoggerDrainRing)算好的目标字节数,已保证扇区对齐(正常写)或为末块余量
   * (会话停止)。此处 want=min(avail,cap):正常情况 avail>=cap → want=cap(对齐块);cap 已夹到
   * WRITE_BLOCK(32KB,=bounce 容量)。avail 与 cap 同为 512 倍时 want 仍 512 倍,保持对齐。 */
  uint32_t want = (avail > cap) ? cap : avail;

  uint32_t first = rb->size - rd;        /* 环尾前的连续段长度 */
  if (first >= want)
  {
    memcpy(dst, &rb->data[rd], want);    /* 无环绕,一段 */
  }
  else
  {
    memcpy(dst, &rb->data[rd], first);             /* 环尾段 */
    memcpy(dst + first, &rb->data[0], want - first); /* 环首段,拼成一块 */
  }
  return want;
}

static void RingBuf_Consume(AppRingBuffer_t *rb, uint32_t len)
{
  if (rb == NULL || len == 0U) return;
  rb->rd_idx = (rb->rd_idx + len) % rb->size;
}

/* ===== 双缓冲块池存储(2026-07-14) — 类型/句柄声明已上移到文件前置声明区。 =====
 * N 个 32KB 连续对齐块。LoggerDrainRing 取空闲块 memcpy 后交写线程落盘。
 * 32 字节对齐:IDMA 对 SDMMC FIFO 做 32-bit 访问,源缓冲须字长对齐(H2 防护,同旧 bounce)。 */
static uint8_t s_block_pool[APP_SD_BLOCK_POOL_N][APP_SD_WRITE_BLOCK] __attribute__((aligned(32)));
/* 写线程句柄 sdWriterTaskHandle 随 sdWriterTask_attributes 一起定义(沿用项目 *TaskHandle 惯例) */

/* 创建块池队列 + 收尾信号量,并把 N 个块索引全部塞进 FreeQ。由 MX_FREERTOS_Init 调用。 */
static void AppSdBlockPoolInit(void)
{
  s_free_q  = osMessageQueueNew(APP_SD_BLOCK_POOL_N, sizeof(uint8_t), NULL);
  s_write_q = osMessageQueueNew(APP_SD_WRITEQ_LEN, sizeof(AppSdWriteMsg_t), NULL);
  s_sd_writer_done = osSemaphoreNew(1, 0, NULL);
  for (uint8_t i = 0U; i < APP_SD_BLOCK_POOL_N; i++)
  {
    (void)osMessageQueuePut(s_free_q, &i, 0U, 0U);
  }
}

/* Drain one contiguous chunk from a ring into a free pool block, then enqueue it
 * to the writer task (no direct f_write).
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
  uint32_t n;
  uint32_t avail = RingBuf_Available(rb);
  if (avail == 0U) return 0;

  /* ★2026-07-09 扇区对齐写(借鉴 DATALOG1 每次 f_write 恒为整数扇区块)。取代旧"攒到 gate
   * 写任意字节数"。核心:除会话停止的末块外,【每次写的字节数恒为 512 的整数倍】→ 写整数个
   * 扇区,零 FIL 窗口 read-modify-write(_FS_TINY=0 时非对齐尾部会触发回读扇区),f_write 直接
   * 走 disk_write 多块路径 → 提吞吐、消 IDMA 512 边界复写。
   * 决策:
   *  (a) 会话停止(min_flush==0):写全部 avail(末块,允许非对齐——文件收尾,CSV 靠 \n 无害)。
   *  (b) 否则:aligned = avail 向下取整到 512。gate = min(WRITE_BLOCK, size/2 取整到512)——大环
   *      攒到 32KB、小环攒到半环才写,减少小写。aligned>=gate 或已超时(低速环)则写 aligned;
   *      否则继续攒。aligned 恒 512 倍 → 扇区对齐;余量(avail%512)留环里下轮再拼。 */
  uint32_t want;
  if (min_flush == 0U)
  {
    want = avail;                                   /* (a) 会话停止:全排空(末块) */
  }
  else
  {
    uint32_t aligned = avail - (avail % APP_SD_SECTOR);   /* 向下取整到扇区 */
    if (aligned == 0U) return 0;                    /* 不足一扇区:继续攒 */

    /* 攒批阈值 gate:恢复为 3/4 环(与扇区对齐前的老逻辑一致),而非半环——半环让小环
     * (H3 16K/QMA 32K/MAG 16K)只攒到 8-16KB 就写,块太小 → 写次数升、PROGRAMMING 摊薄少
     * → 吞吐降(实测 22-23 会话块从 13KB→8.5KB、落盘 648→626KB/s、掉帧升)。3/4 环让小环
     * 攒到 12/24/12KB,大环封顶 FLUSH_CHUNK(64KB)。全部取整到扇区。 */
    uint32_t gate = (rb->size * 3U) / 4U;
    /* ★2026-07-13 封顶改 APP_SD_WRITE_BLOCK(32KB),与生产者 wake_gate(RingBuf_Write
     * 里 min(3/4环,WRITE_BLOCK))【完全一致】。原封顶 FLUSH_CHUNK(64KB)导致大环(QMA扩到
     * 64KB后 drain_gate=48KB)与 wake_gate(封顶32KB)裂开:环攒到32KB唤醒logger,但logger
     * 见 aligned(32KB)<gate(48KB)→不写继续攒;而32KB那次wake边沿已用掉,涨到48KB不再唤醒
     * → QMA卡在48-64KB靠50ms超时兜底,高负载下溢出→实测QMA中段落盘停止掉25-63%。封顶对齐
     * 后:任何环攒到32KB即写,wake与drain同步,消除"唤醒却不写、不写又不唤醒"窗口。 */
    if (gate > APP_SD_WRITE_BLOCK) gate = APP_SD_WRITE_BLOCK;  /* 封顶32KB,与wake_gate一致 */
    gate -= (gate % APP_SD_SECTOR);                 /* 取整到扇区 */
    if (gate == 0U) gate = APP_SD_SECTOR;           /* 极小环兜底:至少一扇区 */

    uint32_t now = osKernelGetTickCount();
    uint8_t aged = (uint8_t)((rb->last_write_tick != 0U) &&
                             ((uint32_t)(now - rb->last_write_tick) >= APP_SD_WRITE_MAX_AGE_TICKS));
    if (rb->last_write_tick == 0U) rb->last_write_tick = now;  /* 首次起计时 */

    if ((aligned < gate) && !aged) return 0;        /* 不满 gate 且未超时:继续攒 */
    want = aligned;                                 /* 写扇区对齐块(超时也只写对齐部分) */
  }

  /* 夹到块容量(32KB,512 倍 → 保持扇区对齐);超出余量下一轮 drain 继续。 */
  if (want > APP_SD_WRITE_BLOCK) want = APP_SD_WRITE_BLOCK;

  /* 取空闲块(非阻塞)。取不到 = 池空背压 → 本环本轮不写,数据留环靠环兜底(与旧"环满drop"等价)。 */
  uint8_t blk;
  if (osMessageQueueGet(s_free_q, &blk, NULL, 0U) != osOK)
  {
    return 0;   /* 无空闲块:背压,下轮再来 */
  }

  n = RingBuf_CopyToBounce(rb, s_block_pool[blk], want);
  if (n == 0U)
  {
    (void)osMessageQueuePut(s_free_q, &blk, 0U, 0U);  /* 空环:归还块 */
    return 0;
  }

  /* ★消费即释放:memcpy 进块后立即 Consume 腾空环(防溢出的关键动作),不等写成功。
   * 块此刻成为数据唯一持有者,交给写线程落盘。 */
  RingBuf_Consume(rb, n);
  *rows_since_sync += n;
  rb->last_write_tick = osKernelGetTickCount();

  AppSdWriteMsg_t m = { .type = (uint8_t)APP_SDMSG_DATA, .block_idx = blk,
                        .file_idx = file_idx, .len = n };
  /* 压 WriteQ:队列已按 N+余量设深。块数=N≤队列容量,正常不满;用 osWaitForever 兜底保证不丢块。 */
  (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
  *out_res = FR_OK;
  return 1;
}

/* 专职 SD 写线程:WriteQ 的唯一消费者,FatFs 的唯一访问者。按 FIFO 串行执行落盘动作,
 * 保证 FatFs 单线程不变量。写数据块前 __DMB() 确保源块对 IDMA 可见(M33 写缓冲,H1 防护)。 */
static void StartSdWriterTask(void *argument)
{
  (void)argument;
  AppSdWriteMsg_t msg;
  for (;;)
  {
    if (osMessageQueueGet(s_write_q, &msg, NULL, osWaitForever) != osOK) continue;

    switch ((AppSdMsgType_t)msg.type)
    {
      case APP_SDMSG_DATA:
      {
        __DMB();   /* H1:源块已落 SRAM,再启动 IDMA(同旧 LoggerDrainRing) */
        FRESULT r = FatFs_SD_LoggerWriteFileIndex(msg.file_idx,
                                                  s_block_pool[msg.block_idx], msg.len);
        /* MIC 文件在 WAV 未打开时返回 FR_NOT_ENABLED:SAI ISR 仍在灌 g_ring_mic,
         * 属预期的音频丢弃(改造前 LoggerDrainRing 静默消费),不算写失败、不刷屏。 */
        if ((r != FR_OK) &&
            !((r == FR_NOT_ENABLED) && (msg.file_idx == FATFS_SD_FILE_MIC_WAV)))
        {
          printf("[SdWriter] write fail idx=%u %s (%d)\r\n",
                 (unsigned int)msg.file_idx, FatFs_SD_ResultToString(r), (int)r);
          AppFlowStatsRecordWriteFailure();
        }
        /* 无论成败(含预期的 MIC 丢弃)都归还块索引,防块泄漏。 */
        (void)osMessageQueuePut(s_free_q, &msg.block_idx, 0U, 0U);
        break;
      }
      case APP_SDMSG_SYNC:
        (void)FatFs_SD_LoggerSync();
        break;
      case APP_SDMSG_WAVCKPT:
        (void)FatFs_SD_WavCheckpoint();
        break;
      case APP_SDMSG_DEVCFG:
        (void)DeviceCfg_WriteCurrentToSD();
        break;
      case APP_SDMSG_STOP:
        FatFs_SD_LoggerStop();
        (void)osSemaphoreRelease(s_sd_writer_done);   /* 解除收尾屏障 */
        break;
      case APP_SDMSG_APPENDFRAME:
      {
        /* 逐帧路径写 LSM 温度行到 TMP_LOW。改造前 logger 侧写失败会停会话;异步化后与 DATA
         * 一致:记录写失败并打印(供审计),但不停会话(温度是低价值诊断,不值得因它中止整会话)。 */
        FRESULT r = FatFs_SD_LoggerAppendFrame(&msg.frame);
        if (r != FR_OK)
        {
          printf("[SdWriter] append fail frame=%lu %s (%d)\r\n",
                 (unsigned long)msg.frame.frame_id, FatFs_SD_ResultToString(r), (int)r);
          AppFlowStatsRecordWriteFailure();
        }
        break;
      }
      default:
        break;
    }
  }
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

    /* ★2026-07-11 只读诊断(APP_LSM_STALL_PROBE):抓 LSM 链 ~周期性阻塞在哪一步。
     * 用真实时钟 AppDwtUs()(合成 timestamp 推不出真实阻塞时刻)。测三段:①等信号量
     * ②读+解析+写环整段 ③本轮总耗时。任一超阈值打印时刻/耗时/两环余量。默认关,零开销。
     * 测完设 0U 移除。DWT ~26.8s 回绕,单段耗时远小于此,安全。 */
#ifndef APP_LSM_STALL_PROBE
#define APP_LSM_STALL_PROBE 1U   /* ★2026-07-14 重开:抓 LSM/MIC ~9-10% 掉帧的 2.5s gap 阻塞源。
                                  * LSM链本身已确认健康(cyc~23ms/fifoMax~256),真凶在 logger。新增
                                  * [LoggerLoop] 探针给整个 drain 循环计时(见下),补"千刀万剐"盲区:
                                  * SLC耗尽后单次写~130ms 卡在 150ms 阈值下,现有 [LoggerBlk] write 不触发,
                                  * 但多次130ms写连续堆积会撑爆 LSM 环。测完设 0U 移除。 */
#endif
#if (APP_LSM_STALL_PROBE != 0U)
    /* ★2026-07-11 v3:全部改用 HAL_GetTick(ms,TIM6,无回绕)——v2 用 AppDwtUs 每26.8s回绕一次,
     * 打出 sem=42亿us 假阳性刷屏。1ms 分辨率足够抓 >20ms 阻塞。已确认 LSM 本身健康(cyc~23ms、
     * fifoMax~256 从不逼近512),故本探针留着只为对照,真凶在 logger(见 [LoggerBlk])。 */
    uint32_t probe_ms0 = HAL_GetTick();
    uint32_t probe_t_sem0 = probe_ms0;
    static uint32_t probe_prev_loop_ms = 0U;
    uint32_t probe_cyc_ms = (probe_prev_loop_ms == 0U) ? 0U : (probe_ms0 - probe_prev_loop_ms);
    probe_prev_loop_ms = probe_ms0;
    uint16_t probe_fifo_max = 0U;
#endif

    /* Block on FIFO watermark interrupt (released by EXTI1 ISR on PB1).
     * Use 100ms timeout so we still poll the FIFO level periodically — this
     * also covers any interrupt we might miss during heavy SPI bus traffic. */
    if (s_lsm_fifo_sem == NULL ||
        osSemaphoreAcquire(s_lsm_fifo_sem, 100U) != osOK)
    {
      /* Timeout: still check FIFO — sensor may have produced data without us
       * catching the rising edge. */
    }

#if (APP_LSM_STALL_PROBE != 0U)
    uint32_t probe_sem_ms = HAL_GetTick() - probe_t_sem0;   /* 等信号量耗时(ms) */
    uint32_t probe_t_drain0 = HAL_GetTick();                /* 排空段起点(ms) */
#endif

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
#if (APP_LSM_STALL_PROBE != 0U)
      if (fifo_level > probe_fifo_max) probe_fifo_max = fifo_level;   /* 记录本轮 FIFO 峰值 */
#endif
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
            /* BIN 模式:acc/gyr 拆两条独立记录(各 22B、各自带 timestamp,自包含),
             * 共用同一 frame_id 做对齐键 → ACC_LOW / GYR_LOW 两文件(与 CSV 拆分一致)。
             * ★ 对齐保护(A2):仅当两环都有空间时才双写,否则整对丢弃。满配溢出时若
             * 只丢一侧会破坏 frame_id 成对性(实测两文件 frame_id 集合发散上万条),
             * 原子成对写保证两文件 frame_id 集合恒等——与环尺寸无关。 */
            if ((RingBuf_FreeSpace(&g_ring_lsm_acc) >= sizeof(SensorBinFrame_LSMAcc_t)) &&
                (RingBuf_FreeSpace(&g_ring_lsm_gyr) >= sizeof(SensorBinFrame_LSMGyr_t)))
            {
              SensorBinFrame_LSMAcc_t acc_frame;
              acc_frame.frame_id = fid;   /* ts 已去除:PC 端由 base+fid×interval 还原 */
              acc_frame.acc_x = cur_acc[0];
              acc_frame.acc_y = cur_acc[1];
              acc_frame.acc_z = cur_acc[2];
              acc_frame.crc32 = SensorBin_CalcCRC32(&acc_frame, sizeof(acc_frame) - 4U);
              RingBuf_Write(&g_ring_lsm_acc, (const uint8_t *)&acc_frame, sizeof(acc_frame));

              SensorBinFrame_LSMGyr_t gyr_frame;
              gyr_frame.frame_id = fid;   /* ts 已去除:PC 端由 base+fid×interval 还原 */
              gyr_frame.gyr_x = cur_gyr[0];
              gyr_frame.gyr_y = cur_gyr[1];
              gyr_frame.gyr_z = cur_gyr[2];
              gyr_frame.crc32 = SensorBin_CalcCRC32(&gyr_frame, sizeof(gyr_frame) - 4U);
              RingBuf_Write(&g_ring_lsm_gyr, (const uint8_t *)&gyr_frame, sizeof(gyr_frame));
            }
          }
          else
          {
            /* CSV 模式:acc/gyr 拆两行,共用同一 frame_id 做对齐键。
             *   acc 行: frame_id,datetime,ax,ay,az   (4 逗号) → g_ring_lsm_acc
             *   gyr 行: frame_id,gx,gy,gz             (3 逗号) → g_ring_lsm_gyr
             * 只有两行逗号数都正确才双写,保证每个 frame_id 在两文件里成对出现
             * (否则一坏一好会破坏 join 对齐)。 */
            char gyrbuf[96];
            uint32_t aoff = 0;
            aoff += AppU32ToDec(&rowbuf[aoff], fid);
            rowbuf[aoff++] = ',';
            memcpy(&rowbuf[aoff], dt_batch, 12U); aoff += 12U;
            rowbuf[aoff++] = ',';
            aoff += AppF1ToDec(&rowbuf[aoff], (float)cur_acc[0] * xl_s);
            rowbuf[aoff++] = ',';
            aoff += AppF1ToDec(&rowbuf[aoff], (float)cur_acc[1] * xl_s);
            rowbuf[aoff++] = ',';
            aoff += AppF1ToDec(&rowbuf[aoff], (float)cur_acc[2] * xl_s);
            rowbuf[aoff++] = '\r';
            rowbuf[aoff++] = '\n';

            uint32_t goff = 0;
            goff += AppU32ToDec(&gyrbuf[goff], fid);
            gyrbuf[goff++] = ',';
            goff += AppF1ToDec(&gyrbuf[goff], (float)cur_gyr[0] * g_s);
            gyrbuf[goff++] = ',';
            goff += AppF1ToDec(&gyrbuf[goff], (float)cur_gyr[1] * g_s);
            gyrbuf[goff++] = ',';
            goff += AppF1ToDec(&gyrbuf[goff], (float)cur_gyr[2] * g_s);
            gyrbuf[goff++] = '\r';
            gyrbuf[goff++] = '\n';

            /* Guard: acc 行须 4 逗号(5列)、gyr 行须 3 逗号(4列)。~0.5% 行会莫名多/
             * 缺逗号,双写前都校验;任一坏则整对丢弃(宁可丢一对也不破坏对齐/文件)。
             * ★ 对齐保护(A2):再加两环空间检查,仅当两环都容得下才双写,否则整对丢弃
             * ——满配溢出时只丢一侧会使 ACC_LOW/GYR_LOW 的 frame_id 集合发散。 */
            uint32_t anc = 0U, gnc = 0U;
            for (uint32_t k = 0U; k < aoff - 2U; k++) { if (rowbuf[k] == ',') anc++; }
            for (uint32_t k = 0U; k < goff - 2U; k++) { if (gyrbuf[k] == ',') gnc++; }
            if (anc == 4U && gnc == 3U &&
                (RingBuf_FreeSpace(&g_ring_lsm_acc) >= aoff) &&
                (RingBuf_FreeSpace(&g_ring_lsm_gyr) >= goff))
            {
              RingBuf_Write(&g_ring_lsm_acc, (const uint8_t *)rowbuf, aoff);
              RingBuf_Write(&g_ring_lsm_gyr, (const uint8_t *)gyrbuf, goff);
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

#if (APP_LSM_STALL_PROBE != 0U)
    /* 只读诊断收尾:本轮 drain 段耗时 + 整轮耗时。任一超阈值(排空>30ms 或整轮>60ms,
     * 6664Hz 下正常一轮 <~5ms)打印:相对启动秒 / 等信号量us / 排空us / 整轮us / 两环余量。
     * ★关键判据:若阻塞在"等信号量"(probe_sem_us 大)→ LSM 没被调度到(CPU饿死/被抢);
     * 若阻塞在"排空"(drain_us 大)→ FIFO读/解析/RingBuf_Write 里某步慢(SPI?环满自旋?)。 */
    uint32_t probe_drain_ms = HAL_GetTick() - probe_t_drain0;
    /* v3 触发:①相邻两轮间隔 cyc>50ms(正常~23ms,超说明LSM整轮被延迟)②FIFO峰值>480(逼近512满
     * →硬件FIFO溢出)③排空>50ms。全 ms 制,无回绕假阳性。 */
    if ((probe_cyc_ms > 50U) || (probe_fifo_max > 480U) || (probe_drain_ms > 50U))
    {
      printf("[LSMstall] t=%lu.%03lus cyc=%lums fifoMax=%u sem=%lums drain=%lums accFree=%lu gyrFree=%lu\r\n",
             (unsigned long)(probe_ms0 / 1000U),
             (unsigned long)(probe_ms0 % 1000U),
             (unsigned long)probe_cyc_ms, (unsigned)probe_fifo_max,
             (unsigned long)probe_sem_ms, (unsigned long)probe_drain_ms,
             (unsigned long)RingBuf_FreeSpace(&g_ring_lsm_acc),
             (unsigned long)RingBuf_FreeSpace(&g_ring_lsm_gyr));
    }
#endif
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
        bin_frame.frame_id = fid;   /* ts 已去除:PC 端由 base+fid×interval 还原 */
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
        bin_frame.frame_id = fid;   /* ts 已去除:PC 端由 base+fid×interval 还原 */
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

void StartLoggerTask(void *argument)
{
  FRESULT result;
  uint32_t rows_since_sync = 0U;
  uint8_t sd_file_open = 0U;
  uint32_t wav_next_ckpt = 0U;  /* B3: 下次 WAV 头 checkpoint 的 tick */
  uint32_t s_temp_last_tick = 0U;  /* ★2026-07-15 温度(TMP_LOW)限流:上次入队 APPENDFRAME 的 tick */
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
  /* A/B 开关:默认 0=轮询(零损坏已验证 CKBX0080/0081)。
   * ★2026-07-09 新试点 = IDMA + HWFC 组合(1U):调研查明 IDMA 损坏机制 = TX FIFO underrun
   * (IDMA 走 AHB 总线搬数据进发送 FIFO,满配时总线/中断挤占让 IDMA 送晚一拍→FIFO 空→卡仍按
   *  SDMMC_CK 收→错位字节;只在满配+高速通道损坏,完美对上 underrun 负载相关特征)。HWFC
   * (SDMMC_CLKCR 硬件流控,fbeb5c3 才加,当年 IDMA 测试时是关的!)会在 FIFO 将欠载时自动
   *  暂停 SDMMC_CK 等 IDMA 补上——正是硬件级根治 underrun 的机制。故"IDMA + HWFC 开"这个组合
   *  历史从未测过。H1 __DMB + H2 32B 对齐仍在(与本假设正交,保留)。
   * 测法:满配96k长录≥30min,读卡逐行比对坏行率(对照 CKBX0079 的 0.40%)。=0 → 损坏根治,
   *  IDMA 可安全用(注:仍不解决掉帧,掉帧瓶颈是卡 PROGRAMMING 非传输)。仍坏 → 彻底钉死。
   * 测完按结论决定是否保留;宏改回 0U 即恢复零损坏轮询路径。 */
#ifndef APP_SD_USE_IDMA
#define APP_SD_USE_IDMA 1U   /* ★IDMA+HWFC 损坏判别试点开启中(2026-07-09)。测完按结论回退/保留 */
#endif
  SD_SetDmaMode(APP_SD_USE_IDMA);
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  printf("[Logger] SDMMC %s mode\r\n",
         (APP_SD_USE_IDMA != 0U) ? "IDMA + HWFC (underrun-fix test)"
                                 : "POLLING (corruption-free SD writes)");

  /* ★SD 写吞吐基准(默认关):开 APP_SD_BENCH=1 时,logger 启动即跑一次裸卡写基准,
   * 打印各块大小×sync策略 MB/s 后死循环挂起(不进采集)。测完关此宏恢复正常。
   * 用于钉死"0.65MB/s 是卡物理墙 还是 我们写太碎/f_sync 拖累"。 */
#ifndef APP_SD_BENCH
#define APP_SD_BENCH 0U   /* 基准已测完(2026-07-10):卡6MB/s、交织5.8、f_sync无影响 → SD写不是瓶颈。恢复正常采集。 */
#endif
#if (APP_SD_BENCH != 0U)
  FatFs_SD_RunWriteBenchmark(s_block_pool[0], APP_SD_WRITE_BLOCK);   /* 复用块池[0]做源,内部死循环不返回 */
#endif

  for (;;)
  {
    AppAcqControl_t acq;
    uint8_t sd_session_active;
    uint8_t acq_running;

    /* ★事件驱动:阻塞等"数据就绪"唤醒,50ms 超时兜底。取代旧的 osDelay(2) 忙轮询 —— logger
     * 平时睡,生产者(RingBuf_Write 跨阈值,含 mic ISR)唤醒它。50ms 超时保证:低速环(env/mag)
     * 攒不满唤醒阈值时靠超时被扫到(走 LoggerDrainRing aged 分支)、周期 sync/WAV checkpoint、
     * 会话启停检测都不饿死(这些本是秒级周期,50ms 足够)。空闲期也靠超时每 50ms 醒一次跑会话
     * 检查(替代原 idle 的 osDelay(10))。借鉴 DATALOG1 SDM_Thread osMessageGet 阻塞唤醒。 */
    (void)osSemaphoreAcquire(s_logger_wake, 50U);

    AppAcqCheckAutoStop();
    AppAcqGetCopy(&acq);
    acq_running = acq.running;

    uint8_t sd_acq_active = (uint8_t)((acq_running != 0U) && (acq.sink == APP_ACQ_SINK_SD));

    /* 非阈值模式:直接按采集状态决定是否开 SD 会话 */
    sd_session_active = sd_acq_active;

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
         * and frame_id/tick_ms in the new session align with real samples. */
        RingBuf_Reset(&g_ring_lsm_acc);
        RingBuf_Reset(&g_ring_lsm_gyr);
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
        wav_next_ckpt = osKernelGetTickCount() + 30000U;  /* B3: 首个 WAV checkpoint 在 ~30s 后 */
        s_temp_last_tick = 0U;   /* ★温度限流按会话清零:0=首帧立即写(会话起点留一条温度) */
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

    /* 机会式写入门控(SdFat 思路,降满配掉帧):卡在 PROGRAMMING/GC 忙时,发起 SD 写会
     * 死等其退出(掉帧根因——drain 串行堆积、慢通道环溢出)。改为:写前先查 SD_IsCardBusy()
     * (读 BUSYD0 硬件位,非阻塞),卡忙则本轮不写、短让出让传感器继续把数据攒进环,只在卡
     * 空闲的间隙 drain → 消除"等卡"浪费。默认开启;定 APP_SD_OPPORTUNISTIC=0 可回退到原
     * 无条件 drain 做 A/B 对照。见 docs/.../2026-07-08-dropframe-opportunistic-write.md。 */
/* ★ 实测已证无效(2026-07-09):满配持续高压下卡几乎一直忙,门控只是频繁 break+osDelay
 * 空转,反让掉帧更差(H3 13→15%/QMA 19→22%)。默认关闭。真根因是写太碎→PROGRAMMING
 * 次数多(见 F1/F3),不是"等卡浪费 CPU"。保留开关供参考,勿开。 */
#ifndef APP_SD_OPPORTUNISTIC
#define APP_SD_OPPORTUNISTIC 0U
#endif

    /* Round-robin drain of the four ring buffers. Each chunk is copied into a
     * free pool block (inside LoggerDrainRing) then handed to the writer task, so
     * the SDMMC IDMA never reads from a live ring — see the helper's comment. Every
     * ring is checked each cycle so no stream starves under heavy load. */
    {
      /* 表驱动 round-robin drain(消除 7 处重复,并支持每个 ring 写前的机会式门控)。
       * 顺序与原实现一致:LSM_ACC, LSM_GYR, QMA, H3, MIC, AHT_ENV, MAG。 */
      static const struct { AppRingBuffer_t *rb; uint8_t file_idx; const char *name; }
        k_drain_list[] = {
          { &g_ring_lsm_acc, 0U,                     "LSM_ACC" },
          { &g_ring_lsm_gyr, FATFS_SD_FILE_LSM_GYR,  "LSM_GYR" },
          { &g_ring_qma_acc, 3U,                     "QMA_ACC" },
          { &g_ring_h3_acc,  2U,                     "H3_ACC"  },
          { &g_ring_mic,     FATFS_SD_FILE_MIC_WAV,  "MIC.WAV" },
          { &g_ring_aht_env, 4U,                     "AHT_ENV" },
          { &g_ring_mag,     5U,                     "MAG"     },
        };
      const uint32_t k_drain_n = sizeof(k_drain_list) / sizeof(k_drain_list[0]);

      uint8_t did_work = 1;
      uint8_t write_failed = 0;
#if (APP_LSM_STALL_PROBE != 0U)
      /* ★2026-07-14 整个 drain 循环计时(补"千刀万剐"盲区)。SLC耗尽后单次写~130ms 卡在
       * [LoggerBlk] 的 150ms 阈值下不触发,但一轮里多次 130ms 写连续堆积(logger 在此 while
       * 里连写不睡)会占住数百 ms~秒级,期间 LSM 环无人排空→溢出。这里统计本轮:总耗时/
       * 写次数/写字节数,并记录 LSM 两环进入时的余量。判据:
       *   loop耗时大 + 写多次/字节多 → 千刀万剐持续慢写(带宽缺口,治标只能降产出/换卡);
       *   loop耗时大 + 写极少/字节少 → logger 被高优先级任务抢占饿死(治标提 logger 优先级)。*/
      uint32_t probe_loop0 = HAL_GetTick();
      uint32_t probe_loop_writes = 0U;             /* 本轮实际 f_write 次数(dr>0) */
      uint32_t probe_loop_rss0 = rows_since_sync;  /* 进入本轮的累计字节基准(LoggerDrainRing 每写 +=n) */
      uint32_t probe_loop_acc_free0 = RingBuf_FreeSpace(&g_ring_lsm_acc);
      uint32_t probe_loop_gyr_free0 = RingBuf_FreeSpace(&g_ring_lsm_gyr);
#endif
      while (did_work && (sd_file_open != 0U) && (AppAcqIsSdSessionActive() != 0U))
      {
        did_work = 0;
        for (uint32_t i = 0U; i < k_drain_n; i++)
        {
#if (APP_SD_OPPORTUNISTIC != 0U)
          /* 机会式门控:每个 ring 发起写前查卡忙(读 BUSYD0 硬件位,非阻塞)。卡在 GC/
           * PROGRAMMING 忙时不阻塞式发起写(那会死等,是掉帧根因),直接跳出本轮 drain,
           * 数据留在环里,下方 osDelay 短让出后重来。逐 ring 查(非仅轮首)可避免"上一次
           * 写的 PROGRAMMING 未完就阻塞发起下一 ring 写"。绝不 spin 空转等卡。 */
          if (SD_IsCardBusy() != 0U)
          {
            break;
          }
#endif
          /* V2 判别实验(默认关):把所有 ring 的写强制指向同一个普通文件(idx 0),
           * 总产出满配不变、文件数 7→1 → 隔离"多文件交织 vs 单文件顺序"这一个变量。
           * 若 V2 下掉帧清零 → 坐实 648KB/s 是交织退化成随机写、非卡物理上限。
           * ⚠️ 数据是 7 路混写的乱数据,仅测吞吐,不可留用;测完把 APP_SD_V2 设 0 回退。
           * 见 docs/.../2026-07-08-dropframe-opportunistic-write.md §10。 */
#ifndef APP_SD_V2
#define APP_SD_V2 0U   /* V2判别实验已结束(2026-07-09满配):证伪交织假设,单文件顺序写下
                        * h3/qma/mic 照样重丢 → 648KB/s 是真实持续写上限,非交织退化。已回退。
                        * 详见 docs/.../2026-07-08-dropframe-opportunistic-write.md §10。 */
#endif
#if (APP_SD_V2 != 0U)
          uint8_t v2_idx = 0U;   /* 全写 g_log_files[0] 单文件 */
          int dr = LoggerDrainRing(k_drain_list[i].rb, v2_idx,
                                   APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
#else
#if (APP_LSM_STALL_PROBE != 0U)
          uint32_t probe_wr0 = HAL_GetTick();
#endif
          int dr = LoggerDrainRing(k_drain_list[i].rb, k_drain_list[i].file_idx,
                                   APP_RING_FLUSH_CHUNK, &rows_since_sync, &result);
#if (APP_LSM_STALL_PROBE != 0U)
          uint32_t probe_wr_ms = HAL_GetTick() - probe_wr0;
          if (probe_wr_ms > 150U)   /* 单次 f_write(≤64KB)阻塞——正常几ms,>150ms=卡在这次写 */
            printf("[LoggerBlk] %s write %lums @t=%lu.%03lus\r\n",
                   k_drain_list[i].name, (unsigned long)probe_wr_ms,
                   (unsigned long)(probe_wr0 / 1000U), (unsigned long)(probe_wr0 % 1000U));
#endif
#endif  /* APP_SD_V2 */
          if (dr < 0)
          {
            printf("[Logger] %s write fail %s (%d)\r\n",
                   k_drain_list[i].name, FatFs_SD_ResultToString(result), (int)result);
            AppFlowStatsRecordWriteFailure();
            write_failed = 1;
            break;
          }
          if (dr > 0) did_work = 1;
#if (APP_LSM_STALL_PROBE != 0U)
          if (dr > 0) probe_loop_writes++;
#endif
        }
        if (write_failed) break;
      }
#if (APP_LSM_STALL_PROBE != 0U)
      /* 整轮 drain 结束:超 200ms 打印(6664Hz 下 LSM 环 160KB=1.0s 缓冲,一轮>200ms 就吃掉
       * 20% 余量,连续几轮就溢出)。writes/bytes 区分"持续慢写堆积" vs "被抢占饿死"。
       * accFree/gyrFree 打进入本轮时的余量,gap 前应看到余量骤降。 */
      {
        uint32_t probe_loop_ms = HAL_GetTick() - probe_loop0;
        uint32_t probe_loop_bytes = rows_since_sync - probe_loop_rss0;  /* 本轮写出字节(rss 单调增,循环内不清零) */
        if (probe_loop_ms > 200U)
          printf("[LoggerLoop] %lums writes=%lu bytes=%lu accFree0=%lu gyrFree0=%lu @t=%lu.%03lus\r\n",
                 (unsigned long)probe_loop_ms,
                 (unsigned long)probe_loop_writes, (unsigned long)probe_loop_bytes,
                 (unsigned long)probe_loop_acc_free0, (unsigned long)probe_loop_gyr_free0,
                 (unsigned long)(probe_loop0 / 1000U), (unsigned long)(probe_loop0 % 1000U));
      }
#endif

      /* ★事件驱动:原 osDelay(2) 攒批节奏已移除。本轮 while(did_work) 把所有达阈值的环
       * 写完后自然退出,落到下方周期 sync/checkpoint,再回到循环顶部 osSemaphoreAcquire 阻塞
       * 睡 → 由生产者跨阈值唤醒或 50ms 超时唤醒。攒批仍由 LoggerDrainRing 的 3/4环 gate 保证
       * (不满阈值不写,继续攒),不再靠固定延时。ring 溢出由生产者侧 drop 统计。 */
    }

    /* Periodic metadata flush: commit FAT + directory entries every
     * APP_SD_SYNC_INTERVAL_BYTES of payload so an extreme-load write-integrity
     * failure (the 6664+96k FAT/dir loss — empty session dir) can't orphan the
     * whole session. rows_since_sync accumulates drained bytes (LoggerDrainRing
     * += n).
     * ★2026-07-12 回退到 1MB(原值)。曾于 07-10 改 16MB 想减 FAT 碎写降掉帧,但实测
     * (CTBX_2026-07-12-04-16)酿成严重数据安全 bug:长录 3.79h 写 9.5GB 后低电量关机,
     * 目录项/FAT 元数据提交太稀疏 → 掉电前来不及补写整会话的目录+FAT(15万簇~600KB) →
     * 半提交 → 整会话目录 0 文件项、9.5GB 全成孤儿簇丢失。
     * ★2026-07-13 1MB→4MB 实验【已证伪并回退到 1MB】。原以为 1MB sync 的 FAT/目录碎写吃带宽
     * 是掉帧主因,改 4MB 想回收带宽。但 CTBX-13-05(4MB)对照 02-55(1MB)实测:掉帧无改善反略升
     * (LSM 9.5%→10.1% / QMA 5.5%→6.1%),实效写速 726→714KB/s 未升。审计揭真相:cnt=1 碎写
     * 598 次里 539 次是【高地址数据区写】,仅 59 次是低地址 FAT/目录——sync 只影响那 59 次,杯水
     * 车薪。因果是反的:不是碎写拖慢卡,是卡 SLC 缓存耗尽后持续写速仅 ~714KB/s < CSV 满载生成
     * 730KB/s,产出比卡能写的多 16KB/s → logger 被每次写阻塞 125ms、环攒不满 32KB 就超时兜底写
     * 小块。这是带宽物理缺口,调度层动不了。掉帧真解只有 BIN 降产出(519KB/s,富余 195)或换更快卡。
     * 故回退 1MB:sync 间隔既不是掉帧因,就选掉电更安全的 1MB(硬掉电最多丢 2s 元数据)。 */
#ifndef APP_SD_SYNC_INTERVAL_BYTES
#define APP_SD_SYNC_INTERVAL_BYTES  (1U * 1024U * 1024U)
#endif
    if ((sd_file_open != 0U) && (rows_since_sync >= APP_SD_SYNC_INTERVAL_BYTES))
    {
      AppSdWriteMsg_t m = { 0 };
      m.type = (uint8_t)APP_SDMSG_SYNC;
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
      rows_since_sync = 0U;
    }

    /* B3: 每 ~30s 回填一次 WAV 头(chunk_size/data_size)并 f_sync,使 MIC.WAV 在任意
     * 时刻都是可播放的合法文件。硬掉电(低电量漏检测)时音频最多丢最后 ~30s——独立于
     * 电压检测的保险。计时用 tick(慢 ~19% → 实际约 37s,不影响保险功能)。 */
    if ((sd_file_open != 0U) && ((int32_t)(osKernelGetTickCount() - wav_next_ckpt) >= 0))
    {
      AppSdWriteMsg_t m = { 0 };
      m.type = (uint8_t)APP_SDMSG_WAVCKPT;
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
      wav_next_ckpt = osKernelGetTickCount() + 30000U;
    }

    /* ★2026-07-15 温度(TMP_LOW)限流到 ~1Hz。逐帧路径原本每次 LSM FIFO drain(6664Hz链)都
     * 压一帧→写线程 f_write 一行 ~15B 温度 CSV→FatFs 每攒 512B 刷一个扇区→海量 cnt=1 单扇区写
     * (实测占 52% SD写预算、437s)。温度恒变极慢(37℃级),1Hz 足够。门控:距上次入队<1s 的帧仍
     * pop 排空缓冲(防积压)但不入队、不落盘。cnt=1 碎写由此几乎消失,SD 写带宽回收给数据通道。 */
    while ((sd_file_open != 0U) && (AppAcqIsSdSessionActive() != 0U) && (AppFrameBufferPop(&frame) != 0U))
    {
      uint32_t now_tick = osKernelGetTickCount();
      if ((s_temp_last_tick == 0U) ||
          ((uint32_t)(now_tick - s_temp_last_tick) >= APP_TEMP_LOG_INTERVAL_TICKS))
      {
        s_temp_last_tick = now_tick;
        AppSdWriteMsg_t m;
        m.type = (uint8_t)APP_SDMSG_APPENDFRAME;
        m.block_idx = 0U; m.file_idx = 0U; m.len = 0U;
        m.frame = frame;
        (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
        rows_since_sync++;
        AppFlowStatsRecordFrameWrite(&frame);
      }
      /* else: 帧已 pop(排空缓冲),温度这行丢弃——1Hz 之间的温度无价值 */
    }

    if ((sd_file_open != 0U) && (AppAcqDrainPendingStop() != 0U))
    {
      /* 收尾排空帧缓冲:同样限流温度(缓冲可能积压上千帧,不限流会在停止时突发写上千行温度,
       * 重现 cnt=1 碎写)。仍 pop 排空,只是 1Hz 之外的温度行不入队。 */
      while (AppFrameBufferPop(&frame) != 0U)
      {
        uint32_t now_tick = osKernelGetTickCount();
        if ((s_temp_last_tick == 0U) ||
            ((uint32_t)(now_tick - s_temp_last_tick) >= APP_TEMP_LOG_INTERVAL_TICKS))
        {
          s_temp_last_tick = now_tick;
          AppSdWriteMsg_t m;
          m.type = (uint8_t)APP_SDMSG_APPENDFRAME;
          m.block_idx = 0U; m.file_idx = 0U; m.len = 0U;
          m.frame = frame;
          (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
          rows_since_sync++;
          AppFlowStatsRecordFrameWrite(&frame);
        }
      }

      /* final sync 经队列(会在 STOP 前被写线程按 FIFO 执行);随后 AppLoggerStopSdSession
       * 会做屏障+STOP 完成真正收尾。 */
      {
        AppSdWriteMsg_t m = { 0 };
        m.type = (uint8_t)APP_SDMSG_SYNC;
        (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
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
