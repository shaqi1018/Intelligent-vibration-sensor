#ifndef __SENSOR_SNAPSHOT_H__
#define __SENSOR_SNAPSHOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lsm6dsox.h"
#include "h3lis100dl.h"
#include "qma6100p.h"

#define APP_SENSOR_SAMPLE_PERIOD_MS     1U
#define APP_SENSOR_STALE_TIMEOUT_MS     (APP_SENSOR_SAMPLE_PERIOD_MS * 2U)
#define APP_SENSOR_COHERENT_WINDOW_MS   APP_SENSOR_SAMPLE_PERIOD_MS
#define APP_SENSOR_FRAME_BUFFER_DEPTH   512U

#define APP_SENSOR_MASK_LSM6DSOX        (1UL << 0)
#define APP_SENSOR_MASK_H3LIS100DL      (1UL << 1)
#define APP_SENSOR_MASK_QMA6100P        (1UL << 2)
#define APP_SENSOR_MASK_ALL             (APP_SENSOR_MASK_LSM6DSOX | APP_SENSOR_MASK_H3LIS100DL | APP_SENSOR_MASK_QMA6100P)

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  uint32_t last_update_ms;
  LSM6DSOX_AllData_t data;
} AppLsm6dsoxSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  uint32_t last_update_ms;
  H3LIS100DL_Data_t data;
} AppH3lis100dlSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  uint32_t last_update_ms;
  QMA6100P_Data_t data;
} AppQma6100pSnapshot_t;

typedef struct {
  AppLsm6dsoxSnapshot_t lsm6dsox;
  AppH3lis100dlSnapshot_t h3lis100dl;
  AppQma6100pSnapshot_t qma6100p;
} AppSensorSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  LSM6DSOX_AllData_t data;
} AppFrameLsm6dsox_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  H3LIS100DL_Data_t data;
} AppFrameH3lis100dl_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  QMA6100P_Data_t data;
} AppFrameQma6100p_t;

typedef struct {
  uint32_t frame_id;
  uint32_t tick_ms;
  uint32_t enabled_mask;
  uint32_t present_mask;
  AppFrameLsm6dsox_t lsm6dsox;
  AppFrameH3lis100dl_t h3lis100dl;
  AppFrameQma6100p_t qma6100p;
} AppSensorFrame_t;

/* === Lock-free SPSC ring buffer ============================================
 * Single producer (sensor task) calls RingBuf_Write to append CSV bytes.
 * Single consumer (logger) calls RingBuf_PeekContiguous + RingBuf_Consume to
 * flush via one large f_write per file. wr_idx/rd_idx are volatile single
 * 32-bit writes so no mutex is needed. The buffer is full when
 * (wr_idx + 1) % size == rd_idx (one byte sentinel reserved).
 *
 * The data array is provided externally (statically allocated in .bss).
 */
typedef struct {
  uint8_t *data;
  uint32_t size;
  volatile uint32_t wr_idx;
  volatile uint32_t rd_idx;
  volatile uint32_t dropped;          /* bytes dropped due to full buffer */
  volatile uint32_t high_watermark;   /* peak fill level for diagnostics */
} AppRingBuffer_t;

#define APP_RING_LSM_IMU_SIZE   (128U * 1024U)  /* 6664Hz×60B/row≈400KB/s；128KB≈0.32s，覆盖SD最坏擦写延迟 */
#define APP_RING_QMA_ACC_SIZE   (32U * 1024U)
#define APP_RING_H3_ACC_SIZE    (16U * 1024U)   /* 400Hz × 14B/row × 5s ≈ 28KB cap, 16KB enough with logger drain */
/* 48kHz×2B = 96KB/s；32KB ≈ 0.33s 缓冲，足够吸收 SD PROGRAMMING 抖动 */
#define APP_RING_MIC_SIZE       (32U * 1024U)
/* Largest contiguous chunk handed to f_write per call. Bigger = fewer FAT
 * cluster traversals; smaller = lower latency. 16KB is a good trade-off. */
#define APP_RING_FLUSH_CHUNK    (16U * 1024U)

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SNAPSHOT_H__ */
