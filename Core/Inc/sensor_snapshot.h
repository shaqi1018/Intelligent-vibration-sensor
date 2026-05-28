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

typedef enum {
  APP_SENSOR_ID_LSM6DSOX = 0,
  APP_SENSOR_ID_H3LIS100DL = 1,
  APP_SENSOR_ID_QMA6100P = 2,
} AppSensorId_t;

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

/* === LSM batch frame ====================================================
 * One batch represents up to N consecutive ACC/GYR sample pairs that came
 * from a single FIFO drain at a fixed BDR. Storing them as a batch lets the
 * logger emit the rows with one large write call per file, instead of
 * thousands of tiny writes that overwhelm the FAT layer at 6664 Hz.
 *
 * Reconstruction in the logger:
 *   sample i tick_ms = base_tick_ms - (n_pairs-1-i) * period_us / 1000
 */
#define APP_LSM_BATCH_MAX_PAIRS         128U
#define APP_LSM_BATCH_BUFFER_DEPTH      8U

typedef struct {
  uint32_t base_frame_id;             /* frame_id of the LAST sample in the batch */
  uint32_t base_tick_ms;              /* tick_ms when the batch was read */
  uint32_t period_us;                 /* 1/BDR in us — used for back-extrapolation */
  uint16_t n_pairs;                   /* number of valid sample pairs in this batch */
  int16_t  acc[APP_LSM_BATCH_MAX_PAIRS][3];   /* raw int16 ACC samples */
  int16_t  gyro[APP_LSM_BATCH_MAX_PAIRS][3];  /* raw int16 GYR samples */
  float    acc_sensitivity;           /* mg/LSB at the time of capture */
  float    gyro_sensitivity;          /* mdps/LSB at the time of capture */
} AppLsmBatch_t;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SNAPSHOT_H__ */
