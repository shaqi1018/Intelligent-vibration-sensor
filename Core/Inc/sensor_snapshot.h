#ifndef __SENSOR_SNAPSHOT_H__
#define __SENSOR_SNAPSHOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lsm6dsox.h"
#include "h3lis100dl.h"
#include "qma6100p.h"

#define APP_SENSOR_SAMPLE_PERIOD_MS   500U
#define APP_SENSOR_STALE_TIMEOUT_MS   (APP_SENSOR_SAMPLE_PERIOD_MS * 2U)

typedef struct {
  uint8_t valid;
  uint32_t last_update_ms;
  LSM6DSOX_AllData_t data;
} AppLsm6dsoxSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t last_update_ms;
  H3LIS100DL_Data_t data;
} AppH3lis100dlSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t last_update_ms;
  QMA6100P_Data_t data;
} AppQma6100pSnapshot_t;

typedef struct {
  AppLsm6dsoxSnapshot_t lsm6dsox;
  AppH3lis100dlSnapshot_t h3lis100dl;
  AppQma6100pSnapshot_t qma6100p;
} AppSensorSnapshot_t;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SNAPSHOT_H__ */
