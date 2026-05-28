#ifndef __FATFS_SD_H__
#define __FATFS_SD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "ff.h"
#include "sensor_snapshot.h"

const char *FatFs_SD_ResultToString(FRESULT result);
FRESULT FatFs_SD_Mount(void);
void FatFs_SD_Unmount(void);
void FatFs_SD_ForceReinit(void);
const char *FatFs_SD_GetSessionDir(void);
FRESULT FatFs_SD_LoggerStart(void);
FRESULT FatFs_SD_LoggerAppendFrame(const AppSensorFrame_t *frame);
/* Write a raw byte stream to log file `idx`
 *   0=LSM_IMU, 1=LSM_TMP, 2=H3_ACC, 3=QMA_ACC
 * Used by the ring buffer flush path in StartLoggerTask. */
FRESULT FatFs_SD_LoggerWriteFileIndex(uint8_t idx, const uint8_t *data, uint32_t len);
FRESULT FatFs_SD_LoggerSync(void);
void FatFs_SD_LoggerStop(void);
void FatFs_SD_RunPhaseBSmokeTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __FATFS_SD_H__ */
