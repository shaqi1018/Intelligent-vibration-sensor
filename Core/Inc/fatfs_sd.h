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
FRESULT FatFs_SD_LoggerStart(void);
FRESULT FatFs_SD_LoggerAppendRow(const AppSensorSnapshot_t *snapshot, uint32_t tick_ms, uint32_t row_seq);
FRESULT FatFs_SD_LoggerSync(void);
void FatFs_SD_LoggerStop(void);
void FatFs_SD_RunPhaseBSmokeTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __FATFS_SD_H__ */
