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
const char *FatFs_SD_GetSessionDir(void);
FRESULT FatFs_SD_LoggerStart(void);
FRESULT FatFs_SD_LoggerAppendFrame(const AppSensorFrame_t *frame);
/* Write a raw byte stream to log file `idx`
 *   0=ACC_LOW, 1=TMP_LOW, 2=ACC_HIGH, 3=ACC_MID, 4=ENV, 5=MAG, 6=GYR_LOW
 * 实际文件名带 3 位段号(如 ACC_LOW001.CSV)。Used by the ring buffer flush path. */
FRESULT FatFs_SD_LoggerWriteFileIndex(uint8_t idx, const uint8_t *data, uint32_t len);

/* 低量程六轴的加速度/角速度拆两个文件:索引 0=ACC_LOW(frame_id,datetime,acc_xyz)、
 * 6=GYR_LOW(frame_id,gyr_xyz),同一采样对共用 frame_id 做对齐键。 */
#define FATFS_SD_FILE_LSM_GYR   6U

/* MIC.WAV —— 文件索引 7。因需开头占位头、停止时回填大小，单独用一个
 * FIL + 字节计数器管理，不走 CSV 的 g_log_files[] 数组。 */
#define FATFS_SD_FILE_MIC_WAV   7U
FRESULT FatFs_SD_WavCreate(uint32_t sample_rate_hz, uint16_t bits);
FRESULT FatFs_SD_WavFinalize(void);

FRESULT FatFs_SD_LoggerSync(void);
void FatFs_SD_LoggerStop(void);
/* 1=logger 仍持有打开文件;0=全部文件已 finalize/sync/close(可安全断电/复位)。 */
uint8_t FatFs_SD_IsLoggerActive(void);
void FatFs_SD_RunPhaseBSmokeTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __FATFS_SD_H__ */
