#ifndef __SD_DISKIO_H__
#define __SD_DISKIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "diskio.h"

#define SDDISKIO_DRIVE_NUM    0U
#define SDDISKIO_SECTOR_SIZE  512U

DSTATUS SD_disk_status(BYTE pdrv);
DSTATUS SD_disk_initialize(BYTE pdrv);
DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);
void SD_SetDmaMode(unsigned char enable);

/* Route-2 SDMMC write-path diagnostics. SD_ResetWriteStats clears the counters
 * (call at SD session start); SD_PrintWriteStats emits a one-line [SDstat]
 * summary over UART (call at session stop). */
void SD_ResetWriteStats(void);
void SD_PrintWriteStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __SD_DISKIO_H__ */
