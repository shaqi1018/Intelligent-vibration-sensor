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

#ifdef __cplusplus
}
#endif

#endif /* __SD_DISKIO_H__ */
