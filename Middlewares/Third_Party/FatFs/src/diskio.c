#include "diskio.h"

#include "sd_diskio.h"

DSTATUS disk_status(BYTE pdrv)
{
  return SD_disk_status(pdrv);
}

DSTATUS disk_initialize(BYTE pdrv)
{
  return SD_disk_initialize(pdrv);
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  return SD_disk_read(pdrv, buff, sector, count);
}

#if _USE_WRITE == 1
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  return SD_disk_write(pdrv, buff, sector, count);
}
#endif

#if _USE_IOCTL == 1
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  return SD_disk_ioctl(pdrv, cmd, buff);
}
#endif

DWORD get_fattime(void)
{
  return ((DWORD)(2026U - 1980U) << 25) |
         ((DWORD)1U << 21) |
         ((DWORD)1U << 16);
}
