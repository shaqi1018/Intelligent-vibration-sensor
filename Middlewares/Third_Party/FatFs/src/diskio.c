#include "diskio.h"

#include "sd_diskio.h"
#include "rtc_pcf85063.h"

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

/* FatFs 文件时间戳回调：从 PCF85063 RTC 取墙钟时间。
 * 返回打包的 DWORD：
 *   bit31-25 年-1980 | bit24-21 月(1-12) | bit20-16 日(1-31)
 *   bit15-11 时(0-23) | bit10-5 分(0-59) | bit4-0 秒/2(0-29)
 * 仅当 ffconf.h 中 _FS_NORTC==0 时 ff.c 才会调用本函数。 */
DWORD get_fattime(void)
{
  Pcf85063_Time_t t;
  if (Pcf85063_GetTime(&t) != PCF85063_OK)
  {
    /* RTC 读取失败：回退到固定时间，避免写入非法时间戳 */
    return ((DWORD)(2026U - 1980U) << 25) |
           ((DWORD)1U << 21) |
           ((DWORD)1U << 16);
  }
  /* PCF85063: year 为 0-99（+2000 得公历年），month 1-12，24 小时制 */
  return ((DWORD)(2000U + (uint32_t)t.year - 1980U) << 25) |
         ((DWORD)t.month  << 21) |
         ((DWORD)t.day    << 16) |
         ((DWORD)t.hour   << 11) |
         ((DWORD)t.minute << 5)  |
         ((DWORD)(t.second >> 1));
}
