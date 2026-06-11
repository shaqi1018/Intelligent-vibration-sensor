#include "diskio.h"

#include "sd_diskio.h"
#include "rtc_pcf85063.h"
#include "app_time.h"

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

/* FatFs 文件时间戳回调。
 * 时间来源：app_time 缓存（不直读 I2C），避免与 ES8311 共用 I2C2 导致时间戳
 * 整批回退。get_fattime 被高频调用（每次 open/sync/close），用缓存永不失败。
 * 返回打包的 DWORD：bit31-25 年-1980 | bit24-21 月 | bit20-16 日 |
 *   bit15-11 时 | bit10-5 分 | bit4-0 秒/2 */
DWORD get_fattime(void)
{
  uint32_t epoch = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
  if (epoch < 946684800U) { epoch = 946684800U; }
  Pcf85063_Time_t t;
  Pcf85063_FromEpochSeconds(epoch, &t);
  return ((DWORD)(2000U + (uint32_t)t.year - 1980U) << 25) |
         ((DWORD)t.month  << 21) |
         ((DWORD)t.day    << 16) |
         ((DWORD)t.hour   << 11) |
         ((DWORD)t.minute << 5)  |
         ((DWORD)(t.second >> 1));
}
