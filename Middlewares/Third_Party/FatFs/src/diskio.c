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
 * 返回打包的 DWORD：
 *   bit31-25 年-1980 | bit24-21 月(1-12) | bit20-16 日(1-31)
 *   bit15-11 时(0-23) | bit10-5 分(0-59) | bit4-0 秒/2(0-29)
 * 仅当 ffconf.h 中 _FS_NORTC==0 时 ff.c 才会调用本函数。
 *
 * 时间来源：app_time 缓存（开机 + set_time 时各读一次 RTC 锚定，之后用 DWT
 * 推算，纯内存运算）。get_fattime 被 FatFs 高频调用（每个文件 open/sync/close），
 * 若每次直读 I2C 的 RTC，会与 ES8311（共用 I2C2 总线）抢总线、间歇失败，导致
 * 同一会话里部分文件时间戳整批回退。改用缓存后永不失败，且与 CSV datetime 同源，
 * 文件时间与内容时间一致。 */
DWORD get_fattime(void)
{
  uint32_t epoch = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
  /* Pcf85063_FromEpochSeconds 的年份字段以 2000 为基准，要求 epoch≥2000-01-01；
   * 未同步(epoch≈0)时钳到 2000-01-01，避免年份字段下溢成非法 FAT 时间戳。 */
  if (epoch < 946684800U) { epoch = 946684800U; }
  Pcf85063_Time_t t;
  Pcf85063_FromEpochSeconds(epoch, &t);   /* 纯计算，不访问 I2C */
  return ((DWORD)(2000U + (uint32_t)t.year - 1980U) << 25) |
         ((DWORD)t.month  << 21) |
         ((DWORD)t.day    << 16) |
         ((DWORD)t.hour   << 11) |
         ((DWORD)t.minute << 5)  |
         ((DWORD)(t.second >> 1));
}
