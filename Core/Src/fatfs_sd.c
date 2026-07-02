#include "fatfs_sd.h"

#include <stdio.h>
#include <string.h>

#include "diskio.h"
#include "sd_diskio.h"
#include "sdmmc.h"
#include "acq_config.h"  /* ACQ_OUTPUT_CSV / ACQ_OUTPUT_BIN */
#include "app_time.h"       /* AppTime_GetEpochUs — 会话目录名的 RTC 时间来源 */
#include "rtc_pcf85063.h"   /* Pcf85063_Time_t / Pcf85063_FromEpochSeconds */

/* 会话目录名：CTBX_YYYY-MM-DD-HH-MM(RTC 实时时间)，如 CTBX_2026-06-24-14-30。
 * 21 字符超 8.3 短名上限，依赖 FatFs 长文件名(ffconf.h _USE_LFN=1)。
 * 同一分钟内重复开会话时追加 _n(n=1..99)去重。 */
#define FATFS_SD_SESSION_PREFIX   "CTBX"
#define FATFS_SD_DIR_PATH_MAX     32U   /* "0:/CTBX_2026-06-24-14-30_99" = 27 + NUL */
#define FATFS_SD_FILE_PATH_MAX    48U   /* dir(27) + "/IMU0002.CSV"(12) = 39 + NUL */

/* 6 个 CSV 文件名（8.3 格式）.
 * LSM_IMU merges accel+gyro at 6664Hz; LSM_TMP holds the slow temperature
 * channel; H3 and QMA are unchanged; AHT_ENV and MAG are new. */
#define FATFS_SD_FNAME_LSM_IMU   "LSM_IMU.CSV"
#define FATFS_SD_FNAME_LSM_TMP   "LSM_TMP.CSV"
#define FATFS_SD_FNAME_H3_ACC    "H3_ACC.CSV"
#define FATFS_SD_FNAME_QMA_ACC   "QMA_ACC.CSV"
#define FATFS_SD_FNAME_AHT_ENV   "AHT_ENV.CSV"   /* 新增：索引 4 */
#define FATFS_SD_FNAME_MAG       "MAG.CSV"        /* 新增：索引 5 */
#define FATFS_SD_FNAME_MIC_WAV   "MIC.WAV"

#define FATFS_SD_FILE_LSM_IMU     0U
#define FATFS_SD_FILE_LSM_TMP     1U
#define FATFS_SD_FILE_H3_ACC      2U
#define FATFS_SD_FILE_QMA_ACC     3U

#define FATFS_SD_NUM_FILES        6U  /* 0..5 CSV; MIC_WAV 独立索引6 */

static FATFS g_sd_fatfs;
static FIL g_log_files[FATFS_SD_NUM_FILES];
static char g_session_dir[FATFS_SD_DIR_PATH_MAX];
static uint8_t g_sd_mounted = 0U;
static uint8_t g_logger_active = 0U;
static uint32_t g_logger_rows_written = 0U;

/* ---- 文件分段(segmentation) ----
 * 6 个传感器数据文件(索引 0..5)各自独立按字节数滚动；MIC.WAV(索引6)不分段。
 * 段1沿用原文件名(如 LSM_IMU.CSV)；续段用短标签+4位段号(如 IMU0002.CSV)，
 * 满足 8.3 短名限制(_USE_LFN=0)。CSV 续段重写表头，BIN 续段无头(同首段)。*/
static uint32_t g_seg_max_bytes = 0U;                    /* 0=不分段；否则每段字节上限 */
static uint8_t  g_log_format    = ACQ_OUTPUT_CSV;        /* LoggerStart 时缓存的输出格式 */
static uint32_t g_file_bytes[FATFS_SD_NUM_FILES] = {0};  /* 当前段已写字节数 */
static uint16_t g_file_seg[FATFS_SD_NUM_FILES]   = {0};  /* 当前段号(1 基) */

static const char *const kLogFnames[FATFS_SD_NUM_FILES] = {
  FATFS_SD_FNAME_LSM_IMU, FATFS_SD_FNAME_LSM_TMP,
  FATFS_SD_FNAME_H3_ACC,  FATFS_SD_FNAME_QMA_ACC,
  FATFS_SD_FNAME_AHT_ENV, FATFS_SD_FNAME_MAG
};
/* 续段短标签：≤4 字符，拼 4 位段号后 ≤8 字符，满足 8.3 */
static const char *const kLogTags[FATFS_SD_NUM_FILES] = {
  "IMU", "TMP", "H3_", "QMA", "ENV", "MAG"
};
/* CSV 表头（缩放后物理量 mg / mdps）。BIN 模式不写。*/
static const char *const kLogHeaders[FATFS_SD_NUM_FILES] = {
  "frame_id,datetime,acc_x_mg,acc_y_mg,acc_z_mg,gyr_x_mdps,gyr_y_mdps,gyr_z_mdps\r\n",
  "frame_id,tick_ms,temp_C\r\n",
  "frame_id,datetime,acc_x_mg,acc_y_mg,acc_z_mg\r\n",
  "frame_id,datetime,acc_x_mg,acc_y_mg,acc_z_mg\r\n",
  "frame_id,datetime,temp_C,humidity_pct\r\n",
  "frame_id,datetime,mag_x_mG,mag_y_mG,mag_z_mG\r\n"
};

/* MIC.WAV —— 单独的 FIL + 字节计数器（区别于 CSV 的 g_log_files[]）。
 * 头里的 chunk_size/data_size 先写占位，停止时 f_lseek 回填。 */
static FIL      g_wav_file;
static uint8_t  g_wav_open  = 0U;
static uint32_t g_wav_bytes = 0U;   /* 已写入的 PCM 字节数 */

/* 标准 WAV 头：44 字节、紧凑排布、字段按 canonical WAV 顺序。
 * chunk_size 在偏移 4、data_size 在偏移 40（FatFs_SD_WavFinalize 回填用）。 */
typedef struct __attribute__((packed)) {
  char     riff[4];        uint32_t chunk_size;   char     wave[4];
  char     fmt[4];         uint32_t fmt_size;     uint16_t audio_format; uint16_t num_channels;
  uint32_t sample_rate;    uint32_t byte_rate;    uint16_t block_align;  uint16_t bits_per_sample;
  char     data[4];        uint32_t data_size;
} WavHeader_t;

const char *FatFs_SD_ResultToString(FRESULT result)
{
  switch (result)
  {
    case FR_OK: return "FR_OK";
    case FR_DISK_ERR: return "FR_DISK_ERR";
    case FR_INT_ERR: return "FR_INT_ERR";
    case FR_NOT_READY: return "FR_NOT_READY";
    case FR_NO_FILE: return "FR_NO_FILE";
    case FR_NO_PATH: return "FR_NO_PATH";
    case FR_INVALID_NAME: return "FR_INVALID_NAME";
    case FR_DENIED: return "FR_DENIED";
    case FR_EXIST: return "FR_EXIST";
    case FR_INVALID_OBJECT: return "FR_INVALID_OBJECT";
    case FR_WRITE_PROTECTED: return "FR_WRITE_PROTECTED";
    case FR_INVALID_DRIVE: return "FR_INVALID_DRIVE";
    case FR_NOT_ENABLED: return "FR_NOT_ENABLED";
    case FR_NO_FILESYSTEM: return "FR_NO_FILESYSTEM";
    case FR_MKFS_ABORTED: return "FR_MKFS_ABORTED";
    case FR_TIMEOUT: return "FR_TIMEOUT";
    case FR_LOCKED: return "FR_LOCKED";
    case FR_NOT_ENOUGH_CORE: return "FR_NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER: return "FR_INVALID_PARAMETER";
    default: return "FR_UNKNOWN";
  }
}

static FRESULT FatFs_SD_WriteExact(FIL *file, const void *buffer, UINT length)
{
  FRESULT result;
  UINT written = 0U;

  result = f_write(file, buffer, length, &written);
  if ((result != FR_OK) || (written != length))
  {
    return (result == FR_OK) ? FR_INT_ERR : result;
  }

  return FR_OK;
}

static FRESULT FatFs_SD_FindNextSessionDir(char *dir, size_t dir_size)
{
  Pcf85063_Time_t t;
  FILINFO info;

  /* 用 RTC 实时时间命名会话目录。AppTime 在启动时以 RTC 为锚同步(DWT 计时,不受
   * RTOS tick 慢 19% 影响),此处只读一次、不碰 I2C2(避免与 ES8311 抢总线)。
   * 若 RTC 不可达,AppTime 锚为 0 → 时间落在 1970,目录日期不准但下面的 _n 去重
   * 仍保证多会话不撞名(降级模式)。年份 = 2000 + BCD 年。 */
  uint32_t epoch_s = (uint32_t)(AppTime_GetEpochUs() / 1000000ULL);
  Pcf85063_FromEpochSeconds(epoch_s, &t);
  unsigned int year = 2000U + (unsigned int)t.year;

  for (unsigned int suffix = 0U; suffix <= 99U; suffix++)
  {
    int len;
    if (suffix == 0U)
    {
      len = snprintf(dir, dir_size, "0:/%s_%04u-%02u-%02u-%02u-%02u",
                     FATFS_SD_SESSION_PREFIX, year,
                     (unsigned int)t.month, (unsigned int)t.day,
                     (unsigned int)t.hour,  (unsigned int)t.minute);
    }
    else
    {
      len = snprintf(dir, dir_size, "0:/%s_%04u-%02u-%02u-%02u-%02u_%u",
                     FATFS_SD_SESSION_PREFIX, year,
                     (unsigned int)t.month, (unsigned int)t.day,
                     (unsigned int)t.hour,  (unsigned int)t.minute, suffix);
    }
    if ((len < 0) || ((size_t)len >= dir_size))
    {
      return FR_INVALID_NAME;
    }

    FRESULT r = f_stat(dir, &info);
    if (r == FR_NO_FILE)
    {
      return FR_OK;  /* 目录不存在，可用 */
    }
    if (r != FR_OK)
    {
      return r;
    }
    /* 目录已存在(同一分钟内重复开会话)，追加 _n 再试 */
  }

  return FR_DENIED;
}

static FRESULT FatFs_SD_OpenCsvFile(FIL *file, const char *dir, const char *fname,
                                     const char *header, uint8_t output_format)
{
  char path[FATFS_SD_FILE_PATH_MAX];
  char fname_with_ext[32];
  FRESULT r;

  /* 根据 output_format 选择扩展名: .CSV 或 .BIN */
  const char *base = fname;
  const char *dot = strrchr(fname, '.');
  if (dot != NULL)
  {
    size_t base_len = (size_t)(dot - fname);
    const char *ext = (output_format == ACQ_OUTPUT_BIN) ? ".BIN" : ".CSV";
    snprintf(fname_with_ext, sizeof(fname_with_ext), "%.*s%s", (int)base_len, fname, ext);
  }
  else
  {
    snprintf(fname_with_ext, sizeof(fname_with_ext), "%s", fname);
  }

  (void)snprintf(path, sizeof(path), "%s/%s", dir, fname_with_ext);

  r = f_open(file, path, FA_CREATE_ALWAYS | FA_WRITE);
  if (r != FR_OK)
  {
    printf("[FatFs] 打开 %s 失败: %s (%d)\r\n", path, FatFs_SD_ResultToString(r), (int)r);
    return r;
  }

  /* CSV 模式写表头,BIN 模式跳过 */
  if (output_format == ACQ_OUTPUT_CSV && header != NULL)
  {
    r = FatFs_SD_WriteExact(file, header, (UINT)strlen(header));
    if (r != FR_OK)
    {
      (void)f_close(file);
      return r;
    }
  }

  return f_sync(file);
}

/* 打开数据文件 idx 的当前段(文件名由 g_file_seg[idx] 决定)，并清零该段字节计数。
 * 段1用原名 kLogFnames[idx]；段>1 用 kLogTags[idx]+4位段号。扩展名/表头由
 * FatFs_SD_OpenCsvFile 按 g_log_format 处理(CSV写表头、BIN跳过)。*/
static FRESULT FatFs_SD_OpenLogSegment(uint8_t idx)
{
  char base[16];

  if (g_file_seg[idx] <= 1U)
  {
    (void)snprintf(base, sizeof(base), "%s", kLogFnames[idx]);
  }
  else
  {
    /* 续段：短标签 + 4位段号 + 占位 .CSV(扩展名由 OpenCsvFile 按格式重定) */
    (void)snprintf(base, sizeof(base), "%s%04u.CSV",
                   kLogTags[idx], (unsigned int)g_file_seg[idx]);
  }

  FRESULT r = FatFs_SD_OpenCsvFile(&g_log_files[idx], g_session_dir, base,
                                   kLogHeaders[idx], g_log_format);
  if (r == FR_OK)
  {
    g_file_bytes[idx] = 0U;
  }
  return r;
}

FRESULT FatFs_SD_Mount(void)
{
  FRESULT result;

  if (g_sd_mounted != 0U)
  {
    return FR_OK;
  }

  if (SDMMC1_IsCardDetected() == 0U)
  {
    return FR_NOT_READY;
  }

  if ((disk_initialize(SDDISKIO_DRIVE_NUM) & STA_NOINIT) != 0U)
  {
    return FR_NOT_READY;
  }

  /* Matches committed order: memset → unmount → mount.
   * The old re-ordering (unmount → memset → mount) caused FR_DISK_ERR
   * because f_mount(NULL) on a zeroed FATFS struct was a no-op, leaving
   * stale volume metadata that corrupted the subsequent f_mount. */
  memset(&g_sd_fatfs, 0, sizeof(g_sd_fatfs));
  result = f_mount(NULL, "0:", 1U);
  result = f_mount(&g_sd_fatfs, "0:", 1U);
  if (result == FR_OK)
  {
    g_sd_mounted = 1U;
  }

  return result;
}

void FatFs_SD_Unmount(void)
{
  if (g_wav_open != 0U)
  {
    (void)FatFs_SD_WavFinalize();
  }

  if (g_logger_active != 0U)
  {
    for (uint32_t i = 0U; i < FATFS_SD_NUM_FILES; i++)
    {
      (void)f_close(&g_log_files[i]);
    }
    g_logger_active = 0U;
    g_session_dir[0] = '\0';
  }

  if (g_sd_mounted != 0U)
  {
    (void)f_mount(NULL, "0:", 1U);
    g_sd_mounted = 0U;
  }
}

const char *FatFs_SD_GetSessionDir(void)
{
  return g_session_dir;
}

FRESULT FatFs_SD_LoggerStart(void)
{
  FRESULT result;

  if (g_logger_active != 0U)
  {
    return FR_OK;
  }

  if ((disk_initialize(SDDISKIO_DRIVE_NUM) & STA_NOINIT) != 0U)
  {
    return FR_NOT_READY;
  }

  result = FatFs_SD_Mount();
  if (result != FR_OK)
  {
    return result;
  }

  /* 查找下一个可用会话目录 */
  result = FatFs_SD_FindNextSessionDir(g_session_dir, sizeof(g_session_dir));
  if (result != FR_OK)
  {
    printf("[FatFs] 查找会话目录失败: %s (%d)\r\n", FatFs_SD_ResultToString(result), (int)result);
    FatFs_SD_Unmount();
    return result;
  }

  /* 创建会话目录 */
  result = f_mkdir(g_session_dir);
  if (result != FR_OK)
  {
    printf("[FatFs] 创建目录 %s 失败: %s (%d)\r\n",
           g_session_dir, FatFs_SD_ResultToString(result), (int)result);
    FatFs_SD_Unmount();
    return result;
  }

  /* 获取当前配置的输出格式与分段大小 */
  AcqConfig_t cfg;
  AcqConfig_GetCopy(&cfg);
  g_log_format = cfg.output_format;

  /* 计算分段字节上限：0=不分段；否则夹紧到 [16,4000]MB(<FAT32 单文件 4GB) */
  if (cfg.seg_size_mb == 0U)
  {
    g_seg_max_bytes = 0U;
  }
  else
  {
    uint32_t mb = cfg.seg_size_mb;
    if (mb < 16U)   { mb = 16U; }
    if (mb > 4000U) { mb = 4000U; }
    g_seg_max_bytes = mb * 1024U * 1024U;   /* 4000*1MiB=4194304000 < UINT32_MAX */
  }

  /* 打开 6 个数据文件的首段(CSV 或 BIN)，并根据格式写入表头 */
  for (uint32_t i = 0U; i < FATFS_SD_NUM_FILES; i++)
  {
    g_file_seg[i]   = 1U;
    g_file_bytes[i] = 0U;
    result = FatFs_SD_OpenLogSegment((uint8_t)i);
    if (result != FR_OK)
    {
      /* 关闭已打开的文件 */
      for (uint32_t j = 0U; j < i; j++)
      {
        (void)f_close(&g_log_files[j]);
      }
      FatFs_SD_Unmount();
      return result;
    }
  }

  if (g_seg_max_bytes == 0U)
  {
    printf("[FatFs] 会话目录已创建: %s (不分段)\r\n", g_session_dir);
  }
  else
  {
    printf("[FatFs] 会话目录已创建: %s (每文件分段 %luMB)\r\n",
           g_session_dir, (unsigned long)(g_seg_max_bytes / (1024U * 1024U)));
  }
  g_logger_active = 1U;
  g_logger_rows_written = 0U;
  return FR_OK;
}

FRESULT FatFs_SD_LoggerAppendFrame(const AppSensorFrame_t *frame)
{
  FRESULT result = FR_OK;
  char line[128];
  int len;

  if (frame == NULL)
  {
    return FR_INVALID_PARAMETER;
  }

  if (g_logger_active == 0U)
  {
    return FR_NOT_ENABLED;
  }

  if (SDMMC1_IsCardDetected() == 0U)
  {
    return FR_NOT_READY;
  }

  /* LSM6DSOX acc/gyr now go through the LSM_IMU ring buffer (raw int16),
   * QMA acc through the QMA_ACC ring buffer (raw int14). The per-frame path
   * here only handles the slow LSM temperature and the H3LIS samples. */

  /* LSM6DSOX temp — 文件索引 LSM_TMP */
  if (frame->lsm6dsox.valid != 0U)
  {
    len = snprintf(line, sizeof(line), "%lu,%lu,%.1f\r\n",
                   (unsigned long)frame->frame_id,
                   (unsigned long)frame->tick_ms,
                   frame->lsm6dsox.data.temp_C);
    if ((len > 0) && ((size_t)len < sizeof(line)))
      result = FatFs_SD_WriteExact(&g_log_files[FATFS_SD_FILE_LSM_TMP], line, (UINT)len);
    if (result != FR_OK) return result;
  }

  /* H3LIS100DL acc and QMA6100P acc now write via their own ring buffers,
   * see RingBuf_PeekContiguous + LoggerWriteFileIndex in StartLoggerTask. */

  g_logger_rows_written++;
  return FR_OK;
}

/* MIC.WAV：在会话目录创建文件并写入占位头（chunk_size=36、data_size=0）。
 * sample_rate_hz ∈ {8000,16000,48000}，bits 固定 16。停止时由
 * FatFs_SD_WavFinalize 回填 chunk_size/data_size。 */
FRESULT FatFs_SD_WavCreate(uint32_t sample_rate_hz, uint16_t bits)
{
  if (g_logger_active == 0U) return FR_NOT_ENABLED;

  char path[FATFS_SD_FILE_PATH_MAX];
  (void)snprintf(path, sizeof(path), "%s/%s", g_session_dir, FATFS_SD_FNAME_MIC_WAV);

  FRESULT r = f_open(&g_wav_file, path, FA_CREATE_ALWAYS | FA_WRITE);
  if (r != FR_OK)
  {
    printf("[FatFs] 打开 %s 失败: %s (%d)\r\n", path, FatFs_SD_ResultToString(r), (int)r);
    return r;
  }

  WavHeader_t h;
  memcpy(h.riff, "RIFF", 4); memcpy(h.wave, "WAVE", 4);
  memcpy(h.fmt,  "fmt ", 4); memcpy(h.data, "data", 4);
  h.chunk_size      = 36U;          /* 占位：36 + data_size，停止时回填 */
  h.fmt_size        = 16U;
  h.audio_format    = 1U;           /* PCM */
  h.num_channels    = 1U;           /* 单声道 */
  h.sample_rate     = sample_rate_hz;
  h.bits_per_sample = bits;
  h.block_align     = (uint16_t)(1U * bits / 8U);
  h.byte_rate       = sample_rate_hz * h.block_align;
  h.data_size       = 0U;           /* 占位 */

  r = FatFs_SD_WriteExact(&g_wav_file, &h, sizeof(h));
  if (r != FR_OK)
  {
    (void)f_close(&g_wav_file);
    return r;
  }

  g_wav_open  = 1U;
  g_wav_bytes = 0U;
  return FR_OK;
}

/* Generic single-file log write — used by the ring buffer flush path
 * (LSM_IMU and QMA_ACC both come through here from the logger task).
 * idx==FATFS_SD_FILE_MIC_WAV(6) 路由到独立的 g_wav_file 并累加字节数。 */
FRESULT FatFs_SD_LoggerWriteFileIndex(uint8_t idx, const uint8_t *data, uint32_t len)
{
  if (data == NULL || len == 0U) return FR_INVALID_PARAMETER;

  if (idx == FATFS_SD_FILE_MIC_WAV)
  {
    if (g_wav_open == 0U) return FR_NOT_ENABLED;
    if (SDMMC1_IsCardDetected() == 0U) return FR_NOT_READY;
    FRESULT r = FatFs_SD_WriteExact(&g_wav_file, data, (UINT)len);
    if (r == FR_OK) g_wav_bytes += len;
    return r;
  }

  if (idx >= FATFS_SD_NUM_FILES) return FR_INVALID_PARAMETER;
  if (g_logger_active == 0U) return FR_NOT_ENABLED;
  if (SDMMC1_IsCardDetected() == 0U) return FR_NOT_READY;

  /* 分段：本段已写 g_file_bytes[idx]，若再写 len 会超过上限就先滚动到下一段。
   * 在写入前滚动可保证“整块写入”不跨文件，每段大小始终 ≤ g_seg_max_bytes
   * (误差 < 一次 flush 块 16KB)。g_file_bytes!=0 的判定避免首块即滚动。*/
  if ((g_seg_max_bytes != 0U) && (g_file_bytes[idx] != 0U) &&
      ((g_file_bytes[idx] + len) > g_seg_max_bytes))
  {
    (void)f_sync(&g_log_files[idx]);
    (void)f_close(&g_log_files[idx]);
    g_file_seg[idx]++;
    FRESULT ro = FatFs_SD_OpenLogSegment(idx);
    if (ro != FR_OK)
    {
      printf("[FatFs] 文件%u 开新段(seg=%u)失败: %s (%d)\r\n",
             (unsigned int)idx, (unsigned int)g_file_seg[idx],
             FatFs_SD_ResultToString(ro), (int)ro);
      return ro;
    }
  }

  FRESULT r = FatFs_SD_WriteExact(&g_log_files[idx], data, (UINT)len);
  if (r == FR_OK)
  {
    g_file_bytes[idx] += len;
  }
  return r;
}

/* MIC.WAV 收尾：回填 chunk_size(@偏移4) 与 data_size(@偏移40)，同步并关闭。 */
FRESULT FatFs_SD_WavFinalize(void)
{
  if (g_wav_open == 0U) return FR_OK;

  uint32_t chunk_size = 36U + g_wav_bytes;
  FRESULT r1 = f_lseek(&g_wav_file, 4U);
  if (r1 == FR_OK) r1 = FatFs_SD_WriteExact(&g_wav_file, &chunk_size, 4U);
  FRESULT r2 = f_lseek(&g_wav_file, 40U);
  if (r2 == FR_OK) r2 = FatFs_SD_WriteExact(&g_wav_file, &g_wav_bytes, 4U);

  (void)f_sync(&g_wav_file);
  (void)f_close(&g_wav_file);
  g_wav_open = 0U;
  return (r1 == FR_OK) ? r2 : r1;
}

FRESULT FatFs_SD_LoggerSync(void)
{
  FRESULT result = FR_OK;

  if (g_logger_active == 0U)
  {
    return FR_NOT_ENABLED;
  }

  for (uint32_t i = 0U; i < FATFS_SD_NUM_FILES; i++)
  {
    FRESULT r = f_sync(&g_log_files[i]);
    if (r != FR_OK)
    {
      result = r;
    }
  }

  if (g_wav_open != 0U) { (void)f_sync(&g_wav_file); }

  if (result != FR_OK)
  {
    printf("[FatFs] sync rows=%lu result=%s (%d)\r\n",
           (unsigned long)g_logger_rows_written,
           FatFs_SD_ResultToString(result),
           (int)result);
  }
  return result;
}

void FatFs_SD_LoggerStop(void)
{
  if (g_wav_open != 0U) { (void)FatFs_SD_WavFinalize(); }

  if (g_logger_active != 0U)
  {
    for (uint32_t i = 0U; i < FATFS_SD_NUM_FILES; i++)
    {
      (void)f_sync(&g_log_files[i]);
      (void)f_close(&g_log_files[i]);
    }

    printf("[FatFs] logger stop rows=%lu dir=%s\r\n",
           (unsigned long)g_logger_rows_written,
           g_session_dir);

    g_logger_active = 0U;
    g_logger_rows_written = 0U;
    g_session_dir[0] = '\0';
  }

  if (g_sd_mounted != 0U)
  {
    (void)f_mount(NULL, "0:", 1U);
    g_sd_mounted = 0U;
  }
}

void FatFs_SD_RunPhaseBSmokeTest(void)
{
  FRESULT result;
  FIL file;
  UINT written = 0U;
  UINT read_len = 0U;
  BYTE sector_buffer[SDDISKIO_SECTOR_SIZE] = {0};
  char readback[96] = {0};
  static const char kTestText[] = "Phase B SD write test\r\n";

  if (SDMMC1_IsCardDetected() == 0U)
  {
    printf("[FatFs] 未检测到SD卡，跳过文件系统测试\r\n");
    return;
  }

  if (disk_initialize(SDDISKIO_DRIVE_NUM) & STA_NOINIT)
  {
    printf("[FatFs] 底层磁盘初始化失败\r\n");
    return;
  }

  result = FatFs_SD_Mount();
  if (result != FR_OK)
  {
    printf("[FatFs] 挂载失败: %s (%d)\r\n", FatFs_SD_ResultToString(result), (int)result);
    return;
  }
  printf("[FatFs] 挂载成功\r\n");

  result = f_open(&file, "0:/PHASEB.TXT", FA_CREATE_ALWAYS | FA_WRITE);
  if (result != FR_OK)
  {
    printf("[FatFs] 打开测试文件失败: %s (%d)\r\n", FatFs_SD_ResultToString(result), (int)result);
    FatFs_SD_Unmount();
    return;
  }

  result = f_write(&file, kTestText, (UINT)(sizeof(kTestText) - 1U), &written);
  if ((result != FR_OK) || (written != (UINT)(sizeof(kTestText) - 1U)))
  {
    printf("[FatFs] 写入测试文件失败: %s (%d), written=%u\r\n",
           FatFs_SD_ResultToString(result), (int)result, (unsigned int)written);
    (void)f_close(&file);
    FatFs_SD_Unmount();
    return;
  }

  result = f_sync(&file);
  if (result != FR_OK)
  {
    printf("[FatFs] 同步测试文件失败: %s (%d)\r\n", FatFs_SD_ResultToString(result), (int)result);
    (void)f_close(&file);
    FatFs_SD_Unmount();
    return;
  }

  result = f_close(&file);
  if (result != FR_OK)
  {
    printf("[FatFs] 关闭测试文件失败: %s (%d)\r\n", FatFs_SD_ResultToString(result), (int)result);
    FatFs_SD_Unmount();
    return;
  }

  result = f_open(&file, "0:/PHASEB.TXT", FA_READ);
  if (result != FR_OK)
  {
    printf("[FatFs] 回读打开失败: %s (%d)\r\n", FatFs_SD_ResultToString(result), (int)result);
    FatFs_SD_Unmount();
    return;
  }

  result = f_read(&file, readback, (UINT)(sizeof(readback) - 1U), &read_len);
  if (result != FR_OK)
  {
    printf("[FatFs] 回读失败: %s (%d)\r\n", FatFs_SD_ResultToString(result), (int)result);
    (void)f_close(&file);
    FatFs_SD_Unmount();
    return;
  }
  readback[read_len] = '\0';
  (void)f_close(&file);

  if (strncmp(readback, kTestText, sizeof(kTestText) - 1U) != 0)
  {
    printf("[FatFs] 回读内容不匹配: %s\r\n", readback);
    FatFs_SD_Unmount();
    return;
  }

  if (SD_disk_read(SDDISKIO_DRIVE_NUM, sector_buffer, 0U, 1U) != RES_OK)
  {
    printf("[FatFs] 底层扇区读取验证失败\r\n");
    FatFs_SD_Unmount();
    return;
  }

  printf("[FatFs] 阶段B测试成功: 挂载/写入/关闭/回读 已通过\r\n");
  FatFs_SD_Unmount();
}
