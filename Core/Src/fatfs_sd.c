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
#define FATFS_SD_FILE_PATH_MAX    48U   /* dir(27) + "/ACC_HIGH001.CSV"(16) = 43 + NUL */

/* 7 个数据文件的基名(已开 LFN,用完整长名,不受 8.3 限制)。命名按功能通用化
 * (不暴露芯片型号):后缀 _LOW/_MID/_HIGH 为加速度量程档,前缀 ACC/GYR/TMP 为信号类型。
 * 低量程为六轴,加速度/角速度拆两文件:ACC_LOW(索引0,带 datetime) + GYR_LOW(索引6,仅
 * frame_id,靠同一采样对的 frame_id 与 ACC 对齐);TMP_LOW 慢温度通道;ACC_HIGH/ACC_MID;
 * ENV(温湿度)/MAG(磁力)。
 * 实际文件名 = 基名 + 3位段号 + 扩展(如 ACC_HIGH001.CSV);首段即 001,分段续写 002/003…
 * 扩展名(.CSV/.BIN)由 FatFs_SD_OpenCsvFile 按输出格式决定。 */
#define FATFS_SD_FNAME_ACC_LOW   "ACC_LOW"        /* 索引 0：低量程加速度 */
#define FATFS_SD_FNAME_TMP_LOW   "TMP_LOW"        /* 索引 1：低量程模块温度 */
#define FATFS_SD_FNAME_ACC_HIGH  "ACC_HIGH"       /* 索引 2：高量程加速度 */
#define FATFS_SD_FNAME_ACC_MID   "ACC_MID"        /* 索引 3：中量程加速度 */
#define FATFS_SD_FNAME_ENV       "ENV"            /* 索引 4：温湿度 */
#define FATFS_SD_FNAME_MAG       "MAG"            /* 索引 5：磁力 */
#define FATFS_SD_FNAME_GYR_LOW   "GYR_LOW"        /* 索引 6：低量程角速度 */
#define FATFS_SD_FNAME_MIC_WAV   "MIC.WAV"        /* 麦克风,不分段,保留扩展名 */

#define FATFS_SD_FILE_LSM_ACC     0U
#define FATFS_SD_FILE_LSM_TMP     1U
#define FATFS_SD_FILE_H3_ACC      2U
#define FATFS_SD_FILE_QMA_ACC     3U

#define FATFS_SD_NUM_FILES        7U  /* 0..6 CSV(含 LSM_GYR); MIC_WAV 独立索引7 */

static FATFS g_sd_fatfs;
static FIL g_log_files[FATFS_SD_NUM_FILES];
static char g_session_dir[FATFS_SD_DIR_PATH_MAX];
static uint8_t g_sd_mounted = 0U;
static uint8_t g_logger_active = 0U;
static uint32_t g_logger_rows_written = 0U;

/* ---- 文件分段(segmentation) ----
 * 6 个传感器数据文件(索引 0..5)各自独立按字节数滚动；MIC.WAV(索引6)不分段。
 * 所有段统一命名为「基名+3位段号」：首段即 ACC_HIGH001.CSV，写满续段 002/003…，
 * 各文件独立计数。CSV 续段重写表头，BIN 续段无头(同首段)。*/
static uint32_t g_seg_max_bytes = 0U;                    /* 0=不分段；否则每段字节上限 */
static uint8_t  g_log_format    = ACQ_OUTPUT_CSV;        /* LoggerStart 时缓存的输出格式 */
static uint32_t g_file_bytes[FATFS_SD_NUM_FILES] = {0};  /* 当前段已写字节数 */
static uint16_t g_file_seg[FATFS_SD_NUM_FILES]   = {0};  /* 当前段号(1 基) */

/* 数据文件基名(不含段号/扩展名);实际文件名 = 基名 + 3位段号 + .CSV/.BIN */
static const char *const kLogBasenames[FATFS_SD_NUM_FILES] = {
  FATFS_SD_FNAME_ACC_LOW, FATFS_SD_FNAME_TMP_LOW,
  FATFS_SD_FNAME_ACC_HIGH, FATFS_SD_FNAME_ACC_MID,
  FATFS_SD_FNAME_ENV,     FATFS_SD_FNAME_MAG,
  FATFS_SD_FNAME_GYR_LOW
};
/* CSV 表头（缩放后物理量 mg / mdps）。BIN 模式不写。
 * ACC_LOW 带 datetime;GYR_LOW 仅 frame_id(与 ACC 同帧号对齐,不重复 datetime)。*/
static const char *const kLogHeaders[FATFS_SD_NUM_FILES] = {
  "frame_id,datetime,acc_x_mg,acc_y_mg,acc_z_mg\r\n",
  "frame_id,tick_ms,temp_C\r\n",
  "frame_id,datetime,acc_x_mg,acc_y_mg,acc_z_mg\r\n",
  "frame_id,datetime,acc_x_mg,acc_y_mg,acc_z_mg\r\n",
  "frame_id,datetime,temp_C,humidity_pct\r\n",
  "frame_id,datetime,mag_x_mG,mag_y_mG,mag_z_mG\r\n",
  "frame_id,gyr_x_mdps,gyr_y_mdps,gyr_z_mdps\r\n"
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

  /* 【已回退 f_expand 预分配】曾试图开段时 f_expand 整段(512MB)消 FAT 停顿,但实测:
   * (1) 对满配掉帧零帮助(根因是 SD 卡 GC 周期性内部忙,非 FAT 链更新);
   * (2) 开录一次性预分配 7×512MB=3.5GB FAT 簇链,非优雅结束时半提交 FAT → 整盘损坏、
   *     零可恢复数据(实测 CTBX_..14-53 新卡整盘 corrupt)。无收益+放大损坏 → 移除。 */

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

/* 打开数据文件 idx 的当前段(段号由 g_file_seg[idx] 决定)，并清零该段字节计数。
 * 统一命名：基名 + 3位段号 + 占位 .CSV(如 ACC_HIGH001.CSV);首段即 001。扩展名/表头
 * 由 FatFs_SD_OpenCsvFile 按 g_log_format 处理(CSV写表头、BIN跳过重定扩展名)。*/
static FRESULT FatFs_SD_OpenLogSegment(uint8_t idx)
{
  char base[20];

  (void)snprintf(base, sizeof(base), "%s%03u.CSV",
                 kLogBasenames[idx], (unsigned int)g_file_seg[idx]);

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

/* logger 是否仍持有打开的文件。FatFs_SD_LoggerStop 在 finalize WAV(回填头)+ 对全部
 * 文件 f_sync/f_close 之后才清 0 —— 关机/进 MSC 前应轮询此值为 0,确保文件已完整落盘
 * 再断电/复位,避免写到一半掉电损坏(注意 AppAcqIsRunning 在停采瞬间即 0,但文件是
 * logger 任务随后异步关的,不能用它判断"已落盘")。 */
uint8_t FatFs_SD_IsLoggerActive(void)
{
  return g_logger_active;
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

  /* 低量程加速度/角速度经各自 ring buffer(raw int16)写出,中量程加速度经其 ring
   * (raw int14)。此逐帧路径仅处理慢速的低量程模块温度与高量程加速度样本。 */

  /* 低量程模块温度 — 文件索引 TMP_LOW */
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

  /* 高量程与中量程加速度经各自的 ring buffer 写出,
   * 见 StartLoggerTask 里的 RingBuf_CopyToBounce + LoggerWriteFileIndex。 */

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
 * (低量程与中量程加速度都经此路径从 logger 任务写出)。
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

/* MIC.WAV 录制途中周期性 checkpoint(B3,防硬掉电音频不可播的保险)。
 * 用当前 g_wav_bytes 回填 chunk_size(@4)/data_size(@40) 并 f_sync,使文件在任意时刻
 * 都是一个头部有效、可播放的合法 WAV(最多丢 checkpoint 之后未回填的尾部)。★ 回填后
 * 文件指针停在偏移 44,必须 seek 回数据尾(44+g_wav_bytes)再返回,否则后续 PCM 写入会
 * 覆盖已录音频。不 close——录制继续。独立于低电量电压检测:即便漏检测硬掉电,音频也只
 * 丢最后一个 checkpoint 周期(~30s)且能正常播放。 */
FRESULT FatFs_SD_WavCheckpoint(void)
{
  if (g_wav_open == 0U) return FR_OK;
  if (SDMMC1_IsCardDetected() == 0U) return FR_NOT_READY;

  uint32_t chunk_size = 36U + g_wav_bytes;
  FRESULT r = f_lseek(&g_wav_file, 4U);
  if (r == FR_OK) r = FatFs_SD_WriteExact(&g_wav_file, &chunk_size, 4U);
  if (r == FR_OK) r = f_lseek(&g_wav_file, 40U);
  if (r == FR_OK) r = FatFs_SD_WriteExact(&g_wav_file, &g_wav_bytes, 4U);
  if (r == FR_OK) (void)f_sync(&g_wav_file);

  /* ★ 关键:回填后指针在 44,必须 seek 回数据尾,否则下一次写 PCM 会覆盖音频 */
  FRESULT rs = f_lseek(&g_wav_file, (FSIZE_t)(44U + g_wav_bytes));
  if (r == FR_OK) r = rs;
  return r;
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

    /* ★2026-07-12 顺序修正:先 f_mount(NULL) 卸载(flush 残留 FS 状态)再清
     * g_logger_active。原顺序在 f_mount 之前就把 active 清 0,导致优雅关机的
     * Uc_WaitLoggerClosed(等 IsLoggerActive()==0)可能在 f_mount 还没跑完时就
     * 放行断电 → FS 未完全卸载。现在 IsLoggerActive()==0 严格代表"文件已 close
     * 且卷已 unmount",断电前一定 flush 完成。 */
    printf("[FatFs] logger stop rows=%lu dir=%s\r\n",
           (unsigned long)g_logger_rows_written,
           g_session_dir);

    g_logger_rows_written = 0U;
    g_session_dir[0] = '\0';

    if (g_sd_mounted != 0U)
    {
      (void)f_mount(NULL, "0:", 1U);
      g_sd_mounted = 0U;
    }

    g_logger_active = 0U;   /* 最后置 0:此后 IsLoggerActive()==0 = 全部收尾+卸载完成 */
    return;
  }

  if (g_sd_mounted != 0U)
  {
    (void)f_mount(NULL, "0:", 1U);
    g_sd_mounted = 0U;
  }
}

/* ============================================================================
 * SD 写吞吐基准(sdbench)—— 隔离测卡的裸顺序写能力,与采集路径完全无关。
 * 目的:钉死"~0.65MB/s 是卡物理墙 还是 我们写模式的浪费"。SDMMC 配置为
 * 24MHz×4bit=12MB/s 理论带宽,满配实测只推 ~0.65MB/s(5%)。本基准不跑任何
 * 传感器,单文件连续写固定字节流,分别测不同块大小 × 不同 f_sync 策略的 MB/s。
 *
 * 计时用 HAL_GetTick()(TIM6 硬件 1ms,精确;RTOS tick 慢 ~19% 不能用于计时)。
 * 由 APP_SD_BENCH 编译开关在 logger 任务启动时调用一次,打印后死循环停住
 *(不进采集)。开关默认 0 → 零影响、完全可逆。
 *
 * 输出每行:块大小 | sync策略 | 写字节 | 耗时ms | MB/s。三种可能结果:
 *  (1) 各块大小都≈0.65MB/s        → 真卡墙,环/块调优无用,只能砍mic或换卡。
 *  (2) MB/s 随块增大明显上升       → 我们写太碎,加大写块能真降掉帧。
 *  (3) 小块也能 >2MB/s + sync明显拖 → 瓶颈是 f_sync/交织,解耦写线程能救。
 * ==========================================================================*/
#ifndef APP_SD_BENCH
#define APP_SD_BENCH 0U   /* 基准已测完(2026-07-10):见 app_freertos.c 同名宏。已恢复正常采集。 */
#endif

#if (APP_SD_BENCH != 0U)

#define SDBENCH_TOTAL_BYTES   (8U * 1024U * 1024U)   /* 每组固定写 8MB */

/* 跑一组:块大小 block、每 sync_every 字节 f_sync 一次(0=整程不 sync,仅收尾 close)。
 * src/src_cap = 调用方传入的源缓冲(复用 logger 的 s_sd_bounce,不再单独占 RAM)。
 * 返回落盘 MB/s×100(定点,避免浮点格式化);<0 表示出错。 */
static int32_t SdBench_RunOne(uint32_t block, uint32_t sync_every,
                              const uint8_t *src, uint32_t src_cap)
{
  if (block > src_cap) block = src_cap;   /* 块不能超过源缓冲容量 */
  FIL f;
  FRESULT r;
  UINT wr = 0U;
  uint32_t written = 0U;
  uint32_t since_sync = 0U;

  r = f_open(&f, "0:/SDBENCH.BIN", FA_CREATE_ALWAYS | FA_WRITE);
  if (r != FR_OK) { printf("[SdBench] open fail %s\r\n", FatFs_SD_ResultToString(r)); return -1; }

  uint32_t t0 = HAL_GetTick();
  while (written < SDBENCH_TOTAL_BYTES)
  {
    uint32_t n = block;
    if ((written + n) > SDBENCH_TOTAL_BYTES) n = SDBENCH_TOTAL_BYTES - written;

    __DMB();   /* 与真实写路径一致:启动 IDMA 前确保源缓冲已落 SRAM */
    r = f_write(&f, src, (UINT)n, &wr);
    if ((r != FR_OK) || (wr != n))
    {
      printf("[SdBench] write fail %s wr=%u\r\n", FatFs_SD_ResultToString(r), (unsigned)wr);
      (void)f_close(&f);
      return -2;
    }
    written   += n;
    since_sync += n;

    if ((sync_every != 0U) && (since_sync >= sync_every))
    {
      (void)f_sync(&f);
      since_sync = 0U;
    }
  }
  (void)f_close(&f);   /* close 收尾:提交 FAT+目录(这部分不计入吞吐,与 DATALOG1 一致) */
  uint32_t dt = HAL_GetTick() - t0;
  if (dt == 0U) dt = 1U;

  /* MB/s×100 = 字节/(1MB) / (dt/1000) × 100 = 字节×100000 /(dt×1048576)。
   * 用 uint64_t 防溢出(8MB×100000 = 8e11)。 */
  uint32_t mbps100 = (uint32_t)(((uint64_t)written * 100000ULL) / ((uint64_t)dt * 1048576ULL));
  return (int32_t)mbps100;
}

/* 多文件交织跑测:nfiles 个文件同时打开,round-robin 每文件写 block 字节,直到
 * 累计写满 SDBENCH_TOTAL_BYTES。模拟真实工况的"7 文件交织",隔离交织对吞吐的影响。
 * 固定不 sync(仅收尾 close),块大小固定(调用方给已知最优 64KB)。
 * 返回聚合落盘 MB/s×100;<0 出错。 */
static int32_t SdBench_RunMulti(uint32_t nfiles, uint32_t block,
                                const uint8_t *src, uint32_t src_cap)
{
  static FIL fs[8];   /* static:每 FIL 含 512B 扇区缓冲,8 个放栈上(4KB)会溢出 logger 栈 */
  uint8_t opened = 0U;
  int32_t rc = -1;
  uint32_t written = 0U;
  uint32_t t0;
  uint32_t dt;

  if (nfiles > 8U) nfiles = 8U;
  if (block > src_cap) block = src_cap;

  for (uint32_t i = 0U; i < nfiles; i++)
  {
    char path[24];
    (void)snprintf(path, sizeof(path), "0:/SDBM_%u.BIN", (unsigned)i);
    if (f_open(&fs[i], path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    { printf("[SdBench] multi open%u fail\r\n", (unsigned)i); goto multi_close; }
    opened++;
  }

  t0 = HAL_GetTick();
  while (written < SDBENCH_TOTAL_BYTES)
  {
    for (uint32_t i = 0U; (i < nfiles) && (written < SDBENCH_TOTAL_BYTES); i++)
    {
      UINT wr = 0U;
      uint32_t n = block;
      if ((written + n) > SDBENCH_TOTAL_BYTES) n = SDBENCH_TOTAL_BYTES - written;
      __DMB();
      if ((f_write(&fs[i], src, (UINT)n, &wr) != FR_OK) || (wr != n))
      { printf("[SdBench] multi write fail f%u\r\n", (unsigned)i); goto multi_close; }
      written += n;
    }
  }
  dt = HAL_GetTick() - t0;
  if (dt == 0U) dt = 1U;
  rc = (int32_t)((uint32_t)(((uint64_t)written * 100000ULL) / ((uint64_t)dt * 1048576ULL)));

multi_close:
  for (uint32_t i = 0U; i < opened; i++) (void)f_close(&fs[i]);
  for (uint32_t i = 0U; i < opened; i++)
  {
    char path[24];
    (void)snprintf(path, sizeof(path), "0:/SDBM_%u.BIN", (unsigned)i);
    (void)f_unlink(path);
  }
  return rc;
}

/* 打印一行结果(MB/s 用 定点×100 拆成整数.两位小数,避免 microLIB %f 依赖)。 */
static void SdBench_PrintRow(const char *tag, uint32_t block, int32_t mbps100, uint32_t dt_ms)
{
  if (mbps100 < 0) { printf("[SdBench] %-10s blk=%5uB  ERROR\r\n", tag, (unsigned)block); return; }
  printf("[SdBench] %-10s blk=%5uB  %ums  %u.%02u MB/s\r\n",
         tag, (unsigned)block, (unsigned)dt_ms,
         (unsigned)(mbps100 / 100), (unsigned)(mbps100 % 100));
}

void FatFs_SD_RunWriteBenchmark(uint8_t *scratch, uint32_t scratch_cap)
{
  static const uint32_t k_blocks[] = { 4096U, 16384U, 32768U, 65536U };
  const uint32_t k_nblk = sizeof(k_blocks) / sizeof(k_blocks[0]);
  uint8_t bench_ok = 1U;

  printf("\r\n[SdBench] ===== SD 写吞吐基准开始 (8MB/组, 24MHz x4bit=12MB/s 理论) =====\r\n");

  if ((scratch == NULL) || (scratch_cap == 0U))
  { printf("[SdBench] 无源缓冲,跳过\r\n"); bench_ok = 0U; }
  else if (SDMMC1_IsCardDetected() == 0U)
  { printf("[SdBench] 无卡,跳过\r\n"); bench_ok = 0U; }
  else if (disk_initialize(SDDISKIO_DRIVE_NUM) & STA_NOINIT)
  { printf("[SdBench] disk_initialize 失败\r\n"); bench_ok = 0U; }
  else if ((g_sd_mounted == 0U) && (FatFs_SD_Mount() != FR_OK))
  { printf("[SdBench] 挂载失败\r\n"); bench_ok = 0U; }

  if (bench_ok != 0U)
  {
  memset(scratch, 0x5A, scratch_cap);   /* 填非零可辨识图案(复用 logger bounce,基准独占) */

  /* 两个总表:两种 IDMA/轮询模式 × 四种块大小 × 两种 sync 策略。
   * 通过 SD_SetDmaMode 切模式(基准独占卡,无并发,切换安全)。 */
  for (uint32_t m = 0U; m < 2U; m++)
  {
    SD_SetDmaMode((uint8_t)m);   /* 0=轮询 1=IDMA+HWFC */
    printf("[SdBench] ---- 模式: %s ----\r\n", (m != 0U) ? "IDMA+HWFC" : "POLLING");

    for (uint32_t i = 0U; i < k_nblk; i++)
    {
      /* A) 整程不 sync(仅 close 收尾)—— DATALOG1 策略,测卡纯顺序写上限。 */
      uint32_t t0 = HAL_GetTick();
      int32_t a = SdBench_RunOne(k_blocks[i], 0U, scratch, scratch_cap);
      SdBench_PrintRow("nosync", k_blocks[i], a, HAL_GetTick() - t0);

      /* B) 每 1MB f_sync 一次 —— 我们当前策略,测 f_sync(FAT随机写) 拖了多少。 */
      t0 = HAL_GetTick();
      int32_t b = SdBench_RunOne(k_blocks[i], 1024U * 1024U, scratch, scratch_cap);
      SdBench_PrintRow("sync1MB", k_blocks[i], b, HAL_GetTick() - t0);
    }
  }

  (void)f_unlink("0:/SDBENCH.BIN");

  /* ★交织基准:固定 IDMA+64KB(单文件已测最优 6MB/s),把 8MB 分摊到 N 个文件
   * round-robin 写,看聚合吞吐随文件数塌到多少。真实工况开 7+1 文件,疑"多文件交织
   * → 卡写位置盘上跳变 → 无法保持 AU 连续写 → 6MB/s 打成 <0.7"。若 N=7 塌到 ~0.6
   * → 坐实交织是 9 倍浪费的元凶,解法=合并文件/每文件攒更大块再切。 */
  printf("[SdBench] ---- 交织(IDMA+64KB,8MB 分摊到 N 文件 round-robin) ----\r\n");
  SD_SetDmaMode(1U);
  static const uint32_t k_nfiles[] = { 1U, 2U, 4U, 7U };
  for (uint32_t i = 0U; i < (sizeof(k_nfiles)/sizeof(k_nfiles[0])); i++)
  {
    uint32_t t0 = HAL_GetTick();
    int32_t v = SdBench_RunMulti(k_nfiles[i], 65536U, scratch, scratch_cap);
    uint32_t dt = HAL_GetTick() - t0;
    if (v < 0) { printf("[SdBench] nfiles=%u  ERROR\r\n", (unsigned)k_nfiles[i]); }
    else printf("[SdBench] nfiles=%u  blk=65536B  %ums  %u.%02u MB/s\r\n",
                (unsigned)k_nfiles[i], (unsigned)dt,
                (unsigned)(v / 100), (unsigned)(v % 100));
  }

  printf("[SdBench] ===== 基准结束 =====\r\n");
  }  /* bench_ok */

  printf("[SdBench] 基准模式挂起(不进采集)。复位并关 APP_SD_BENCH 恢复正常。\r\n");
  for (;;) { HAL_Delay(1000); }
}

#endif /* APP_SD_BENCH */

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
