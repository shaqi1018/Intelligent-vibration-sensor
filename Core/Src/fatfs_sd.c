#include "fatfs_sd.h"

#include <stdio.h>
#include <string.h>

#include "diskio.h"
#include "sd_diskio.h"
#include "sdmmc.h"

/* 会话目录前缀，8.3 格式：CKBX + 4 位数字 = 8 字符 */
#define FATFS_SD_SESSION_PREFIX   "CKBX"
#define FATFS_SD_SESSION_MAX      9999U
#define FATFS_SD_DIR_PATH_MAX     24U   /* "0:/CKBOX0001" = 13 + NUL */
#define FATFS_SD_FILE_PATH_MAX    32U   /* "0:/CKBOX0001/LSM_ACC.CSV" = 25 + NUL */

/* 5 个 CSV 文件名（8.3 格式） */
#define FATFS_SD_FNAME_LSM_ACC   "LSM_ACC.CSV"
#define FATFS_SD_FNAME_LSM_GYR   "LSM_GYR.CSV"
#define FATFS_SD_FNAME_LSM_TMP   "LSM_TMP.CSV"
#define FATFS_SD_FNAME_H3_ACC    "H3_ACC.CSV"
#define FATFS_SD_FNAME_QMA_ACC   "QMA_ACC.CSV"

#define FATFS_SD_NUM_FILES        5U

static FATFS g_sd_fatfs;
static FIL g_log_files[FATFS_SD_NUM_FILES];
static char g_session_dir[FATFS_SD_DIR_PATH_MAX];
static uint8_t g_sd_mounted = 0U;
static uint8_t g_logger_active = 0U;
static uint32_t g_logger_rows_written = 0U;

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
  unsigned int index;
  FILINFO info;

  for (index = 1U; index <= FATFS_SD_SESSION_MAX; index++)
  {
    int len = snprintf(dir, dir_size, "0:/%s%04u", FATFS_SD_SESSION_PREFIX, index);
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
    /* 目录已存在，尝试下一个序号 */
  }

  return FR_DENIED;
}

static FRESULT FatFs_SD_OpenCsvFile(FIL *file, const char *dir, const char *fname,
                                     const char *header)
{
  char path[FATFS_SD_FILE_PATH_MAX];
  FRESULT r;

  (void)snprintf(path, sizeof(path), "%s/%s", dir, fname);

  r = f_open(file, path, FA_CREATE_ALWAYS | FA_WRITE);
  if (r != FR_OK)
  {
    printf("[FatFs] 打开 %s 失败: %s (%d)\r\n", path, FatFs_SD_ResultToString(r), (int)r);
    return r;
  }

  r = FatFs_SD_WriteExact(file, header, (UINT)strlen(header));
  if (r != FR_OK)
  {
    (void)f_close(file);
    return r;
  }

  return f_sync(file);
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

  /* CSV 表头定义 */
  static const char kHdrLsmAcc[] = "frame_id,tick_ms,acc_x_mg,acc_y_mg,acc_z_mg\r\n";
  static const char kHdrLsmGyr[] = "frame_id,tick_ms,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps\r\n";
  static const char kHdrLsmTmp[] = "frame_id,tick_ms,temp_C\r\n";
  static const char kHdrH3Acc[]  = "frame_id,tick_ms,raw_x,raw_y,raw_z,acc_x_mg,acc_y_mg,acc_z_mg\r\n";
  static const char kHdrQmaAcc[] = "frame_id,tick_ms,raw_x,raw_y,raw_z,acc_x_mg,acc_y_mg,acc_z_mg\r\n";

  static const char *headers[FATFS_SD_NUM_FILES] = {
    kHdrLsmAcc, kHdrLsmGyr, kHdrLsmTmp, kHdrH3Acc, kHdrQmaAcc
  };
  static const char *fnames[FATFS_SD_NUM_FILES] = {
    FATFS_SD_FNAME_LSM_ACC, FATFS_SD_FNAME_LSM_GYR, FATFS_SD_FNAME_LSM_TMP,
    FATFS_SD_FNAME_H3_ACC,  FATFS_SD_FNAME_QMA_ACC
  };

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

  /* 打开 5 个 CSV 文件并写入表头 */
  for (uint32_t i = 0U; i < FATFS_SD_NUM_FILES; i++)
  {
    result = FatFs_SD_OpenCsvFile(&g_log_files[i], g_session_dir, fnames[i], headers[i]);
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

  printf("[FatFs] 会话目录已创建: %s\r\n", g_session_dir);
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

  /* LSM6DSOX acc — 文件索引 0 */
  if (frame->lsm6dsox.valid != 0U)
  {
    len = snprintf(line, sizeof(line), "%lu,%lu,%.1f,%.1f,%.1f\r\n",
                   (unsigned long)frame->frame_id,
                   (unsigned long)frame->tick_ms,
                   frame->lsm6dsox.data.acc.x,
                   frame->lsm6dsox.data.acc.y,
                   frame->lsm6dsox.data.acc.z);
    if ((len > 0) && ((size_t)len < sizeof(line)))
      result = FatFs_SD_WriteExact(&g_log_files[0], line, (UINT)len);
    if (result != FR_OK) return result;
  }

  /* LSM6DSOX gyro — 文件索引 1 */
  if (frame->lsm6dsox.valid != 0U)
  {
    len = snprintf(line, sizeof(line), "%lu,%lu,%.1f,%.1f,%.1f\r\n",
                   (unsigned long)frame->frame_id,
                   (unsigned long)frame->tick_ms,
                   frame->lsm6dsox.data.gyro.x,
                   frame->lsm6dsox.data.gyro.y,
                   frame->lsm6dsox.data.gyro.z);
    if ((len > 0) && ((size_t)len < sizeof(line)))
      result = FatFs_SD_WriteExact(&g_log_files[1], line, (UINT)len);
    if (result != FR_OK) return result;
  }

  /* LSM6DSOX temp — 文件索引 2 */
  if (frame->lsm6dsox.valid != 0U)
  {
    len = snprintf(line, sizeof(line), "%lu,%lu,%.1f\r\n",
                   (unsigned long)frame->frame_id,
                   (unsigned long)frame->tick_ms,
                   frame->lsm6dsox.data.temp_C);
    if ((len > 0) && ((size_t)len < sizeof(line)))
      result = FatFs_SD_WriteExact(&g_log_files[2], line, (UINT)len);
    if (result != FR_OK) return result;
  }

  /* H3LIS100DL acc — 文件索引 3 */
  if (frame->h3lis100dl.valid != 0U)
  {
    len = snprintf(line, sizeof(line), "%lu,%lu,%d,%d,%d,%.1f,%.1f,%.1f\r\n",
                   (unsigned long)frame->frame_id,
                   (unsigned long)frame->tick_ms,
                   (int)frame->h3lis100dl.data.raw[0],
                   (int)frame->h3lis100dl.data.raw[1],
                   (int)frame->h3lis100dl.data.raw[2],
                   frame->h3lis100dl.data.acc_mg[0],
                   frame->h3lis100dl.data.acc_mg[1],
                   frame->h3lis100dl.data.acc_mg[2]);
    if ((len > 0) && ((size_t)len < sizeof(line)))
      result = FatFs_SD_WriteExact(&g_log_files[3], line, (UINT)len);
    if (result != FR_OK) return result;
  }

  /* QMA6100P acc — 文件索引 4 */
  if (frame->qma6100p.valid != 0U)
  {
    len = snprintf(line, sizeof(line), "%lu,%lu,%d,%d,%d,%.1f,%.1f,%.1f\r\n",
                   (unsigned long)frame->frame_id,
                   (unsigned long)frame->tick_ms,
                   (int)frame->qma6100p.data.raw[0],
                   (int)frame->qma6100p.data.raw[1],
                   (int)frame->qma6100p.data.raw[2],
                   frame->qma6100p.data.acc_mg[0],
                   frame->qma6100p.data.acc_mg[1],
                   frame->qma6100p.data.acc_mg[2]);
    if ((len > 0) && ((size_t)len < sizeof(line)))
      result = FatFs_SD_WriteExact(&g_log_files[4], line, (UINT)len);
    if (result != FR_OK) return result;
  }

  g_logger_rows_written++;
  return FR_OK;
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

  printf("[FatFs] sync rows=%lu result=%s (%d)\r\n",
         (unsigned long)g_logger_rows_written,
         FatFs_SD_ResultToString(result),
         (int)result);
  return result;
}

void FatFs_SD_LoggerStop(void)
{
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
