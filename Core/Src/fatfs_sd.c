#include "fatfs_sd.h"

#include <stdio.h>
#include <string.h>

#include "diskio.h"
#include "sd_diskio.h"
#include "sdmmc.h"

#define FATFS_SD_LOG_DIR        "0:/LOG"
#define FATFS_SD_LOG_PATH_MAX   32U

static FATFS g_sd_fatfs;
static FIL g_log_file;
static char g_log_path[FATFS_SD_LOG_PATH_MAX];
static uint8_t g_sd_mounted = 0U;
static uint8_t g_logger_active = 0U;

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

static uint32_t FatFs_SD_ComputeAgeMs(uint32_t tick_ms, uint32_t last_update_ms)
{
  return tick_ms - last_update_ms;
}

static uint8_t FatFs_SD_IsFresh(uint8_t valid, uint32_t age_ms)
{
  if ((valid == 0U) || (age_ms > APP_SENSOR_STALE_TIMEOUT_MS))
  {
    return 0U;
  }

  return 1U;
}

static FRESULT FatFs_SD_EnsureLogDirectory(void)
{
  FRESULT result;
  FILINFO info;

  result = f_stat(FATFS_SD_LOG_DIR, &info);
  if (result == FR_OK)
  {
    return ((info.fattrib & AM_DIR) != 0U) ? FR_OK : FR_EXIST;
  }

  if (result != FR_NO_FILE)
  {
    return result;
  }

  return f_mkdir(FATFS_SD_LOG_DIR);
}

static FRESULT FatFs_SD_FindNextLogPath(char *path, size_t path_size)
{
  unsigned int index;
  FILINFO info;

  for (index = 1U; index <= 9999U; index++)
  {
    int path_len = snprintf(path, path_size, FATFS_SD_LOG_DIR "/LOG%04u.CSV", index);
    if ((path_len < 0) || ((size_t)path_len >= path_size))
    {
      return FR_INVALID_NAME;
    }

    switch (f_stat(path, &info))
    {
      case FR_OK:
        break;
      case FR_NO_FILE:
        return FR_OK;
      default:
        return FR_DISK_ERR;
    }
  }

  return FR_DENIED;
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

  memset(&g_sd_fatfs, 0, sizeof(g_sd_fatfs));
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
    (void)f_close(&g_log_file);
    g_logger_active = 0U;
    g_log_path[0] = '\0';
  }

  if (g_sd_mounted != 0U)
  {
    (void)f_mount(NULL, "0:", 1U);
    g_sd_mounted = 0U;
  }
}

FRESULT FatFs_SD_LoggerStart(void)
{
  FRESULT result;
  static const char kCsvHeader[] =
      "tick_ms,row_seq,"
      "lsm_valid,lsm_age_ms,lsm_acc_x_mg,lsm_acc_y_mg,lsm_acc_z_mg,lsm_gyro_x_mdps,lsm_gyro_y_mdps,lsm_gyro_z_mdps,lsm_temp_c,"
      "h3_valid,h3_age_ms,h3_raw_x,h3_raw_y,h3_raw_z,h3_acc_x_mg,h3_acc_y_mg,h3_acc_z_mg,"
      "qma_valid,qma_age_ms,qma_raw_x,qma_raw_y,qma_raw_z,qma_acc_x_mg,qma_acc_y_mg,qma_acc_z_mg\r\n";

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

  result = FatFs_SD_EnsureLogDirectory();
  if (result != FR_OK)
  {
    FatFs_SD_Unmount();
    return result;
  }

  result = FatFs_SD_FindNextLogPath(g_log_path, sizeof(g_log_path));
  if (result != FR_OK)
  {
    FatFs_SD_Unmount();
    return result;
  }

  result = f_open(&g_log_file, g_log_path, FA_CREATE_ALWAYS | FA_WRITE);
  if (result != FR_OK)
  {
    FatFs_SD_Unmount();
    return result;
  }

  result = FatFs_SD_WriteExact(&g_log_file, kCsvHeader, (UINT)(sizeof(kCsvHeader) - 1U));
  if (result != FR_OK)
  {
    (void)f_close(&g_log_file);
    FatFs_SD_Unmount();
    return result;
  }

  result = f_sync(&g_log_file);
  if (result != FR_OK)
  {
    (void)f_close(&g_log_file);
    FatFs_SD_Unmount();
    return result;
  }

  g_logger_active = 1U;
  printf("[FatFs] 日志文件已打开: %s\r\n", g_log_path);
  return FR_OK;
}

FRESULT FatFs_SD_LoggerAppendRow(const AppSensorSnapshot_t *snapshot, uint32_t tick_ms, uint32_t row_seq)
{
  FRESULT result;
  char line[512];
  uint32_t lsm_age_ms;
  uint32_t h3_age_ms;
  uint32_t qma_age_ms;
  uint8_t lsm_valid;
  uint8_t h3_valid;
  uint8_t qma_valid;
  int line_len;

  if (snapshot == NULL)
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

  lsm_age_ms = FatFs_SD_ComputeAgeMs(tick_ms, snapshot->lsm6dsox.last_update_ms);
  h3_age_ms = FatFs_SD_ComputeAgeMs(tick_ms, snapshot->h3lis100dl.last_update_ms);
  qma_age_ms = FatFs_SD_ComputeAgeMs(tick_ms, snapshot->qma6100p.last_update_ms);

  lsm_valid = FatFs_SD_IsFresh(snapshot->lsm6dsox.valid, lsm_age_ms);
  h3_valid = FatFs_SD_IsFresh(snapshot->h3lis100dl.valid, h3_age_ms);
  qma_valid = FatFs_SD_IsFresh(snapshot->qma6100p.valid, qma_age_ms);

  line_len = snprintf(
      line,
      sizeof(line),
      "%lu,%lu,"
      "%u,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
      "%u,%lu,%d,%d,%d,%.1f,%.1f,%.1f,"
      "%u,%lu,%d,%d,%d,%.1f,%.1f,%.1f\r\n",
      (unsigned long)tick_ms,
      (unsigned long)row_seq,
      (unsigned int)lsm_valid,
      (unsigned long)lsm_age_ms,
      snapshot->lsm6dsox.data.acc.x,
      snapshot->lsm6dsox.data.acc.y,
      snapshot->lsm6dsox.data.acc.z,
      snapshot->lsm6dsox.data.gyro.x,
      snapshot->lsm6dsox.data.gyro.y,
      snapshot->lsm6dsox.data.gyro.z,
      snapshot->lsm6dsox.data.temp_C,
      (unsigned int)h3_valid,
      (unsigned long)h3_age_ms,
      (int)snapshot->h3lis100dl.data.raw[0],
      (int)snapshot->h3lis100dl.data.raw[1],
      (int)snapshot->h3lis100dl.data.raw[2],
      snapshot->h3lis100dl.data.acc_mg[0],
      snapshot->h3lis100dl.data.acc_mg[1],
      snapshot->h3lis100dl.data.acc_mg[2],
      (unsigned int)qma_valid,
      (unsigned long)qma_age_ms,
      (int)snapshot->qma6100p.data.raw[0],
      (int)snapshot->qma6100p.data.raw[1],
      (int)snapshot->qma6100p.data.raw[2],
      snapshot->qma6100p.data.acc_mg[0],
      snapshot->qma6100p.data.acc_mg[1],
      snapshot->qma6100p.data.acc_mg[2]);

  if ((line_len < 0) || ((size_t)line_len >= sizeof(line)))
  {
    return FR_INT_ERR;
  }

  result = FatFs_SD_WriteExact(&g_log_file, line, (UINT)line_len);
  if (result != FR_OK)
  {
    return result;
  }

  return FR_OK;
}

FRESULT FatFs_SD_LoggerSync(void)
{
  if (g_logger_active == 0U)
  {
    return FR_NOT_ENABLED;
  }

  return f_sync(&g_log_file);
}

void FatFs_SD_LoggerStop(void)
{
  if (g_logger_active != 0U)
  {
    (void)f_sync(&g_log_file);
    (void)f_close(&g_log_file);
    g_logger_active = 0U;
    g_log_path[0] = '\0';
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
