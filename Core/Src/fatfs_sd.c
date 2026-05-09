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
      "frame_id,tick_ms,enabled_mask,present_mask,"
      "lsm_sample_seq,lsm_valid,lsm_acc_x_mg,lsm_acc_y_mg,lsm_acc_z_mg,lsm_gyro_x_mdps,lsm_gyro_y_mdps,lsm_gyro_z_mdps,lsm_temp_c,"
      "h3_sample_seq,h3_valid,h3_raw_x,h3_raw_y,h3_raw_z,h3_acc_x_mg,h3_acc_y_mg,h3_acc_z_mg,"
      "qma_sample_seq,qma_valid,qma_raw_x,qma_raw_y,qma_raw_z,qma_acc_x_mg,qma_acc_y_mg,qma_acc_z_mg\r\n";

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

  printf("[FatFs] 日志文件已打开: %s\r\n", g_log_path);
  g_logger_active = 1U;
  g_logger_rows_written = 0U;
  return FR_OK;
}

FRESULT FatFs_SD_LoggerAppendFrame(const AppSensorFrame_t *frame)
{
  FRESULT result;
  char line[512];
  int line_len;

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

  line_len = snprintf(
      line,
      sizeof(line),
      "%lu,%lu,0x%02lX,0x%02lX,"
      "%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
      "%lu,%u,%d,%d,%d,%.1f,%.1f,%.1f,"
      "%lu,%u,%d,%d,%d,%.1f,%.1f,%.1f\r\n",
      (unsigned long)frame->frame_id,
      (unsigned long)frame->tick_ms,
      (unsigned long)frame->enabled_mask,
      (unsigned long)frame->present_mask,
      (unsigned long)frame->lsm6dsox.sample_seq,
      (unsigned int)frame->lsm6dsox.valid,
      frame->lsm6dsox.data.acc.x,
      frame->lsm6dsox.data.acc.y,
      frame->lsm6dsox.data.acc.z,
      frame->lsm6dsox.data.gyro.x,
      frame->lsm6dsox.data.gyro.y,
      frame->lsm6dsox.data.gyro.z,
      frame->lsm6dsox.data.temp_C,
      (unsigned long)frame->h3lis100dl.sample_seq,
      (unsigned int)frame->h3lis100dl.valid,
      (int)frame->h3lis100dl.data.raw[0],
      (int)frame->h3lis100dl.data.raw[1],
      (int)frame->h3lis100dl.data.raw[2],
      frame->h3lis100dl.data.acc_mg[0],
      frame->h3lis100dl.data.acc_mg[1],
      frame->h3lis100dl.data.acc_mg[2],
      (unsigned long)frame->qma6100p.sample_seq,
      (unsigned int)frame->qma6100p.valid,
      (int)frame->qma6100p.data.raw[0],
      (int)frame->qma6100p.data.raw[1],
      (int)frame->qma6100p.data.raw[2],
      frame->qma6100p.data.acc_mg[0],
      frame->qma6100p.data.acc_mg[1],
      frame->qma6100p.data.acc_mg[2]);

  if ((line_len < 0) || ((size_t)line_len >= sizeof(line)))
  {
    return FR_INT_ERR;
  }

  result = FatFs_SD_WriteExact(&g_log_file, line, (UINT)line_len);
  if (result != FR_OK)
  {
    return result;
  }

  g_logger_rows_written++;
  return FR_OK;
}

FRESULT FatFs_SD_LoggerSync(void)
{
  FRESULT result;

  if (g_logger_active == 0U)
  {
    return FR_NOT_ENABLED;
  }

  result = f_sync(&g_log_file);
  printf("[FatFs] sync rows=%lu result=%s (%d)\r\n",
         (unsigned long)g_logger_rows_written,
         FatFs_SD_ResultToString(result),
         (int)result);
  return result;
}

void FatFs_SD_LoggerStop(void)
{
  FRESULT sync_result = FR_OK;
  FRESULT close_result = FR_OK;

  if (g_logger_active != 0U)
  {
    sync_result = f_sync(&g_log_file);
    close_result = f_close(&g_log_file);
    printf("[FatFs] logger stop rows=%lu sync=%s (%d) close=%s (%d) file=%s\r\n",
           (unsigned long)g_logger_rows_written,
           FatFs_SD_ResultToString(sync_result),
           (int)sync_result,
           FatFs_SD_ResultToString(close_result),
           (int)close_result,
           g_log_path);
    g_logger_active = 0U;
    g_logger_rows_written = 0U;
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
