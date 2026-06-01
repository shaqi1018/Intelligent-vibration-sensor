/**
  ******************************************************************************
  * @file    device_config.h
  * @brief   SD卡设备配置读写模块
  *
  *          功能：
  *            1. 上电从 SD 卡根目录读取 DEVCFG.JSN（8.3格式）并应用到运行时配置
  *            2. 若文件不存在则写入用户友好的模板 JSON，使用内建缺省值
  *            3. 每次采集启动时在会话目录（CKBOXxxxx）中写入 CONFIG.JSN 快照
  *            4. 每次采集结束后将运行时配置写回根目录 DEVCFG.JSN
  *
  *          调用时机：
  *            DeviceCfg_LoadFromSD()         -- FreeRTOS 启动后、采集前
  *            DeviceCfg_WriteConfigToDir()   -- 采集启动时，FatFs_SD_LoggerStart 之后
  *            DeviceCfg_WriteCurrentToSD()   -- 采集结束时自动调用
  ******************************************************************************
  */

#ifndef __DEVICE_CONFIG_H__
#define __DEVICE_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ff.h"
#include "acq_config.h"

/**
 * @brief 从 SD 卡读取并应用 DEVCFG.JSN（8.3格式配置文件）。
 *
 *        - SD 未插入：静默跳过，返回 FR_NOT_READY
 *        - 文件不存在：写入默认模板，返回 FR_NO_FILE（不视为错误）
 *        - 解析成功：调用 AcqConfig_Set() 应用配置，返回 FR_OK
 *        - 参数越界：AcqConfig_Set 拒绝，返回 FR_INVALID_PARAMETER
 *
 * @note  本函数内部自行挂载 / 卸载 SD。调用前 SD 必须处于未挂载状态
 *        （即在 FatFs_SD_LoggerStart 之前调用）。
 */
FRESULT DeviceCfg_LoadFromSD(void);

/**
 * @brief 将当前运行时采集配置写回 SD 卡根目录的 DEVCFG.JSN。
 *
 *        每次 SD 采集会话结束时自动调用，确保下次上电能读取到最新配置。
 *        挂载 SD 后不卸载，由调用者负责后续清理。
 */
FRESULT DeviceCfg_WriteCurrentToSD(void);

/**
 * @brief 将当前运行时采集配置写入指定目录的 CONFIG.JSN。
 *
 * @param dir  目标目录路径，如 "0:/CKBOX0001"
 */
FRESULT DeviceCfg_WriteConfigToDir(const char *dir);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_CONFIG_H__ */
