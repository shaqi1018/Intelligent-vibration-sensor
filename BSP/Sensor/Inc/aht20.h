/**
  ******************************************************************************
  * @file    aht20.h
  * @brief   AHT20 温湿度传感器驱动（I2C1，7-bit 地址 0x38）
  *
  *  时序：上电≥40ms → (可选)校准 → 触发测量(AC 33 00) → 等80ms → 读7字节。
  *  采样周期须 ≥1s（手册自发热/精度约束）。
  ******************************************************************************
  */
#ifndef __AHT20_H__
#define __AHT20_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 初始化：上电延时 + 读状态字，若未校准则发初始化命令。返回 HAL_OK=在线且就绪。 */
HAL_StatusTypeDef AHT20_Init(void);

/* 触发一次测量（AC 33 00）。之后任务应 osDelay(80) 再调 AHT20_ReadResult。 */
HAL_StatusTypeDef AHT20_TriggerMeasure(void);

/* 读取测量结果。temp_c=℃，humidity_pct=%RH。
 * 返回 HAL_OK=成功；HAL_BUSY=数据未就绪(busy)；HAL_ERROR=I2C失败或CRC错。 */
HAL_StatusTypeDef AHT20_ReadResult(float *temp_c, float *humidity_pct);

#ifdef __cplusplus
}
#endif

#endif /* __AHT20_H__ */
