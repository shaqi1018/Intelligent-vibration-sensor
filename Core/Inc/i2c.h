#ifndef I2C_H
#define I2C_H
#include "stm32u5xx_hal.h"
extern I2C_HandleTypeDef hi2c2;
void MX_I2C2_Init(void);

/* 某次 I2C2 传输失败后，hi2c2 可能停在非 READY 错误态，污染后续所有传输
 * （典型表现：ES8311 一次读失败后，之后 get_fattime 读 RTC 全部失败 →
 *  SD 文件时间戳整批 fallback）。DeInit+Init 重置外设与句柄，仅失败时调用。 */
void I2C2_BusRecover(void);

/* 探测 RTC(PCF85063, 0x51) 是否在 I2C2 上应答。1=应答(硬件OK), 0=无应答(硬件/链路问题)。 */
uint8_t I2C2_ProbeRtc(void);
#endif
