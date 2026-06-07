#ifndef APP_TIME_H
#define APP_TIME_H

#include <stdint.h>

/* 从 RTC 读取当前时间并记录 DWT 锚点。
 * 应在 FreeRTOS 启动后、采集开始前调用。
 * 返回 1=成功，0=I2C 失败（时间戳退化为启动相对值）。 */
uint8_t AppTime_Sync(void);

/* 返回当前时刻的 Unix 纪元微秒数（UTC）。精度 ~1µs（DWT 160MHz）。
 * 调用 AppTime_Sync 之前返回 0。 */
uint64_t AppTime_GetEpochUs(void);

/* 返回 AppTime_Sync 时刻对应的 epoch 秒（供日志打印）。 */
uint32_t AppTime_GetAnchorEpochS(void);

#endif
