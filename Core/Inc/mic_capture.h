#ifndef MIC_CAPTURE_H
#define MIC_CAPTURE_H

#include <stdint.h>

#include "sensor_snapshot.h"

extern AppRingBuffer_t g_ring_mic;   /* 定义在 app_freertos.c */

/* 上电 + 初始化 codec + 启动 SAI DMA 采集。
 * sample_rate_hz ∈ {8000,16000,48000}，gain_db = mic PGA 增益（0..42）。
 * 返回 0 成功；-1 codec 自检失败；-2 codec 初始化失败；
 *      -3 SAI 初始化失败；-4 SAI DMA 启动失败。 */
int      Mic_Start(uint32_t sample_rate_hz, uint16_t gain_db);

/* 停 SAI DMA + codec 下电 + 关电源门控。 */
void     Mic_Stop(void);

/* ring 溢出累计字节数（诊断用）。 */
uint32_t Mic_GetDropped(void);

/* SAI DMA 错误自恢复(L5)：HAL_SAI_ErrorCallback 置位，StartMicTask 轮询后停采+重启。
 * Mic_SaiErrorPending 返回是否有未处理错误；Mic_SaiErrorCode 取 HAL ErrorCode 快照；
 * Mic_ClearSaiError 在重启后清除标志。 */
uint8_t  Mic_SaiErrorPending(void);
uint32_t Mic_SaiErrorCode(void);
void     Mic_ClearSaiError(void);

#endif /* MIC_CAPTURE_H */
