/**
  ******************************************************************************
  * @file    acq_timer.h
  * @brief   TIM2 高速采样触发模块（1Hz ~ 10KHz）
  *
  *          采用 TIM2 作为采样基准定时器，每次溢出在 ISR 中调用
  *          vTaskNotifyGiveFromISR 唤醒主采集任务，从而摆脱 osDelay 1ms
  *          tick 的精度限制。
  *
  *          时钟规划（SYSCLK = 160MHz, APB1 = 160MHz, TIM2 内部时钟 = 160MHz）：
  *            sample_rate_hz = 160_000_000 / ((PSC+1) * (ARR+1))
  *          通过把 PSC 固定为某个除数，再用 ARR 调采样率。
  ******************************************************************************
  */

#ifndef __ACQ_TIMER_H__
#define __ACQ_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define ACQ_TIMER_HZ_MIN              1U
#define ACQ_TIMER_HZ_MAX              10000U
#define ACQ_TIMER_HZ_DEFAULT          1000U

extern TIM_HandleTypeDef htim2;

/**
 * @brief 初始化 TIM2 但不启动；默认采样率 1KHz。
 */
HAL_StatusTypeDef AcqTimer_Init(void);

/**
 * @brief 设置采样率（不启动）。可在停止后调用，运行中调用会平滑切换。
 * @param sample_rate_hz 1..10000
 * @return HAL_OK 成功；HAL_ERROR 参数越界
 */
HAL_StatusTypeDef AcqTimer_SetRate(uint32_t sample_rate_hz);

/**
 * @brief 启动 TIM2 中断（开始周期性触发）。
 */
HAL_StatusTypeDef AcqTimer_Start(void);

/**
 * @brief 停止 TIM2 中断。
 */
HAL_StatusTypeDef AcqTimer_Stop(void);

/**
 * @brief 注册被唤醒的任务句柄（FreeRTOS TaskHandle_t）。
 *        每次定时器到期后 ISR 会用 vTaskNotifyGiveFromISR 通知该任务。
 *        传 NULL 表示注销。
 */
void AcqTimer_RegisterTask(void *task_handle);

/**
 * @brief 获取实际生效的采样率（精确到整数 Hz）。
 */
uint32_t AcqTimer_GetEffectiveRate(void);

/**
 * @brief 获取当前 PSC/ARR 寄存器配置（用于诊断输出）。
 */
void AcqTimer_GetTiming(uint32_t *psc, uint32_t *arr);

/**
 * @brief 获取 ISR 内累计计数（含丢失计数），用于诊断。
 */
uint32_t AcqTimer_GetTickCount(void);

/**
 * @brief 获取 64 位微秒时间戳（基于 TIM2 计数推算）。
 *        用于二进制帧的 tick_us 字段。
 */
uint64_t AcqTimer_GetTimestampUs(void);

/**
 * @brief TIM2 IRQ 入口（在 stm32u5xx_it.c 中转发）。
 */
void AcqTimer_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __ACQ_TIMER_H__ */
