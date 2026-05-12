/**
  ******************************************************************************
  * @file    frame_ring.h
  * @brief   单生产者-双消费者无锁环形缓冲（SensorBinFrame_t）
  *
  *          消费者 0 = SD logger
  *          消费者 1 = USB upload
  *
  *          生产者（sensorAcqTask）只调用 FrameRing_Push。
  *          两个消费者各自独立维护 tail 指针，互不干扰。
  *          当某消费者落后超过 FRAME_RING_DEPTH，该消费者发生丢帧（
  *          tail 被强制拉到 head - FRAME_RING_DEPTH，记入 dropped 计数）。
  *
  *          FRAME_RING_DEPTH 必须为 2 的幂次（512 × 64B = 32 KB BSS）。
  ******************************************************************************
  */
#ifndef __FRAME_RING_H__
#define __FRAME_RING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor_bin.h"
#include <stdint.h>

/* 容量必须为 2 的幂 */
#define FRAME_RING_DEPTH          512U
#define FRAME_RING_MASK           (FRAME_RING_DEPTH - 1U)

#define FRAME_RING_CONSUMER_SD    0U
#define FRAME_RING_CONSUMER_USB   1U
#define FRAME_RING_NUM_CONSUMERS  2U

/**
 * @brief 初始化，清零所有消费者（默认禁用）。
 *        在 MX_FREERTOS_Init 中调用一次。
 */
void     FrameRing_Init(void);

/**
 * @brief 启用或禁用消费者；启用时将其 tail 对齐到当前 head（从当前时刻开始读）。
 */
void     FrameRing_EnableConsumer(uint8_t cid, uint8_t en);

/**
 * @brief 生产者推入一帧（仅由 sensorAcqTask 调用，不加锁）。
 */
void     FrameRing_Push(const SensorBinFrame_t *frame);

/**
 * @brief 消费者弹出一帧（每个消费者在自己的任务中调用）。
 * @return 1=成功  0=空
 */
uint32_t FrameRing_Pop(uint8_t cid, SensorBinFrame_t *frame);

/**
 * @brief 返回消费者可读的帧数。
 */
uint32_t FrameRing_Available(uint8_t cid);

/**
 * @brief 读取并清零该消费者的丢帧计数。
 */
uint32_t FrameRing_GetDropped(uint8_t cid);

/**
 * @brief 将消费者 tail 快进到当前 head（丢弃所有积压帧）。
 */
void     FrameRing_Flush(uint8_t cid);

/**
 * @brief 获取全局生产者头指针（用于诊断）。
 */
uint32_t FrameRing_GetHead(void);

#ifdef __cplusplus
}
#endif

#endif /* __FRAME_RING_H__ */
