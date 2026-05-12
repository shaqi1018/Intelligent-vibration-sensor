/**
  ******************************************************************************
  * @file    frame_ring.c
  * @brief   单生产者-双消费者无锁环形缓冲（SensorBinFrame_t）
  *
  *  生产者（sensorAcqTask）只写 head；
  *  每个消费者各自维护独立的 tail，彼此不干扰。
  *  使用 volatile + __DSB/__DMB 内存屏障替代互斥锁，
  *  确保 Cortex-M33 上的可见性。
  ******************************************************************************
  */
#include "frame_ring.h"
#include "main.h"            /* 引入 stm32u5xx_hal.h → core_cm33.h → __DSB */
#include <string.h>
#include "cmsis_os2.h"

/* ========================= 静态存储 ======================================= */

/* 帧缓冲区放在 .bss（512 × 64B = 32 KB） */
static SensorBinFrame_t s_ring[FRAME_RING_DEPTH];

/* 生产者头指针（单调递增，取模寻址） */
static volatile uint32_t s_head = 0U;

/* 每个消费者的尾指针 + 使能标志 + 丢帧计数 */
typedef struct {
  volatile uint32_t tail;
  volatile uint8_t  enabled;
  volatile uint32_t dropped;
} ConsumerState_t;

static ConsumerState_t s_consumers[FRAME_RING_NUM_CONSUMERS];

/* ========================= API 实现 ======================================= */

void FrameRing_Init(void)
{
  uint8_t i;
  memset(s_ring, 0, sizeof(s_ring));
  s_head = 0U;
  for (i = 0U; i < FRAME_RING_NUM_CONSUMERS; i++)
  {
    s_consumers[i].tail    = 0U;
    s_consumers[i].enabled = 0U;
    s_consumers[i].dropped = 0U;
  }
}

void FrameRing_EnableConsumer(uint8_t cid, uint8_t en)
{
  if (cid >= FRAME_RING_NUM_CONSUMERS) { return; }
  if (en != 0U)
  {
    /* 从当前 head 开始消费，不读历史帧 */
    s_consumers[cid].tail    = s_head;
    s_consumers[cid].dropped = 0U;
    s_consumers[cid].enabled = 1U;
  }
  else
  {
    s_consumers[cid].enabled = 0U;
  }
}

void FrameRing_Push(const SensorBinFrame_t *frame)
{
  uint32_t head;
  uint8_t  i;
  uint32_t avail;

  if (frame == NULL) { return; }

  head = s_head;

  /* 检查每个启用的消费者是否落后超过 DEPTH-1（即将被覆盖） */
  for (i = 0U; i < FRAME_RING_NUM_CONSUMERS; i++)
  {
    if (s_consumers[i].enabled == 0U) { continue; }
    avail = head - s_consumers[i].tail;
    if (avail >= FRAME_RING_DEPTH)
    {
      /* 消费者太慢，强制推进 tail 到 head - (DEPTH-1) */
      s_consumers[i].tail = head - (FRAME_RING_DEPTH - 1U);
      s_consumers[i].dropped++;
    }
  }

  /* 写入帧 */
  s_ring[head & FRAME_RING_MASK] = *frame;

  /* 写屏障：确保帧数据在 head 更新前对消费者可见 */
  __DSB();

  s_head = head + 1U;
}

uint32_t FrameRing_Pop(uint8_t cid, SensorBinFrame_t *frame)
{
  uint32_t tail;
  uint32_t head;

  if ((cid >= FRAME_RING_NUM_CONSUMERS) || (frame == NULL)) { return 0U; }
  if (s_consumers[cid].enabled == 0U)                       { return 0U; }

  tail = s_consumers[cid].tail;
  head = s_head;

  /* 读屏障 */
  __DSB();

  if (tail == head) { return 0U; }  /* 空 */

  *frame = s_ring[tail & FRAME_RING_MASK];
  s_consumers[cid].tail = tail + 1U;
  return 1U;
}

uint32_t FrameRing_Available(uint8_t cid)
{
  if (cid >= FRAME_RING_NUM_CONSUMERS)    { return 0U; }
  if (s_consumers[cid].enabled == 0U)     { return 0U; }
  return s_head - s_consumers[cid].tail;
}

uint32_t FrameRing_GetDropped(uint8_t cid)
{
  uint32_t d;
  if (cid >= FRAME_RING_NUM_CONSUMERS) { return 0U; }
  d = s_consumers[cid].dropped;
  s_consumers[cid].dropped = 0U;
  return d;
}

void FrameRing_Flush(uint8_t cid)
{
  if (cid >= FRAME_RING_NUM_CONSUMERS) { return; }
  s_consumers[cid].tail = s_head;
}

uint32_t FrameRing_GetHead(void)
{
  return s_head;
}
