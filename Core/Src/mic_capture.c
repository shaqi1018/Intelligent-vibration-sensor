#include "mic_capture.h"

#include "sai.h"
#include "es8311.h"
#include "board_io.h"
#include "cmsis_os2.h"
#include <string.h>

/* DMA 目标：双半缓冲，半字（int16）。每半 1024 样本 = 2KB。
 * 半满/全满回调各搬运一半到 g_ring_mic（SPSC 生产者侧）。 */
#define MIC_DMA_HALF_SAMPLES   1024U
static int16_t s_dma_buf[MIC_DMA_HALF_SAMPLES * 2U];
static volatile uint32_t s_dropped = 0U;

/* 由 app_freertos.c 暴露（写 g_ring_mic，SPSC 生产者侧）。
 * Task 6 之前为未解析 extern —— 这是文档化的跨任务接口，不是占位。 */
extern uint32_t AppRing_WriteMic(const uint8_t *src, uint32_t len);

static void mic_push(const int16_t *p, uint32_t nsamp)
{
  uint32_t w = AppRing_WriteMic((const uint8_t *)p, nsamp * 2U);
  if (w == 0U) s_dropped += nsamp * 2U;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  (void)hsai;
  mic_push(&s_dma_buf[0], MIC_DMA_HALF_SAMPLES);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  (void)hsai;
  mic_push(&s_dma_buf[MIC_DMA_HALF_SAMPLES], MIC_DMA_HALF_SAMPLES);
}

int Mic_Start(uint32_t sample_rate_hz, uint16_t gain_db)
{
  PaEn_Set(1);
  osDelay(50);                              /* 等 codec 供电稳定 */
  if (ES8311_Probe() != 0) { PaEn_Set(0); return -1; }
  if (ES8311_InitAdc(sample_rate_hz, gain_db) != 0) { PaEn_Set(0); return -2; }
  if (MX_SAI1_Init(sample_rate_hz) != HAL_OK) { ES8311_PowerDown(); PaEn_Set(0); return -3; }
  s_dropped = 0U;
  /* size 参数单位为“数据项个数”：本设计双半缓冲，半满/全满各触发一次，
   * 故传两半总样本数（MIC_DMA_HALF_SAMPLES*2）并依赖 DMA 循环模式。
   * 确切的 item-count 语义在台架（Task 7）确认，必要时改为单半大小。 */
  if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)s_dma_buf,
                          MIC_DMA_HALF_SAMPLES * 2U) != HAL_OK) {
    MX_SAI1_DeInit(); ES8311_PowerDown(); PaEn_Set(0); return -4;
  }
  return 0;
}

void Mic_Stop(void)
{
  HAL_SAI_DMAStop(&hsai_BlockA1);
  MX_SAI1_DeInit();
  ES8311_PowerDown();
  PaEn_Set(0);
}

uint32_t Mic_GetDropped(void) { return s_dropped; }
