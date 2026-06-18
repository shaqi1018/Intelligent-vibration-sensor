#include "mic_capture.h"

#include "sai.h"
#include "es8311.h"
#include "board_io.h"
#include "cmsis_os2.h"
#include <string.h>

/* DMA 目标：双半缓冲，半字（int16）。每半 1024 样本 = 2KB。
 * SAI 以立体声帧运行（I2S 标准要求 2 slot），ES8311 单声道 ADC 在两个 slot
 * 输出相同数据（L=R）。mic_push 只取偶数索引（slot-0/L 声道），丢弃奇数索引（slot-1/R），
 * 向 ring 写入真实 16kHz 单声道样本。 */
#define MIC_DMA_HALF_SAMPLES   1024U
#define MIC_MONO_HALF          (MIC_DMA_HALF_SAMPLES / 2U)   /* 512 单声道样本 */
static int16_t s_dma_buf[MIC_DMA_HALF_SAMPLES * 2U];
static int16_t s_mono_buf[MIC_MONO_HALF];
static volatile uint32_t s_dropped = 0U;

/* 由 app_freertos.c 暴露（写 g_ring_mic，SPSC 生产者侧）。
 * Task 6 之前为未解析 extern —— 这是文档化的跨任务接口，不是占位。 */
extern uint32_t AppRing_WriteMic(const uint8_t *src, uint32_t len);

static void mic_push(const int16_t *p, uint32_t nsamp)
{
  /* p is [L0,R0,L1,R1,...] (nsamp=1024 stereo elements).
   * Extract L channel (even indices) → 512 true mono samples at 16 kHz. */
  (void)nsamp;
  for (uint32_t i = 0U; i < MIC_MONO_HALF; i++)
  {
    s_mono_buf[i] = p[i * 2U];
  }
  uint32_t w = AppRing_WriteMic((const uint8_t *)s_mono_buf, MIC_MONO_HALF * 2U);
  if (w == 0U) { s_dropped += MIC_MONO_HALF * 2U; }
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
  /* 失败回滚里不再 PaEn_Set(0):PAVCC 与 RTC 共用,关它会断 RTC 供电(I2C fail)。 */
  if (ES8311_Probe() != 0) { return -1; }
  if (ES8311_InitAdc(sample_rate_hz, gain_db) != 0) { return -2; }
  if (MX_SAI1_Init(sample_rate_hz) != HAL_OK) { ES8311_PowerDown(); return -3; }
  s_dropped = 0U;
  /* ⚠️ 台架第一要务（Task 7 / 见计划风险）：确认 HAL_SAI_Receive_DMA 的 size 单位。
   *   - 经典 HAL：size = 数据项个数 → 整个缓冲 = MIC_DMA_HALF_SAMPLES*2 = 2048 项（当前值，
   *     半满中断落在 &s_dma_buf[1024]，与两个回调一致）。
   *   - 若所用 U5 SAI/GPDMA HAL 把 size 当“字节数”：需改为 sizeof(s_dma_buf)=4096，
   *     否则只搬运半个缓冲、全满回调读到陈旧数据 → 音频损坏。
   * 添加 SAI HAL 源文件后，查 HAL_SAI_Receive_DMA 是否把 XferSize 直接当 GPDMA BNDT(字节)。
   * 用 1kHz 已知音验证：频谱峰在 1kHz 且无周期性断点即为正确。 */
  if (HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)s_dma_buf,
                          MIC_DMA_HALF_SAMPLES * 2U) != HAL_OK) {
    MX_SAI1_DeInit(); ES8311_PowerDown(); return -4;
  }
  return 0;
}

void Mic_Stop(void)
{
  HAL_SAI_DMAStop(&hsai_BlockA1);
  MX_SAI1_DeInit();
  ES8311_PowerDown();
  /* 不再 PaEn_Set(0):RTC(PCF85063)的 VDD/I2C 上拉与 ES8311 共用 PAVCC(PA_EN 门控),
   * 关 PA_EN 会连带把 RTC 断电 → I2C fail / 时间戳 F6。ES8311_PowerDown 已让 codec
   * 进低功耗,PAVCC 保持供电只多一点静态电流,换 RTC 始终在线,值得。 */
}

uint32_t Mic_GetDropped(void) { return s_dropped; }
