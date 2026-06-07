#include "app_time.h"
#include "rtc_pcf85063.h"
#include "stm32u5xx_hal.h"

/* DWT µs 读取（160MHz → 每 tick = 6.25ns，除以 160 得 µs） */
static inline uint32_t DwtUs(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static uint32_t s_anchor_epoch_s  = 0U;
static uint32_t s_anchor_dwt_us   = 0U;
static uint32_t s_dwt_prev        = 0U;  /* 溢出检测 */
static uint32_t s_wrap_count      = 0U;
static uint8_t  s_synced          = 0U;

uint8_t AppTime_Sync(void)
{
  /* 确保 DWT 已启用 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

  Pcf85063_Time_t t;
  if (Pcf85063_Init() != PCF85063_OK || Pcf85063_GetTime(&t) != PCF85063_OK)
  {
    /* RTC 不可达：以启动时间为基准，epoch_s=0 */
    s_anchor_epoch_s = 0U;
    s_anchor_dwt_us  = DwtUs();
    s_dwt_prev       = s_anchor_dwt_us;
    s_wrap_count     = 0U;
    s_synced         = 1U;
    return 0U;
  }

  /* 读完 RTC 立即采样 DWT，最小化锚点误差 */
  s_anchor_epoch_s = Pcf85063_ToEpochSeconds(&t);
  s_anchor_dwt_us  = DwtUs();
  s_dwt_prev       = s_anchor_dwt_us;
  s_wrap_count     = 0U;
  s_synced         = 1U;
  return 1U;
}

uint64_t AppTime_GetEpochUs(void)
{
  if (s_synced == 0U) { return 0ULL; }

  uint32_t now = DwtUs();

  /* uint32 溢出检测（DWT µs 约 71 分钟绕回） */
  if (now < s_dwt_prev) { s_wrap_count++; }
  s_dwt_prev = now;

  uint64_t offset_us = (uint64_t)s_wrap_count * 0xFFFFFFFFULL
                     + (uint64_t)(now - s_anchor_dwt_us);

  return (uint64_t)s_anchor_epoch_s * 1000000ULL + offset_us;
}

uint32_t AppTime_GetAnchorEpochS(void)
{
  return s_anchor_epoch_s;
}
