#include "app_time.h"
#include "rtc_pcf85063.h"
#include "stm32u5xx_hal.h"
#include "app_locks.h"

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
  /* I2C2 is shared with the ES8311 codec — serialize the whole RTC read so it
   * never interleaves with a Mic_Start/Stop codec sequence (M1). && preserves
   * the original short-circuit: GetTime is skipped if Init fails. */
  AppI2c2Lock();
  uint8_t rtc_ok = (Pcf85063_Init() == PCF85063_OK) && (Pcf85063_GetTime(&t) == PCF85063_OK);
  AppI2c2Unlock();
  if (rtc_ok == 0U)
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

/* DWT µs 绕回周期：2^32 cycles / 160MHz = 26843545.6 µs，取整为 26843546 */
#define APP_TIME_DWT_US_PERIOD  26843546ULL

uint64_t AppTime_GetEpochUs(void)
{
  if (s_synced == 0U) { return 0ULL; }

  /* 临界区保护：LSM/H3/QMA 三个 task 并发调用此函数。
   * now 必须在临界区【内】采样。否则 Task A 采完 now 被切走、Task B 用更大的
   * now 更新了 s_dwt_prev，Task A 回来 now < s_dwt_prev → 假溢出 → 整段 +26.84s。
   * 重载下(6664Hz 多任务高频调用)实测时间戳偏快约 1.8×(37.9s 被记成 68s)，
   * 即多次假绕回。把 DwtUs() 与 wrap 判定/更新一起放进临界区，三者原子，根除。 */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t now = DwtUs();
  if (now < s_dwt_prev) { s_wrap_count++; }
  s_dwt_prev = now;
  uint32_t wraps = s_wrap_count;
  __set_PRIMASK(primask);

  /* 有符号 delta：wrap 后 now < s_anchor_dwt_us 时 uint32 减法下溢，
   * 转为 int32 可正确还原负值；wraps * PERIOD 补偿整圈时间。 */
  int32_t  signed_delta = (int32_t)(now - s_anchor_dwt_us);
  int64_t  offset_us    = (int64_t)wraps * (int64_t)APP_TIME_DWT_US_PERIOD
                        + (int64_t)signed_delta;
  if (offset_us < 0) { offset_us = 0; }

  return (uint64_t)s_anchor_epoch_s * 1000000ULL + (uint64_t)offset_us;
}

uint32_t AppTime_GetAnchorEpochS(void)
{
  return s_anchor_epoch_s;
}
