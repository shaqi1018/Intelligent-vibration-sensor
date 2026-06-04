#include "boot_mode.h"
#include "stm32u5xx_hal.h"

/* STM32U575: backup registers live in TAMP (APB3).
 * TAMP_BASE_NS = 0x46007C00, BKP0R at offset 0x100 -> 0x46007D00.
 * TAMP clock is gated by RCC_APB3ENR_RTCAPBEN (bit 21).
 * Must enable RTCAPBEN before any TAMP register access. */

#define RCC_APB3ENR   (*((volatile uint32_t *)0x46020CA8UL))
#define RTCAPBEN_BIT  (1UL << 21)

#undef TAMP_BKP0R
#define TAMP_BKP0R    (*((volatile uint32_t *)0x46007D00UL))

#define MAGIC_LOG     0xA5A5A5A5UL
#define MAGIC_MSC     0x5A5A5A5AUL
#define MAGIC_WCID    0xAA55AA5AUL

boot_mode_t BootMode_Read(void)
{
  RCC_APB3ENR |= RTCAPBEN_BIT;   /* enable TAMP/RTC clock */
  uint32_t v = TAMP_BKP0R;
  if (v == MAGIC_MSC)  return BOOT_MODE_USB_MSC;
  if (v == MAGIC_WCID) return BOOT_MODE_WCID_BULK;
  return BOOT_MODE_WCID_BULK;  /* default: WCID (CDC removed) */
}

void BootMode_Write(boot_mode_t mode)
{
  uint32_t magic;
  switch (mode) {
    case BOOT_MODE_USB_MSC:   magic = MAGIC_MSC;  break;
    default:                  magic = MAGIC_WCID; break;
  }
  RCC_APB3ENR |= RTCAPBEN_BIT;   /* enable TAMP/RTC clock */
  HAL_PWR_EnableBkUpAccess();
  TAMP_BKP0R = magic;
  __DSB();
  HAL_PWR_DisableBkUpAccess();
}
