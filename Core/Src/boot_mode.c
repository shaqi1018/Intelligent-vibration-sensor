#include "boot_mode.h"
#include "stm32u5xx_hal.h"

/* STM32U575: 2 MB flash, 8 KB page, bank 2 ends at 0x08200000.
 * Use the very last page as boot-mode marker. */
#define BOOT_MODE_FLAG_ADDR     (0x081FE000UL)
#define BOOT_MODE_FLAG_PAGE     (127U)
#define BOOT_MODE_MAGIC_LOG     (0xA5A5A5A5A5A5A5A5ULL)
#define BOOT_MODE_MAGIC_MSC     (0x5A5A5A5A5A5A5A5AULL)

boot_mode_t BootMode_Read(void)
{
  uint64_t v = *(volatile uint64_t *)BOOT_MODE_FLAG_ADDR;
  return (v == BOOT_MODE_MAGIC_MSC) ? BOOT_MODE_USB_MSC : BOOT_MODE_DATA_LOG;
}

void BootMode_Write(boot_mode_t mode)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_err = 0U;
  uint64_t magic = (mode == BOOT_MODE_USB_MSC) ? BOOT_MODE_MAGIC_MSC : BOOT_MODE_MAGIC_LOG;
  /* Quad-word program needs 16 bytes; pad upper half with the magic too. */
  uint64_t qw[2] = { magic, magic };

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks     = FLASH_BANK_2;
  erase.Page      = BOOT_MODE_FLAG_PAGE;
  erase.NbPages   = 1U;
  (void)HAL_FLASHEx_Erase(&erase, &page_err);

  (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                          BOOT_MODE_FLAG_ADDR,
                          (uint32_t)(uintptr_t)qw);

  HAL_FLASH_Lock();
}
