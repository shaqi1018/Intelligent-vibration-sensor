#ifndef BOOT_MODE_H
#define BOOT_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  BOOT_MODE_DATA_LOG  = 0,  /* RTOS: sensors + SD-FatFs + WCID Bulk USB (default) */
  BOOT_MODE_USB_MSC   = 1,  /* bare-metal: SD exposed as USB drive */
  BOOT_MODE_WCID_BULK = 2   /* same as DATA_LOG, explicit WCID */
} boot_mode_t;

extern boot_mode_t g_boot_mode;

boot_mode_t BootMode_Read(void);
void        BootMode_Write(boot_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
