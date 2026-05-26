#ifndef BOOT_MODE_H
#define BOOT_MODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  BOOT_MODE_DATA_LOG = 0,   /* normal RTOS path: sensors + SD-FatFs + USBX-CDC */
  BOOT_MODE_USB_MSC  = 1    /* bare-metal MSC path: SD exposed as USB drive */
} boot_mode_t;

boot_mode_t BootMode_Read(void);
void        BootMode_Write(boot_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
