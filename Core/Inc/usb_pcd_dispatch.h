#ifndef USB_PCD_DISPATCH_H
#define USB_PCD_DISPATCH_H

#include "boot_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Captured at boot from flash. Read by HAL_PCD_*Callback (in usb_pcd_dispatch.c)
 * to route events to the right stack. Set ONCE in main() before MX_USB_OTG_FS_PCD_Init. */
extern boot_mode_t g_boot_mode;

#ifdef __cplusplus
}
#endif

#endif
