#ifndef USB_MSC_APP_H
#define USB_MSC_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t UsbMsc_App_Start(void);
uint8_t UsbMsc_App_IsConfigured(void);
void    UsbMsc_App_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
