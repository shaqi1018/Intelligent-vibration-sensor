#ifndef APP_USBX_DEVICE_H
#define APP_USBX_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

#define USBX_DEVICE_MEMORY_STACK_SIZE      (24U * 1024U)

UINT App_USBX_Device_Init(void);
void App_USBX_Device_Poll(void);
uint8_t App_USBX_Device_IsConfigured(void);
UX_SLAVE_CLASS_CDC_ACM *App_USBX_Device_GetCdcAcm(void);

void USBD_CDC_ACM_Activate(void *cdc_acm_instance);
void USBD_CDC_ACM_Deactivate(void *cdc_acm_instance);
void USBD_CDC_ACM_ParameterChange(void *cdc_acm_instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_USBX_DEVICE_H */
