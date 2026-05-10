#ifndef UX_DEVICE_CDC_ACM_H
#define UX_DEVICE_CDC_ACM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"

void UxDeviceCdcAcm_Activate(void *cdc_acm_instance);
void UxDeviceCdcAcm_Deactivate(void *cdc_acm_instance);
void UxDeviceCdcAcm_ParameterChange(void *cdc_acm_instance);

UX_SLAVE_CLASS_CDC_ACM *UxDeviceCdcAcm_GetInstance(void);
uint8_t UxDeviceCdcAcm_IsReady(void);
uint32_t UxDeviceCdcAcm_Write(const uint8_t *buf, uint32_t len);
uint32_t UxDeviceCdcAcm_Read(uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* UX_DEVICE_CDC_ACM_H */
