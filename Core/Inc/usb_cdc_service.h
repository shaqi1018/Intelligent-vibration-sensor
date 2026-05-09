#ifndef USB_CDC_SERVICE_H
#define USB_CDC_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t UsbCdcService_Init(void);
void UsbCdcService_Poll(void);
uint8_t UsbCdcService_IsConfigured(void);
uint8_t UsbCdcService_IsReady(void);
uint32_t UsbCdcService_Write(const uint8_t *buf, uint32_t len);
uint32_t UsbCdcService_Read(uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_SERVICE_H */
