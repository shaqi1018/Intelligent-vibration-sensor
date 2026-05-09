#include "usb_cdc_service.h"

#include "app_usbx_device.h"
#include "ux_device_cdc_acm.h"

uint8_t UsbCdcService_Init(void)
{
  return (App_USBX_Device_Init() == UX_SUCCESS) ? 1U : 0U;
}

void UsbCdcService_Poll(void)
{
  App_USBX_Device_Poll();
}

uint8_t UsbCdcService_IsConfigured(void)
{
  return App_USBX_Device_IsConfigured();
}

uint8_t UsbCdcService_IsReady(void)
{
  return UxDeviceCdcAcm_IsReady();
}

uint32_t UsbCdcService_Write(const uint8_t *buf, uint32_t len)
{
  return UxDeviceCdcAcm_Write(buf, len);
}

uint32_t UsbCdcService_Read(uint8_t *buf, uint32_t len)
{
  return UxDeviceCdcAcm_Read(buf, len);
}
