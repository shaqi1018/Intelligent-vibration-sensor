/**
  * MSC application bootstrap. Creates USBD device, registers MSC class
  * with the SD-backed storage interface, and starts enumeration.
  * Called only from the MSC boot path (main.c when boot flag == USB_MSC).
  */

#include "usb_msc_app.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc.h"
#include "usbd_storage_if.h"

USBD_HandleTypeDef hUsbDeviceFS;

uint8_t UsbMsc_App_Start(void)
{
  if (USBD_Init(&hUsbDeviceFS, &MSC_Desc, 0U) != USBD_OK)
  {
    return 0U;
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_MSC) != USBD_OK)
  {
    return 0U;
  }
  if (USBD_MSC_RegisterStorage(&hUsbDeviceFS, &USBD_Storage_Interface_fops_FS) != USBD_OK)
  {
    return 0U;
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    return 0U;
  }
  return 1U;
}

uint8_t UsbMsc_App_IsConfigured(void)
{
  return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}

void UsbMsc_App_Stop(void)
{
  (void)USBD_Stop(&hUsbDeviceFS);
  (void)USBD_DeInit(&hUsbDeviceFS);
}
