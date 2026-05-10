#include "app_usbx_device.h"

#include "usb_otg.h"
#include "ux_device_descriptors.h"
#include "ux_device_cdc_acm.h"

#include "ux_dcd_stm32.h"

#include <string.h>

static UX_SLAVE_CLASS_CDC_ACM_PARAMETER g_cdc_acm_parameter;
static uint8_t g_usbx_initialized;
static uint8_t g_usb_started;
static ULONG g_usb_memory_pool[USBX_DEVICE_MEMORY_STACK_SIZE / sizeof(ULONG)];

UINT App_USBX_Device_Init(void)
{
  UCHAR *device_framework_high_speed;
  UCHAR *device_framework_full_speed;
  ULONG device_framework_hs_length;
  ULONG device_framework_fs_length;
  ULONG string_framework_length;
  ULONG language_id_framework_length;
  UCHAR *string_framework;
  UCHAR *language_id_framework;
  UINT status;

  if (g_usbx_initialized != 0U)
  {
    return UX_SUCCESS;
  }

  memset(&g_cdc_acm_parameter, 0, sizeof(g_cdc_acm_parameter));
  g_cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate = USBD_CDC_ACM_Activate;
  g_cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = USBD_CDC_ACM_Deactivate;
  g_cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change = USBD_CDC_ACM_ParameterChange;

  status = ux_system_initialize(g_usb_memory_pool, USBX_DEVICE_MEMORY_STACK_SIZE, UX_NULL, 0U);
  if (status != UX_SUCCESS)
  {
    return status;
  }

  device_framework_high_speed = USBD_Get_Device_Framework_Speed(USBD_HIGH_SPEED, &device_framework_hs_length);
  device_framework_full_speed = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED, &device_framework_fs_length);
  string_framework = USBD_Get_String_Framework(&string_framework_length);
  language_id_framework = USBD_Get_Language_Id_Framework(&language_id_framework_length);

  status = ux_device_stack_initialize(device_framework_high_speed,
                                      device_framework_hs_length,
                                      device_framework_full_speed,
                                      device_framework_fs_length,
                                      string_framework,
                                      string_framework_length,
                                      language_id_framework,
                                      language_id_framework_length,
                                      UX_NULL);
  if (status != UX_SUCCESS)
  {
    return status;
  }

  status = ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                          ux_device_class_cdc_acm_entry,
                                          USBD_Get_Configuration_Number(CLASS_TYPE_CDC_ACM, 0U),
                                          USBD_Get_Interface_Number(CLASS_TYPE_CDC_ACM, 0U),
                                          &g_cdc_acm_parameter);
  if (status != UX_SUCCESS)
  {
    return status;
  }

  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80U);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0U, USBD_MAX_EP0_SIZE / 4U);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1U, USBD_CDCACM_EPIN_FS_MPS / 4U);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2U, USBD_CDCACM_EPINCMD_FS_MPS / 4U);

  status = _ux_dcd_stm32_initialize((ULONG)USB_OTG_FS, (ULONG)&hpcd_USB_OTG_FS);
  if (status != UX_SUCCESS)
  {
    return status;
  }

  if (HAL_PCD_Start(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    return UX_ERROR;
  }

  g_usb_started = 1U;
  g_usbx_initialized = 1U;
  return UX_SUCCESS;
}

void App_USBX_Device_Poll(void)
{
  if (g_usbx_initialized == 0U)
  {
    return;
  }

  ux_system_tasks_run();
}

uint8_t App_USBX_Device_IsConfigured(void)
{
  if ((_ux_system_slave == UX_NULL) || (g_usbx_initialized == 0U))
  {
    return 0U;
  }

  return (_ux_system_slave->ux_system_slave_device.ux_slave_device_state == UX_DEVICE_CONFIGURED) ? 1U : 0U;
}

UX_SLAVE_CLASS_CDC_ACM *App_USBX_Device_GetCdcAcm(void)
{
  return UxDeviceCdcAcm_GetInstance();
}

void USBD_CDC_ACM_Activate(void *cdc_acm_instance)
{
  UxDeviceCdcAcm_Activate(cdc_acm_instance);
}

void USBD_CDC_ACM_Deactivate(void *cdc_acm_instance)
{
  UxDeviceCdcAcm_Deactivate(cdc_acm_instance);
}

void USBD_CDC_ACM_ParameterChange(void *cdc_acm_instance)
{
  UxDeviceCdcAcm_ParameterChange(cdc_acm_instance);
}
