#include "ux_device_cdc_acm.h"

#include "ux_device_descriptors.h"

#include <string.h>

static UX_SLAVE_CLASS_CDC_ACM *g_cdc_acm;
static uint8_t g_last_line_state;

void UxDeviceCdcAcm_Activate(void *cdc_acm_instance)
{
  g_cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;
}

void UxDeviceCdcAcm_Deactivate(void *cdc_acm_instance)
{
  (void)cdc_acm_instance;
  g_cdc_acm = UX_NULL;
  g_last_line_state = 0U;
}

void UxDeviceCdcAcm_ParameterChange(void *cdc_acm_instance)
{
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;

  if (cdc_acm == UX_NULL)
  {
    return;
  }

  g_last_line_state = (uint8_t)((cdc_acm->ux_slave_class_cdc_acm_data_dtr_state != 0U) ||
                                 (cdc_acm->ux_slave_class_cdc_acm_data_rts_state != 0U));
}

UX_SLAVE_CLASS_CDC_ACM *UxDeviceCdcAcm_GetInstance(void)
{
  return g_cdc_acm;
}

uint8_t UxDeviceCdcAcm_IsReady(void)
{
  if ((_ux_system_slave == UX_NULL) || (g_cdc_acm == UX_NULL))
  {
    return 0U;
  }

  if (_ux_system_slave->ux_system_slave_device.ux_slave_device_state != UX_DEVICE_CONFIGURED)
  {
    return 0U;
  }

  return g_last_line_state;
}

uint32_t UxDeviceCdcAcm_Write(const uint8_t *buf, uint32_t len)
{
  ULONG actual_length = 0U;
  UINT status;
  uint32_t retries = 0U;

  if ((buf == NULL) || (len == 0U) || (g_cdc_acm == UX_NULL))
  {
    return 0U;
  }

  /* Drive the non-blocking write state machine to completion.
   * First call transitions from UX_STATE_RESET to WRITE_WAIT (submits transfer).
   * Subsequent calls poll until UX_STATE_NEXT (transfer complete). */
  do
  {
    actual_length = 0U;
    status = ux_device_class_cdc_acm_write_run(g_cdc_acm, (UCHAR *)buf, (ULONG)len, &actual_length);

    if ((status == UX_STATE_NEXT) || (status == UX_SUCCESS))
    {
      return (uint32_t)actual_length;
    }

    if (status == UX_STATE_WAIT)
    {
      /* Transfer submitted but not yet completed — run the USBX task loop
       * to push data through the HAL PCD, then retry. */
      ux_system_tasks_run();
    }
    else
    {
      /* UX_STATE_ERROR, UX_STATE_EXIT, etc. — give up. */
      break;
    }
  } while (++retries < 100U);

  return 0U;
}

uint32_t UxDeviceCdcAcm_Read(uint8_t *buf, uint32_t len)
{
  ULONG actual_length = 0U;
  UINT status;

  if ((buf == NULL) || (len == 0U) || (g_cdc_acm == UX_NULL))
  {
    return 0U;
  }

  status = ux_device_class_cdc_acm_read_run(g_cdc_acm, (UCHAR *)buf, (ULONG)len, &actual_length);
  if ((status == UX_STATE_NEXT) || (status == UX_SUCCESS))
  {
    return (uint32_t)actual_length;
  }

  return 0U;
}
