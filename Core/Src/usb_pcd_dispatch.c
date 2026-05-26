/**
  * Single owner of all HAL_PCD_*Callback symbols. Routes events to either:
  *   - USBX (CDC mode)        -> UX_App_PCD_*Callback (renamed in USBX)
  *   - Classic USBD core (MSC) -> USBD_LL_*
  * Boot mode is captured early in main() and never changes during a session.
  */

#include "stm32u5xx_hal.h"
#include "boot_mode.h"
#include "usbd_core.h"

/* USBX renamed callbacks (Middlewares/ST/usbx/.../ux_dcd_stm32_callback.c) */
extern void UX_App_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
extern void UX_App_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
extern void UX_App_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
extern void UX_App_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
extern void UX_App_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
extern void UX_App_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
extern void UX_App_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
extern void UX_App_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
extern void UX_App_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);

/* Captured by main() before any USB activity. */
boot_mode_t g_boot_mode = BOOT_MODE_DATA_LOG;

#define IS_MSC()  (g_boot_mode == BOOT_MODE_USB_MSC)

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  if (IS_MSC()) { (void)USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup); }
  else          { UX_App_PCD_SetupStageCallback(hpcd); }
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  if (IS_MSC()) { (void)USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff); }
  else          { UX_App_PCD_DataInStageCallback(hpcd, epnum); }
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  if (IS_MSC()) { (void)USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff); }
  else          { UX_App_PCD_DataOutStageCallback(hpcd, epnum); }
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  if (IS_MSC()) { (void)USBD_LL_SOF(hpcd->pData); }
  else          { UX_App_PCD_SOFCallback(hpcd); }
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  if (IS_MSC())
  {
    (void)USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);
    (void)USBD_LL_Reset(hpcd->pData);
  }
  else
  {
    UX_App_PCD_ResetCallback(hpcd);
  }
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  if (IS_MSC()) { (void)USBD_LL_Suspend(hpcd->pData); }
  else          { UX_App_PCD_SuspendCallback(hpcd); }
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  if (IS_MSC()) { (void)USBD_LL_Resume(hpcd->pData); }
  else          { UX_App_PCD_ResumeCallback(hpcd); }
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  if (IS_MSC()) { (void)USBD_LL_DevConnected(hpcd->pData); }
  else          { UX_App_PCD_ConnectCallback(hpcd); }
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  if (IS_MSC()) { (void)USBD_LL_DevDisconnected(hpcd->pData); }
  else          { UX_App_PCD_DisconnectCallback(hpcd); }
}
