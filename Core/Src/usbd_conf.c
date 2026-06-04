/**
  * USBD Library low-level link (Classic stack -> HAL PCD).
  *
  * The PCD callbacks (HAL_PCD_*Callback) are defined in usb_pcd_dispatch.c
  * so they can route to either the USBX stack (CDC mode) or this Classic
  * stack (MSC mode) at runtime. This file only provides the USBD_LL_*
  * downcalls that the USBD core invokes.
  */

#include "usbd_core.h"
#include "usbd_def.h"
#include "usbd_conf.h"
#include "usbd_msc.h"
#include "usb_otg.h"
#include "boot_mode.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern boot_mode_t g_boot_mode;

/* Global USB device handle — used by both MSC and WCID Bulk modes. */
USBD_HandleTypeDef hUSB_Device;

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  /* hpcd is already initialized by MX_USB_OTG_FS_PCD_Init() in main.
   * Just bind it to the USBD device handle so the callbacks know where
   * to route incoming events. */
  hpcd_USB_OTG_FS.pData = pdev;
  pdev->pData = &hpcd_USB_OTG_FS;

  /* FIFO layout — varies by boot mode.
   * Total USB OTG FS FIFO = 320 words (1280 bytes). */
  if (g_boot_mode == BOOT_MODE_WCID_BULK)
  {
    /* WCID Bulk: 3 IN + 1 OUT endpoints.
     * RX = 64 words (256B) — matches DATALOG1; 32 words starves EP0 control.
     * TX0 = EP0, TX1-3 = EP1-3 IN (224B each = 56 words).
     * Total = 64+64+56*3 = 296 <= 320 words available. */
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x40U);    /* 64 words = 256 bytes */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0U, 0x40U); /* 64 words = 256 bytes EP0 */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1U, 0x38U); /* 56 words = 224 bytes EP1 IN */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2U, 0x38U); /* 56 words = 224 bytes EP2 IN */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 3U, 0x38U); /* 56 words = 224 bytes EP3 IN */
  }
  else
  {
    /* MSC or CDC: RX shared, TX0 = EP0, TX1 = bulk IN. */
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80U);    /* 128 words = 512 bytes */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0U, 0x40U); /* 64 words = 256 bytes EP0 */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1U, 0x80U); /* 128 words = 512 bytes EP1 */
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef st = HAL_PCD_DeInit(pdev->pData);
  return (st == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef st = HAL_PCD_Start(pdev->pData);
  return (st == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  HAL_StatusTypeDef st = HAL_PCD_Stop(pdev->pData);
  return (st == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps)
{
  HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
  return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  if ((ep_addr & 0x80U) == 0x80U)
  {
    return hpcd->IN_ep[ep_addr & 0x7FU].is_stall;
  }
  return hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                          uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_MSC_BOT_HandleTypeDef) / 4U) + 1U];
  (void)size;
  return mem;
}

void USBD_static_free(void *p)
{
  (void)p;
}
