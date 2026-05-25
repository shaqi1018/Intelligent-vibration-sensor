/**
  * MSC SCSI -> SD bridge.
  * Block device backed by hsd1 (SDMMC1, 4-bit). Capacity is taken at runtime
  * from HAL's card info so the U-disk shows the real card size.
  */

#include "usbd_storage_if.h"
#include "sdmmc.h"

#define STORAGE_LUN_NBR                  1U

extern SD_HandleTypeDef hsd1;

__ALIGN_BEGIN static const int8_t STORAGE_Inquirydata_FS[] __ALIGN_END = {
  /* LUN 0 */
  0x00,                       /* Peripheral qualifier + type: direct-access */
  0x80,                       /* Removable */
  0x02,                       /* Version: SPC-2 */
  0x02,                       /* Response data format */
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00, 0x00, 0x00,
  'S','T','M','i','c','r','o',' ',           /* Vendor   (8) */
  'S','e','n','s','o','r',' ','L','o','g','g','e','r',' ',' ',' ',  /* Product (16) */
  '0','.','0','1'                            /* Revision (4) */
};

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS = {
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

static int8_t STORAGE_Init_FS(uint8_t lun)
{
  (void)lun;
  return USBD_OK;
}

static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  (void)lun;
  if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    return USBD_FAIL;
  }
  *block_num  = hsd1.SdCard.BlockNbr;
  *block_size = (uint16_t)hsd1.SdCard.BlockSize;
  return USBD_OK;
}

static int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  (void)lun;
  return (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) ? USBD_OK : USBD_FAIL;
}

static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  (void)lun;
  return USBD_OK;
}

static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  (void)lun;
  if (HAL_SD_ReadBlocks(&hsd1, buf, blk_addr, blk_len, 2000U) != HAL_OK)
  {
    return USBD_FAIL;
  }
  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) { }
  return USBD_OK;
}

static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  (void)lun;
  if (HAL_SD_WriteBlocks(&hsd1, buf, blk_addr, blk_len, 2000U) != HAL_OK)
  {
    return USBD_FAIL;
  }
  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) { }
  return USBD_OK;
}

static int8_t STORAGE_GetMaxLun_FS(void)
{
  return STORAGE_LUN_NBR - 1;
}
