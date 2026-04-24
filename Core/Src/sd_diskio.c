#include "sd_diskio.h"

#include "sdmmc.h"

static DSTATUS g_sd_status = STA_NOINIT;

static DSTATUS SD_GetDriveStatus(BYTE pdrv)
{
  if (pdrv != SDDISKIO_DRIVE_NUM)
  {
    return STA_NOINIT;
  }

  if (SDMMC1_IsCardDetected() == 0U)
  {
    g_sd_status = (DSTATUS)(STA_NOINIT | STA_NODISK);
    return g_sd_status;
  }

  if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    g_sd_status = STA_NOINIT;
    return g_sd_status;
  }

  g_sd_status = 0;
  return g_sd_status;
}

DSTATUS SD_disk_status(BYTE pdrv)
{
  return SD_GetDriveStatus(pdrv);
}

DSTATUS SD_disk_initialize(BYTE pdrv)
{
  if (pdrv != SDDISKIO_DRIVE_NUM)
  {
    return STA_NOINIT;
  }

  return SD_GetDriveStatus(pdrv);
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if ((pdrv != SDDISKIO_DRIVE_NUM) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SD_GetDriveStatus(pdrv) & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  if (HAL_SD_ReadBlocks(&hsd1, buff, (uint32_t)sector, (uint32_t)count, HAL_MAX_DELAY) != HAL_OK)
  {
    return RES_ERROR;
  }

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
  }

  return RES_OK;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if ((pdrv != SDDISKIO_DRIVE_NUM) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SD_GetDriveStatus(pdrv) & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  if (HAL_SD_WriteBlocks(&hsd1, buff, (uint32_t)sector, (uint32_t)count, HAL_MAX_DELAY) != HAL_OK)
  {
    return RES_ERROR;
  }

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
  }

  return RES_OK;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  HAL_SD_CardInfoTypeDef card_info;

  if (pdrv != SDDISKIO_DRIVE_NUM)
  {
    return RES_PARERR;
  }

  if ((SD_GetDriveStatus(pdrv) & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  switch (cmd)
  {
    case CTRL_SYNC:
      return RES_OK;

    case GET_SECTOR_COUNT:
      if ((buff == NULL) || (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK))
      {
        return RES_ERROR;
      }
      *(DWORD *)buff = (DWORD)card_info.LogBlockNbr;
      return RES_OK;

    case GET_SECTOR_SIZE:
      if (buff == NULL)
      {
        return RES_PARERR;
      }
      *(WORD *)buff = (WORD)SDDISKIO_SECTOR_SIZE;
      return RES_OK;

    case GET_BLOCK_SIZE:
      if ((buff == NULL) || (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK))
      {
        return RES_ERROR;
      }
      *(DWORD *)buff = (card_info.LogBlockSize != 0U) ?
                       (DWORD)(card_info.LogBlockSize / SDDISKIO_SECTOR_SIZE) :
                       1U;
      if (*(DWORD *)buff == 0U)
      {
        *(DWORD *)buff = 1U;
      }
      return RES_OK;

    default:
      return RES_PARERR;
  }
}
