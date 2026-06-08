#include "sd_diskio.h"

#include "sdmmc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern SemaphoreHandle_t s_sdmmc_dma_sem;

static DSTATUS g_sd_status = STA_NOINIT;
static volatile uint8_t g_sd_use_dma = 0U;  /* 0=polling (default), 1=DMA */

void SD_SetDmaMode(unsigned char enable)
{
  g_sd_use_dma = enable;
}

/* 清理轮询操作后的残留状态，防止 IDMA/Context/Flags 污染后续 DMA 操作。 */
static void SD_CleanupAfterOp(void)
{
  hsd1.Context = SD_CONTEXT_NONE;
  hsd1.Instance->IDMACTRL = 0U;  /* SDMMC_DISABLE_IDMA */
  __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_FLAGS);
}

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

  if (SDMMC1_InitCard() == 0U)
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
  uint8_t retries;

  if ((pdrv != SDDISKIO_DRIVE_NUM) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SD_GetDriveStatus(pdrv) & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  for (retries = 0U; retries < 3U; retries++)
  {
    if (g_sd_use_dma != 0U)
    {
      /* DMA mode: clean stale state, initiate transfer, wait for semaphore. */
      SD_CleanupAfterOp();
      __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_FLAGS);
      if (HAL_SD_ReadBlocks_DMA(&hsd1, buff, (uint32_t)sector, (uint32_t)count) != HAL_OK)
      {
        HAL_SD_DeInit(&hsd1);
        MX_SDMMC1_SD_Init();
        HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
        continue;
      }
      if (xSemaphoreTake(s_sdmmc_dma_sem, pdMS_TO_TICKS(5000)) == pdTRUE)
      {
        /* Wait for card to exit PROGRAMMING/RECEIVING state. The SDMMC
         * transfer is done but the card may still be writing to flash. */
        uint32_t t0 = xTaskGetTickCount();
        HAL_SD_CardStateTypeDef cs;
        do {
          cs = HAL_SD_GetCardState(&hsd1);
          if (cs == HAL_SD_CARD_TRANSFER) { return RES_OK; }
          if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(500)) { break; }
          vTaskDelay(1U); /* yield while card programs — don't starve other tasks */
        } while (cs == HAL_SD_CARD_PROGRAMMING || cs == HAL_SD_CARD_RECEIVING);
      }
      /* Timeout or bad card state — abort and re-init. */
      HAL_SD_Abort(&hsd1);
      SD_CleanupAfterOp();
    }
    else
    {
      /* Polling mode (pre-kernel startup). */
      __disable_irq();
      if (HAL_SD_ReadBlocks(&hsd1, buff, (uint32_t)sector, (uint32_t)count, HAL_MAX_DELAY) == HAL_OK)
      {
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {}
        __enable_irq();
        SD_CleanupAfterOp();
        return RES_OK;
      }
      __enable_irq();
      SD_CleanupAfterOp();
    }
    HAL_SD_DeInit(&hsd1);
    MX_SDMMC1_SD_Init();
    /* HAL_SD_DeInit → MspDeInit disables NVIC; re-enable for next DMA op. */
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  }

  return RES_ERROR;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  uint8_t retries;

  if ((pdrv != SDDISKIO_DRIVE_NUM) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SD_GetDriveStatus(pdrv) & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  for (retries = 0U; retries < 3U; retries++)
  {
    if (g_sd_use_dma != 0U)
    {
      SD_CleanupAfterOp();
      __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_FLAGS);
      if (HAL_SD_WriteBlocks_DMA(&hsd1, buff, (uint32_t)sector, (uint32_t)count) != HAL_OK)
      {
        HAL_SD_DeInit(&hsd1);
        MX_SDMMC1_SD_Init();
        HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
        continue;
      }
      if (xSemaphoreTake(s_sdmmc_dma_sem, pdMS_TO_TICKS(5000)) == pdTRUE)
      {
        /* Wait for card to exit PROGRAMMING state. The SDMMC transfer is
         * done (DATAEND fired) but the card may still be writing to flash. */
        uint32_t t0 = xTaskGetTickCount();
        HAL_SD_CardStateTypeDef cs;
        do {
          cs = HAL_SD_GetCardState(&hsd1);
          if (cs == HAL_SD_CARD_TRANSFER) { return RES_OK; }
          if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(500)) { break; }
          vTaskDelay(1U); /* yield while card programs — don't starve other tasks */
        } while (cs == HAL_SD_CARD_PROGRAMMING);
      }
      HAL_SD_Abort(&hsd1);
      SD_CleanupAfterOp();
    }
    else
    {
      __disable_irq();
      if (HAL_SD_WriteBlocks(&hsd1, buff, (uint32_t)sector, (uint32_t)count, HAL_MAX_DELAY) != HAL_OK)
      {
        __enable_irq();
        SD_CleanupAfterOp();
        HAL_SD_DeInit(&hsd1);
        MX_SDMMC1_SD_Init();
        HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
        continue;
      }
      while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {}
      __enable_irq();
      SD_CleanupAfterOp();
      return RES_OK;
    }
    HAL_SD_DeInit(&hsd1);
    MX_SDMMC1_SD_Init();
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  }

  return RES_ERROR;
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

/* HAL DMA completion callbacks — called from SDMMC1 ISR. */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (s_sdmmc_dma_sem != NULL)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_sdmmc_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  if (s_sdmmc_dma_sem != NULL)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_sdmmc_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  if (s_sdmmc_dma_sem != NULL)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_sdmmc_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
