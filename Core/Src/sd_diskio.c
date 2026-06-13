#include "sd_diskio.h"

#include "sdmmc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>

extern SemaphoreHandle_t s_sdmmc_dma_sem;

static DSTATUS g_sd_status = STA_NOINIT;
static volatile uint8_t g_sd_use_dma = 0U;  /* 0=polling (default), 1=DMA */

/* DMA write completion outcome flag. HAL_SD_TxCpltCallback and
 * HAL_SD_ErrorCallback BOTH release s_sdmmc_dma_sem, so after the semaphore is
 * taken the write path cannot otherwise tell a clean completion from an errored
 * one (e.g. a TXUNDERR FIFO underrun under bus contention). The error callback
 * sets this flag so the write path retries instead of trusting corrupt data. */
static volatile uint8_t g_sd_dma_error = 0U;

/* SDMMC write-path health counters (route-2 instrumentation). Read/printed via
 * SD_PrintWriteStats; reset per SD session via SD_ResetWriteStats. */
typedef struct {
  uint32_t writes;             /* SD_disk_write (DMA) calls */
  uint32_t dma_err_completions;/* completions signalled by HAL_SD_ErrorCallback */
  uint32_t last_error_code;    /* hsd1.ErrorCode captured at last DMA error */
  uint32_t retries;            /* write-loop retries (iterations past the first) */
  uint32_t sem_timeouts;       /* 5s DMA-complete semaphore timeouts */
  uint32_t prog_timeouts;      /* 500ms card-PROGRAMMING wait breaks */
  uint32_t deinits;            /* HAL_SD_DeInit + re-init recovery events */
  uint32_t write_failures;     /* SD_disk_write returning RES_ERROR */
} SdWriteStats_t;
static volatile SdWriteStats_t g_sd_wstats;

void SD_ResetWriteStats(void)
{
  g_sd_wstats.writes = 0U;
  g_sd_wstats.dma_err_completions = 0U;
  g_sd_wstats.last_error_code = 0U;
  g_sd_wstats.retries = 0U;
  g_sd_wstats.sem_timeouts = 0U;
  g_sd_wstats.prog_timeouts = 0U;
  g_sd_wstats.deinits = 0U;
  g_sd_wstats.write_failures = 0U;
}

void SD_PrintWriteStats(void)
{
  printf("[SDstat] writes=%lu dma_err=%lu(code=0x%lX) retry=%lu sem_to=%lu prog_to=%lu deinit=%lu fail=%lu\r\n",
         (unsigned long)g_sd_wstats.writes,
         (unsigned long)g_sd_wstats.dma_err_completions,
         (unsigned long)g_sd_wstats.last_error_code,
         (unsigned long)g_sd_wstats.retries,
         (unsigned long)g_sd_wstats.sem_timeouts,
         (unsigned long)g_sd_wstats.prog_timeouts,
         (unsigned long)g_sd_wstats.deinits,
         (unsigned long)g_sd_wstats.write_failures);
}

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

  g_sd_wstats.writes++;

  for (retries = 0U; retries < 3U; retries++)
  {
    if (retries > 0U) { g_sd_wstats.retries++; }
    if (g_sd_use_dma != 0U)
    {
      SD_CleanupAfterOp();
      __HAL_SD_CLEAR_FLAG(&hsd1, SDMMC_STATIC_FLAGS);
      g_sd_dma_error = 0U;   /* cleared before each transfer; set by error ISR */
      if (HAL_SD_WriteBlocks_DMA(&hsd1, buff, (uint32_t)sector, (uint32_t)count) != HAL_OK)
      {
        g_sd_wstats.deinits++;
        HAL_SD_DeInit(&hsd1);
        MX_SDMMC1_SD_Init();
        HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
        continue;
      }
      if (xSemaphoreTake(s_sdmmc_dma_sem, pdMS_TO_TICKS(5000)) == pdTRUE)
      {
        /* Only trust the transfer if it completed WITHOUT an error callback.
         * A TXUNDERR/DCRCFAIL under bus contention fires HAL_SD_ErrorCallback,
         * which also gives the semaphore — if we trusted that, a partially/
         * wrongly written sector would land on the card (the byte-level CSV
         * corruption). On error, fall through to Abort + DeInit + retry. */
        if (g_sd_dma_error == 0U)
        {
          /* Wait for card to exit PROGRAMMING state. The SDMMC transfer is
           * done (DATAEND fired) but the card may still be writing to flash. */
          uint32_t t0 = xTaskGetTickCount();
          HAL_SD_CardStateTypeDef cs;
          do {
            cs = HAL_SD_GetCardState(&hsd1);
            if (cs == HAL_SD_CARD_TRANSFER) { return RES_OK; }
            if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(500)) { g_sd_wstats.prog_timeouts++; break; }
            vTaskDelay(1U); /* yield while card programs — don't starve other tasks */
          } while (cs == HAL_SD_CARD_PROGRAMMING);
        }
      }
      else
      {
        g_sd_wstats.sem_timeouts++;
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
      /* The data transfer is done — re-enable IRQs BEFORE the card-PROGRAMMING
       * wait. Only the transfer itself needs IRQs masked (the CPU must feed the
       * SDMMC FIFO without an underrun); the flash-program wait does not. Keeping
       * it masked blocked the LSM FIFO-watermark ISR and the sensor tasks for the
       * whole program time → ~31% LSM undercapture at 6664 Hz. Yield while the
       * card programs so the sensor tasks run (mirrors the DMA path). */
      __enable_irq();
      {
        HAL_SD_CardStateTypeDef cs;
        do {
          cs = HAL_SD_GetCardState(&hsd1);
          if (cs == HAL_SD_CARD_TRANSFER) { break; }
          /* Yield only once the scheduler is running. This polling path also runs
           * pre-kernel (boot DeviceCfg / phase-B test) where vTaskDelay is illegal
           * — there, busy-spin (no tasks to yield to anyway). */
          if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) { vTaskDelay(1U); }
        } while (cs == HAL_SD_CARD_PROGRAMMING);
      }
      SD_CleanupAfterOp();
      return RES_OK;
    }
    g_sd_wstats.deinits++;
    HAL_SD_DeInit(&hsd1);
    MX_SDMMC1_SD_Init();
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  }

  g_sd_wstats.write_failures++;
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
  /* Mark the in-flight DMA transfer as failed so the write path retries rather
   * than trusting (possibly corrupt) data on the card. Capture the error code
   * for the route-2 diagnostics print. */
  g_sd_dma_error = 1U;
  g_sd_wstats.dma_err_completions++;
  if (hsd != NULL) { g_sd_wstats.last_error_code = hsd->ErrorCode; }
  if (s_sdmmc_dma_sem != NULL)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_sdmmc_dma_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
