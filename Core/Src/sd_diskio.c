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

/* 非阻塞查卡忙(SdFat isBusy() 的 U5 等价物,用于 logger 机会式写入门控)。
 * SDMMC_FLAG_BUSYD0 = SDMMC_D0 忙信号线电平的反值(硬件位),纯读寄存器、零命令
 * 开销、不阻塞。返回 1=卡忙(PROGRAMMING/GC 中,此刻发写会死等)、0=空闲(可写)。
 * 掉帧根因是每次写死等卡退出 PROGRAMMING;logger 写前先查此位,忙则跳过让数据继续
 * 攒环,只在卡空闲的间隙写 → 消除等卡浪费(见 docs/.../2026-07-08-dropframe-opportunistic-write.md)。
 * 注意:仅在 DPSM 空闲(无传输在途)时该位才反映"卡编程忙";传输途中另有含义,故本
 * 函数用于"下一次写发起前"的门控是准确的。 */
unsigned char SD_IsCardBusy(void)
{
  return (__HAL_SD_GET_FLAG(&hsd1, SDMMC_FLAG_BUSYD0) != 0U) ? 1U : 0U;
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

/* 【回退方案专用,当前未启用】现走 SDMMC 硬件流控(HWFC)+不关中断的单次写(见下方 else 分支)。
 * 若 U5 HWFC 实测有坏行需回退到"__disable_irq + 子块"方案,本宏是那时每子块最大块数。
 * H3LIS100DL 无 FIFO,关中断窗口 > 其采样周期(2.5ms@400Hz)就丢样本;切 ≤K 块、块间开中断+yield
 * 使窗口 < 周期。实测(满配含96kHz MIC):K=8→H3 93.5%/LSM 101%;K=4→H3 97.5% 但 LSM 溢出掉30%
 * (卡 PROGRAMMING 翻倍、吞吐撑不住)。故回退时以 8 为安全底线。 */
#define SD_WRITE_IRQOFF_MAX_BLOCKS  8U

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
      /* ★2026-07-12 拆分计时(SD_WR_SPLIT_PROBE):把"DMA传输(等信号量)"与"PROGRAMMING等待"
       * 分开计时,确认那280ms尖峰到底耗在哪——这是设计读写解耦方案的前提数据。默认关。 */
#ifndef SD_WR_SPLIT_PROBE
#define SD_WR_SPLIT_PROBE 0U
#endif
#if (SD_WR_SPLIT_PROBE != 0U)
      uint32_t probe_dma0 = HAL_GetTick();
#endif
      if (xSemaphoreTake(s_sdmmc_dma_sem, pdMS_TO_TICKS(5000)) == pdTRUE)
      {
        /* Only trust the transfer if it completed WITHOUT an error callback.
         * A TXUNDERR/DCRCFAIL under bus contention fires HAL_SD_ErrorCallback,
         * which also gives the semaphore — if we trusted that, a partially/
         * wrongly written sector would land on the card (the byte-level CSV
         * corruption). On error, fall through to Abort + DeInit + retry. */
        if (g_sd_dma_error == 0U)
        {
#if (SD_WR_SPLIT_PROBE != 0U)
          uint32_t probe_dma_ms = HAL_GetTick() - probe_dma0;   /* DMA传输段(到DATAEND) */
          uint32_t probe_prog0 = HAL_GetTick();
#endif
          /* Wait for card to exit PROGRAMMING state. The SDMMC transfer is
           * done (DATAEND fired) but the card may still be writing to flash. */
          uint32_t t0 = xTaskGetTickCount();
          HAL_SD_CardStateTypeDef cs;
          do {
            cs = HAL_SD_GetCardState(&hsd1);
            if (cs == HAL_SD_CARD_TRANSFER) {
#if (SD_WR_SPLIT_PROBE != 0U)
              uint32_t probe_prog_ms = HAL_GetTick() - probe_prog0;
              if ((probe_dma_ms + probe_prog_ms) > 30U)
                printf("[SDwr] dma=%lums prog=%lums cnt=%lu sec=%lu\r\n",
                       (unsigned long)probe_dma_ms, (unsigned long)probe_prog_ms,
                       (unsigned long)count, (unsigned long)sector);
#endif
              return RES_OK;
            }
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
      /* 轮询写 + SDMMC 硬件流控(HWFC_EN,见 sdmmc.c)。HWFC 会在发送 FIFO 将欠载时自动暂停
       * SDMMC_CK,硬件防 TXUNDERR —— 因此传输期间**不再 __disable_irq**。中断全开 → H3
       * (无 FIFO、DRDY 不锁存)每个边沿都能被及时读走 → SD 路径 H3 满采(此前关中断窗口内被
       * 覆盖丢 ~7-10%)。单次整块写:卡 PROGRAMMING 不增多,LSM 吞吐不受损。仍纯轮询、不碰
       * DMA,故当年 IDMA 总线冒险的字节损坏不会回来。
       * ⚠️ 回退路标:若实测出现坏行/间隙(U5 HWFC 有问题),恢复 __disable_irq + ≤K 块子传输
       *    (git 上一版),并把 sdmmc.c 的 HardwareFlowControl 改回 DISABLE。 */
      if (HAL_SD_WriteBlocks(&hsd1, buff, (uint32_t)sector, (uint32_t)count, HAL_MAX_DELAY) != HAL_OK)
      {
        SD_CleanupAfterOp();
        HAL_SD_DeInit(&hsd1);
        MX_SDMMC1_SD_Init();
        HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
        continue;
      }
      /* 等卡退出 PROGRAMMING(中断本就开着,yield 让 H3/LSM 等传感器任务运行)。 */
      {
        HAL_SD_CardStateTypeDef cs;
        do {
          cs = HAL_SD_GetCardState(&hsd1);
          if (cs == HAL_SD_CARD_TRANSFER) { break; }
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
