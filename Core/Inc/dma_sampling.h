/**
  ******************************************************************************
  * @file    dma_sampling.h
  * @brief   DMA sampling helper (SPI1 + SPI2)
  ******************************************************************************
  */

#ifndef __DMA_SAMPLING_H
#define __DMA_SAMPLING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include <stdint.h>

/* Initialize DMA for SPI1 / SPI2 */
void DmaSampling_InitSPI1(void);
void DmaSampling_InitSPI2(void);

/* Get DMA handles (for linking to SPI) */
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

/* SPI1 transfer state */
uint8_t DmaSampling_IsTransferComplete(void);

/* SPI1 diagnostics */
uint32_t DmaSampling_GetTransferCount(void);
uint32_t DmaSampling_GetErrorCount(void);
uint32_t DmaSampling_GetStartFailCount(void);
uint32_t DmaSampling_GetTimeoutCount(void);
uint32_t DmaSampling_GetIrqCh0Count(void);
uint32_t DmaSampling_GetIrqCh1Count(void);
uint32_t DmaSampling_GetDmaRxCpltCount(void);
uint32_t DmaSampling_GetDmaTxCpltCount(void);
uint32_t DmaSampling_GetSpiErrorCode(void);
uint32_t DmaSampling_GetDmaRxErrorCount(void);
uint32_t DmaSampling_GetDmaTxErrorCount(void);
void DmaSampling_GetChannelStatus(uint32_t *ch0_csr, uint32_t *ch1_csr);
void DmaSampling_GetSpiRegs(uint32_t *sr, uint32_t *cr1, uint32_t *cfg1, uint32_t *ier);
uint32_t DmaSampling_GetSpiIrqCount(void);
void DmaSampling_NotifySpiIrq(void);
void DmaSampling_ResetDebugState(void);
void DmaSampling_RecordStartFail(void);
void DmaSampling_RecordTimeout(void);

/* SPI2 diagnostics */
void DmaSampling_ResetSpi2DebugState(void);
void DmaSampling_RecordSpi2StartFail(void);
void DmaSampling_RecordSpi2Timeout(void);
void DmaSampling_NotifySpi2Irq(void);
uint8_t DmaSampling_IsSpi2TransferComplete(void);
uint32_t DmaSampling_GetSpi2TransferCount(void);
uint32_t DmaSampling_GetSpi2ErrorCount(void);
uint32_t DmaSampling_GetSpi2StartFailCount(void);
uint32_t DmaSampling_GetSpi2TimeoutCount(void);
uint32_t DmaSampling_GetSpi2IrqCount(void);
uint32_t DmaSampling_GetSpi2RxIrqCount(void);
uint32_t DmaSampling_GetSpi2TxIrqCount(void);
uint32_t DmaSampling_GetSpi2DmaRxCpltCount(void);
uint32_t DmaSampling_GetSpi2DmaTxCpltCount(void);
uint32_t DmaSampling_GetSpi2DmaRxErrorCount(void);
uint32_t DmaSampling_GetSpi2DmaTxErrorCount(void);
uint32_t DmaSampling_GetSpi2ErrorCode(void);
void DmaSampling_GetSpi2ChannelStatus(uint32_t *rx_csr, uint32_t *tx_csr);
void DmaSampling_GetSpi2Regs(uint32_t *sr, uint32_t *cr1, uint32_t *cfg1, uint32_t *ier);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_SAMPLING_H */
