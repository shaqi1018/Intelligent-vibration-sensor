/**
  ******************************************************************************
  * @file    dma_sampling.c
  * @brief   DMA sampling helper (SPI1 + SPI2)
  ******************************************************************************
  */

#include "dma_sampling.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/* Forward declarations */
void HAL_DMA_XferCpltCallback_RX(DMA_HandleTypeDef *hdma);
void HAL_DMA_XferCpltCallback_TX(DMA_HandleTypeDef *hdma);
void HAL_DMA_XferErrorCallback_RX(DMA_HandleTypeDef *hdma);
void HAL_DMA_XferErrorCallback_TX(DMA_HandleTypeDef *hdma);

DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

static volatile uint32_t transfer_count = 0;
static volatile uint32_t error_count = 0;
static volatile uint32_t start_fail_count = 0;
static volatile uint32_t timeout_count = 0;
static volatile uint8_t transfer_complete = 0;

static volatile uint32_t irq_ch0_count = 0;
static volatile uint32_t irq_ch1_count = 0;
static volatile uint32_t dma_rx_cplt_count = 0;
static volatile uint32_t dma_tx_cplt_count = 0;
static volatile uint32_t spi_irq_count = 0;
static volatile uint32_t spi_error_code = 0;
static volatile uint32_t dma_rx_error_count = 0;
static volatile uint32_t dma_tx_error_count = 0;

static volatile uint32_t spi2_transfer_count = 0;
static volatile uint32_t spi2_error_count = 0;
static volatile uint32_t spi2_start_fail_count = 0;
static volatile uint32_t spi2_timeout_count = 0;
static volatile uint8_t spi2_transfer_complete = 0;
static volatile uint32_t irq_ch2_count = 0;
static volatile uint32_t irq_ch3_count = 0;
static volatile uint32_t dma_spi2_rx_cplt_count = 0;
static volatile uint32_t dma_spi2_tx_cplt_count = 0;
static volatile uint32_t spi2_irq_count = 0;
static volatile uint32_t spi2_error_code = 0;
static volatile uint32_t dma_spi2_rx_error_count = 0;
static volatile uint32_t dma_spi2_tx_error_count = 0;

/**
  * @brief  Initialize DMA for SPI1
  */
void DmaSampling_InitSPI1(void)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* SPI1 RX DMA */
  hdma_spi1_rx.Instance = GPDMA1_Channel0;
  hdma_spi1_rx.Init.Request = GPDMA1_REQUEST_SPI1_RX;
  hdma_spi1_rx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.SrcInc = DMA_SINC_FIXED;
  hdma_spi1_rx.Init.DestInc = DMA_DINC_INCREMENTED;
  hdma_spi1_rx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi1_rx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi1_rx.Init.Priority = DMA_HIGH_PRIORITY;
  hdma_spi1_rx.Init.SrcBurstLength = 1;
  hdma_spi1_rx.Init.DestBurstLength = 1;
  hdma_spi1_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  hdma_spi1_rx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi1_rx.Init.Mode = DMA_NORMAL;

  HAL_DMA_Init(&hdma_spi1_rx);
  HAL_DMA_ConfigChannelAttributes(&hdma_spi1_rx, DMA_CHANNEL_NPRIV);
  hdma_spi1_rx.XferCpltCallback = HAL_DMA_XferCpltCallback_RX;
  hdma_spi1_rx.XferErrorCallback = HAL_DMA_XferErrorCallback_RX;

  /* SPI1 TX DMA */
  hdma_spi1_tx.Instance = GPDMA1_Channel1;
  hdma_spi1_tx.Init.Request = GPDMA1_REQUEST_SPI1_TX;
  hdma_spi1_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
  hdma_spi1_tx.Init.DestInc = DMA_DINC_FIXED;
  hdma_spi1_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi1_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi1_tx.Init.Priority = DMA_HIGH_PRIORITY;
  hdma_spi1_tx.Init.SrcBurstLength = 1;
  hdma_spi1_tx.Init.DestBurstLength = 1;
  hdma_spi1_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  hdma_spi1_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi1_tx.Init.Mode = DMA_NORMAL;

  HAL_DMA_Init(&hdma_spi1_tx);
  HAL_DMA_ConfigChannelAttributes(&hdma_spi1_tx, DMA_CHANNEL_NPRIV);
  hdma_spi1_tx.XferCpltCallback = HAL_DMA_XferCpltCallback_TX;
  hdma_spi1_tx.XferErrorCallback = HAL_DMA_XferErrorCallback_TX;

  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
}

/**
  * @brief  Initialize DMA for SPI2
  */
void DmaSampling_InitSPI2(void)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* SPI2 RX DMA */
  hdma_spi2_rx.Instance = GPDMA1_Channel2;
  hdma_spi2_rx.Init.Request = GPDMA1_REQUEST_SPI2_RX;
  hdma_spi2_rx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi2_rx.Init.SrcInc = DMA_SINC_FIXED;
  hdma_spi2_rx.Init.DestInc = DMA_DINC_INCREMENTED;
  hdma_spi2_rx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi2_rx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi2_rx.Init.Priority = DMA_HIGH_PRIORITY;
  hdma_spi2_rx.Init.SrcBurstLength = 1;
  hdma_spi2_rx.Init.DestBurstLength = 1;
  hdma_spi2_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  hdma_spi2_rx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi2_rx.Init.Mode = DMA_NORMAL;

  HAL_DMA_Init(&hdma_spi2_rx);
  HAL_DMA_ConfigChannelAttributes(&hdma_spi2_rx, DMA_CHANNEL_NPRIV);
  hdma_spi2_rx.XferCpltCallback = HAL_DMA_XferCpltCallback_RX;
  hdma_spi2_rx.XferErrorCallback = HAL_DMA_XferErrorCallback_RX;

  /* SPI2 TX DMA */
  hdma_spi2_tx.Instance = GPDMA1_Channel3;
  hdma_spi2_tx.Init.Request = GPDMA1_REQUEST_SPI2_TX;
  hdma_spi2_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi2_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
  hdma_spi2_tx.Init.DestInc = DMA_DINC_FIXED;
  hdma_spi2_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi2_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi2_tx.Init.Priority = DMA_HIGH_PRIORITY;
  hdma_spi2_tx.Init.SrcBurstLength = 1;
  hdma_spi2_tx.Init.DestBurstLength = 1;
  hdma_spi2_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  hdma_spi2_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi2_tx.Init.Mode = DMA_NORMAL;

  HAL_DMA_Init(&hdma_spi2_tx);
  HAL_DMA_ConfigChannelAttributes(&hdma_spi2_tx, DMA_CHANNEL_NPRIV);
  hdma_spi2_tx.XferCpltCallback = HAL_DMA_XferCpltCallback_TX;
  hdma_spi2_tx.XferErrorCallback = HAL_DMA_XferErrorCallback_TX;

  HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
}

uint8_t DmaSampling_IsTransferComplete(void) { return transfer_complete; }
uint32_t DmaSampling_GetTransferCount(void) { return transfer_count; }
uint32_t DmaSampling_GetErrorCount(void) { return error_count; }
uint32_t DmaSampling_GetStartFailCount(void) { return start_fail_count; }
uint32_t DmaSampling_GetTimeoutCount(void) { return timeout_count; }
uint32_t DmaSampling_GetIrqCh0Count(void) { return irq_ch0_count; }
uint32_t DmaSampling_GetIrqCh1Count(void) { return irq_ch1_count; }
uint32_t DmaSampling_GetDmaRxCpltCount(void) { return dma_rx_cplt_count; }
uint32_t DmaSampling_GetDmaTxCpltCount(void) { return dma_tx_cplt_count; }
uint32_t DmaSampling_GetSpiIrqCount(void) { return spi_irq_count; }
uint32_t DmaSampling_GetSpiErrorCode(void) { return spi_error_code; }
uint32_t DmaSampling_GetDmaRxErrorCount(void) { return dma_rx_error_count; }
uint32_t DmaSampling_GetDmaTxErrorCount(void) { return dma_tx_error_count; }

uint8_t DmaSampling_IsSpi2TransferComplete(void) { return spi2_transfer_complete; }
uint32_t DmaSampling_GetSpi2TransferCount(void) { return spi2_transfer_count; }
uint32_t DmaSampling_GetSpi2ErrorCount(void) { return spi2_error_count; }
uint32_t DmaSampling_GetSpi2StartFailCount(void) { return spi2_start_fail_count; }
uint32_t DmaSampling_GetSpi2TimeoutCount(void) { return spi2_timeout_count; }
uint32_t DmaSampling_GetSpi2IrqCount(void) { return spi2_irq_count; }
uint32_t DmaSampling_GetSpi2RxIrqCount(void) { return irq_ch2_count; }
uint32_t DmaSampling_GetSpi2TxIrqCount(void) { return irq_ch3_count; }
uint32_t DmaSampling_GetSpi2DmaRxCpltCount(void) { return dma_spi2_rx_cplt_count; }
uint32_t DmaSampling_GetSpi2DmaTxCpltCount(void) { return dma_spi2_tx_cplt_count; }
uint32_t DmaSampling_GetSpi2DmaRxErrorCount(void) { return dma_spi2_rx_error_count; }
uint32_t DmaSampling_GetSpi2DmaTxErrorCount(void) { return dma_spi2_tx_error_count; }
uint32_t DmaSampling_GetSpi2ErrorCode(void) { return spi2_error_code; }

void DmaSampling_NotifySpiIrq(void)
{
  spi_irq_count++;
}

void DmaSampling_NotifySpi2Irq(void)
{
  spi2_irq_count++;
}

void DmaSampling_GetChannelStatus(uint32_t *ch0_csr, uint32_t *ch1_csr)
{
  if (ch0_csr != NULL)
  {
    *ch0_csr = GPDMA1_Channel0->CSR;
  }
  if (ch1_csr != NULL)
  {
    *ch1_csr = GPDMA1_Channel1->CSR;
  }
}

void DmaSampling_GetSpi2ChannelStatus(uint32_t *rx_csr, uint32_t *tx_csr)
{
  if (rx_csr != NULL)
  {
    *rx_csr = GPDMA1_Channel2->CSR;
  }
  if (tx_csr != NULL)
  {
    *tx_csr = GPDMA1_Channel3->CSR;
  }
}

void DmaSampling_GetSpiRegs(uint32_t *sr, uint32_t *cr1, uint32_t *cfg1, uint32_t *ier)
{
  if (sr != NULL)
  {
    *sr = SPI1->SR;
  }
  if (cr1 != NULL)
  {
    *cr1 = SPI1->CR1;
  }
  if (cfg1 != NULL)
  {
    *cfg1 = SPI1->CFG1;
  }
  if (ier != NULL)
  {
    *ier = SPI1->IER;
  }
}

void DmaSampling_GetSpi2Regs(uint32_t *sr, uint32_t *cr1, uint32_t *cfg1, uint32_t *ier)
{
  if (sr != NULL)
  {
    *sr = SPI2->SR;
  }
  if (cr1 != NULL)
  {
    *cr1 = SPI2->CR1;
  }
  if (cfg1 != NULL)
  {
    *cfg1 = SPI2->CFG1;
  }
  if (ier != NULL)
  {
    *ier = SPI2->IER;
  }
}

void DmaSampling_ResetDebugState(void)
{
  transfer_complete = 0;
  error_count = 0;
  spi_error_code = 0;
}

void DmaSampling_ResetSpi2DebugState(void)
{
  spi2_transfer_complete = 0;
  spi2_error_count = 0;
  spi2_error_code = 0;
}

void DmaSampling_RecordStartFail(void)
{
  start_fail_count++;
}

void DmaSampling_RecordTimeout(void)
{
  timeout_count++;
}

void DmaSampling_RecordSpi2StartFail(void)
{
  spi2_start_fail_count++;
}

void DmaSampling_RecordSpi2Timeout(void)
{
  spi2_timeout_count++;
}

void HAL_DMA_XferCpltCallback_RX(DMA_HandleTypeDef *hdma)
{
  if (hdma == &hdma_spi1_rx)
  {
    dma_rx_cplt_count++;
  }
  else if (hdma == &hdma_spi2_rx)
  {
    dma_spi2_rx_cplt_count++;
  }
}

void HAL_DMA_XferCpltCallback_TX(DMA_HandleTypeDef *hdma)
{
  if (hdma == &hdma_spi1_tx)
  {
    dma_tx_cplt_count++;
  }
  else if (hdma == &hdma_spi2_tx)
  {
    dma_spi2_tx_cplt_count++;
  }
}

void HAL_DMA_XferErrorCallback_RX(DMA_HandleTypeDef *hdma)
{
  if (hdma == &hdma_spi1_rx)
  {
    dma_rx_error_count++;
  }
  else if (hdma == &hdma_spi2_rx)
  {
    dma_spi2_rx_error_count++;
  }
}

void HAL_DMA_XferErrorCallback_TX(DMA_HandleTypeDef *hdma)
{
  if (hdma == &hdma_spi1_tx)
  {
    dma_tx_error_count++;
  }
  else if (hdma == &hdma_spi2_tx)
  {
    dma_spi2_tx_error_count++;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    transfer_complete = 1;
    transfer_count++;
  }
  else if (hspi->Instance == SPI2)
  {
    spi2_transfer_complete = 1;
    spi2_transfer_count++;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    error_count++;
    spi_error_code = hspi->ErrorCode;
  }
  else if (hspi->Instance == SPI2)
  {
    spi2_error_count++;
    spi2_error_code = hspi->ErrorCode;
  }
}

void GPDMA1_Channel0_IRQHandler(void)
{
  irq_ch0_count++;
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void GPDMA1_Channel1_IRQHandler(void)
{
  irq_ch1_count++;
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void GPDMA1_Channel2_IRQHandler(void)
{
  irq_ch2_count++;
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
}

void GPDMA1_Channel3_IRQHandler(void)
{
  irq_ch3_count++;
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}
