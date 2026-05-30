/**
  ******************************************************************************
  * @file    bsp_spi.c
  * @brief   SPI bus initialization for sensor drivers.
  *
 *   SPI1 (LSM6DSOX dedicated):
 *     PA5=SCK(AF5), PA6=MISO(AF5), PA7=MOSI(AF5), PC4=CS
  *
 *   SPI2 (H3LIS100DL + QMA6100P shared bus):
 *     PB10=SCK(AF5), PC2=MISO(AF5), PC1=MOSI(AF5)
 *     PC5=H3 CS, PA4=QMA CS
  *
  *   Both buses: Mode3 (CPOL=1, CPHA=1), 8-bit, MSB first
  ******************************************************************************
  */

#include "bsp_spi.h"
#include "dma_sampling.h"
#include "cmsis_os2.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

static void SensorSpi2_NoOp(void)
{
}

HAL_StatusTypeDef Sensor_SPI2_TransmitReceive_DMA(SensorSpi2DmaOwner_t owner,
                                                  void (*cs_low)(void),
                                                  void (*cs_high)(void),
                                                  uint8_t *tx,
                                                  uint8_t *rx,
                                                  uint16_t len,
                                                  uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;
  uint32_t timeout = timeout_ms;
  void (*assert_cs)(void) = (cs_low != NULL) ? cs_low : SensorSpi2_NoOp;
  void (*release_cs)(void) = (cs_high != NULL) ? cs_high : SensorSpi2_NoOp;

  (void)owner;

  if ((tx == NULL) || (rx == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  DmaSampling_ResetSpi2DebugState();

  assert_cs();
  status = HAL_SPI_TransmitReceive_DMA(&hspi2, tx, rx, len);
  if (status != HAL_OK)
  {
    release_cs();
    DmaSampling_RecordSpi2StartFail();
    return status;
  }

  while (timeout-- > 0U)
  {
    if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_READY)
    {
      break;
    }
    osDelay(1U);
  }

  release_cs();

  if (timeout == 0U)
  {
    DmaSampling_RecordSpi2Timeout();
    return HAL_TIMEOUT;
  }

  if (DmaSampling_IsSpi2TransferComplete() == 0U)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void MX_SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitStruct.Pin = LSM_SPI_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LSM_SPI_CS_GPIO_PORT, &GPIO_InitStruct);
  LSM_SPI_CS_HIGH();

  GPIO_InitStruct.Pin = LSM_SPI_SCK_PIN | LSM_SPI_MOSI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LSM_SPI_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(LSM_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);

  hspi1.Instance = LSM_SPI_INSTANCE;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();

  GPIO_InitStruct.Pin = H3_SPI2_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(H3_SPI2_CS_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = QMA_SPI2_CS_PIN;
  HAL_GPIO_Init(QMA_SPI2_CS_GPIO_PORT, &GPIO_InitStruct);

  H3_SPI2_CS_HIGH();
  QMA_SPI2_CS_HIGH();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = SENSOR_SPI2_SCK_AF;
  GPIO_InitStruct.Pin = SENSOR_SPI2_SCK_PIN;
  HAL_GPIO_Init(SENSOR_SPI2_SCK_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR_SPI2_MOSI_PIN;
  GPIO_InitStruct.Alternate = SENSOR_SPI2_MOSI_AF;
  HAL_GPIO_Init(SENSOR_SPI2_MOSI_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = SENSOR_SPI2_MISO_AF;
  GPIO_InitStruct.Pin = SENSOR_SPI2_MISO_PIN;
  HAL_GPIO_Init(SENSOR_SPI2_MISO_PORT, &GPIO_InitStruct);

  hspi2.Instance = SENSOR_SPI2_INSTANCE;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  /* 5MHz — both sensors support 10MHz+ */
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}
