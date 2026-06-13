/**
  ******************************************************************************
  * @file    qma6100p.c
  * @brief   QMA6100P accelerometer driver (SPI2)
  *
  *   Bus  : SPI2  SCK=PB10  MISO=PC2  MOSI=PC1
  *   CS   : PA4 (GPIO, active low)
  ******************************************************************************
  */

#include "qma6100p.h"
#include "bsp_spi.h"
#include "dma_sampling.h"
#include <string.h>

#define QMA6100P_SPI_TIMEOUT_MS SENSOR_SPI2_TIMEOUT_MS

typedef struct
{
  uint8_t chip_id;
  uint16_t lsb_1g;
  int16_t raw[3];
} QMA6100P_Drv_t;

static QMA6100P_Drv_t g_qma;

static HAL_StatusTypeDef QMA6100P_Transfer2(const uint8_t tx[2], uint8_t rx[2])
{
  HAL_StatusTypeDef ret;

  QMA_SPI2_CS_LOW();
  ret = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, 2U, QMA6100P_SPI_TIMEOUT_MS);
  QMA_SPI2_CS_HIGH();

  return ret;
}

static void QMA6100P_CsLow(void)
{
  QMA_SPI2_CS_LOW();
}

static void QMA6100P_CsHigh(void)
{
  QMA_SPI2_CS_HIGH();
}

static HAL_StatusTypeDef QMA6100P_WriteReg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2];

  tx[0] = QMA6100P_SPI_MAKE_CMD(reg, 0U);
  tx[1] = val;

  QMA_SPI2_CS_LOW();
  if (HAL_SPI_Transmit(&hspi2, tx, 2U, QMA6100P_SPI_TIMEOUT_MS) != HAL_OK)
  {
    QMA_SPI2_CS_HIGH();
    return HAL_ERROR;
  }
  QMA_SPI2_CS_HIGH();

  return HAL_OK;
}

static HAL_StatusTypeDef QMA6100P_ReadReg(uint8_t reg, uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef ret;
  uint8_t cmd;
  uint16_t i;

  if ((buf == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  cmd = QMA6100P_SPI_MAKE_CMD(reg, 1U);

  QMA_SPI2_CS_LOW();
  ret = HAL_SPI_Transmit(&hspi2, &cmd, 1U, QMA6100P_SPI_TIMEOUT_MS);
  if (ret == HAL_OK)
  {
    for (i = 0U; i < len; i++)
    {
      uint8_t tx_dummy = 0x00U;

      ret = HAL_SPI_TransmitReceive(&hspi2, &tx_dummy, &buf[i], 1U, QMA6100P_SPI_TIMEOUT_MS);
      if (ret != HAL_OK)
      {
        break;
      }
    }
  }
  QMA_SPI2_CS_HIGH();

  return ret;
}

static HAL_StatusTypeDef QMA6100P_ReadReg_DMA(uint8_t reg, uint8_t *buf, uint16_t len)
{
  static uint8_t tx_buf[8] __attribute__((aligned(32)));
  static uint8_t rx_buf[8] __attribute__((aligned(32)));
  HAL_StatusTypeDef ret;
  uint16_t i;

  if ((buf == NULL) || (len == 0U) || ((uint16_t)(len + 1U) > sizeof(tx_buf)))
  {
    return HAL_ERROR;
  }

  memset(tx_buf, 0, sizeof(tx_buf));
  memset(rx_buf, 0, sizeof(rx_buf));
  tx_buf[0] = QMA6100P_SPI_MAKE_CMD(reg, 1U);

  ret = Sensor_SPI2_TransmitReceive_DMA(SENSOR_SPI2_DMA_OWNER_QMA6100P,
                                        QMA6100P_CsLow,
                                        QMA6100P_CsHigh,
                                        tx_buf,
                                        rx_buf,
                                        (uint16_t)(len + 1U),
                                        QMA6100P_SPI_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  for (i = 0U; i < len; i++)
  {
    buf[i] = rx_buf[i + 1U];
  }

  return HAL_OK;
}


static HAL_StatusTypeDef QMA6100P_SetRange(QMA6100P_Range_t range)
{
  switch (range)
  {
    case QMA6100P_RANGE_4G:  g_qma.lsb_1g = 2048U; break;
    case QMA6100P_RANGE_8G:  g_qma.lsb_1g = 1024U; break;
    case QMA6100P_RANGE_16G: g_qma.lsb_1g = 512U;  break;
    case QMA6100P_RANGE_32G: g_qma.lsb_1g = 256U;  break;
    default:                 g_qma.lsb_1g = 4096U; break;
  }

  return QMA6100P_WriteReg(QMA6100P_REG_RANGE, (uint8_t)range);
}

static HAL_StatusTypeDef QMA6100P_SetBW(QMA6100P_Bandwidth_t bw)
{
  return QMA6100P_WriteReg(QMA6100P_REG_BW_ODR, (uint8_t)bw);
}

static HAL_StatusTypeDef QMA6100P_SetActiveMode(void)
{
  return QMA6100P_WriteReg(QMA6100P_REG_POWER_MANAGE, 0x84U);
}

static HAL_StatusTypeDef QMA6100P_SoftReset(void)
{
  uint8_t reg_val = 0U;
  uint8_t chip_state = 0U;
  uint8_t retry = 0U;

  if (QMA6100P_WriteReg(QMA6100P_REG_RESET, 0xB6U) != HAL_OK) return HAL_ERROR;
  HAL_Delay(1U);
  if (QMA6100P_WriteReg(QMA6100P_REG_RESET, 0x00U) != HAL_OK) return HAL_ERROR;
  HAL_Delay(100U);

  retry = 0U;
  reg_val = 0U;
  while (reg_val != 0x84U)
  {
    if (QMA6100P_WriteReg(QMA6100P_REG_POWER_MANAGE, 0x84U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(2U);
    if (QMA6100P_ReadReg(QMA6100P_REG_POWER_MANAGE, &reg_val, 1U) != HAL_OK) return HAL_ERROR;

    if (++retry > 10U)
    {
      return HAL_ERROR;
    }
  }

  if (QMA6100P_WriteReg(QMA6100P_REG_NVM, 0x08U) != HAL_OK) return HAL_ERROR;
  HAL_Delay(5U);

  reg_val = 0U;
  retry = 0U;
  while ((reg_val & 0x05U) != 0x05U)
  {
    if (QMA6100P_ReadReg(QMA6100P_REG_NVM, &reg_val, 1U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(5U);

    if (++retry >= 100U)
    {
      return HAL_ERROR;
    }
  }

  chip_state = 0U;
  retry = 0U;
  while ((chip_state & 0xF0U) != 0xC0U)
  {
    if (QMA6100P_ReadReg(0x45U, &chip_state, 1U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(2U);

    if (++retry >= 50U)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef QMA6100P_Initialize(void)
{
  if (QMA6100P_SoftReset() != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (QMA6100P_WriteReg(0x11U, 0x80U) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_WriteReg(0x11U, 0x84U) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_WriteReg(0x4AU, 0x20U) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_WriteReg(0x56U, 0x01U) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_WriteReg(0x5FU, 0x80U) != HAL_OK) return HAL_ERROR;
  HAL_Delay(2U);
  if (QMA6100P_WriteReg(0x5FU, 0x00U) != HAL_OK) return HAL_ERROR;
  HAL_Delay(10U);

  if (QMA6100P_SetRange(QMA6100P_RANGE_4G) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_SetBW(QMA6100P_BW_1600) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_SetActiveMode() != HAL_OK) return HAL_ERROR;

  return HAL_OK;
}

HAL_StatusTypeDef QMA6100P_Init(void)
{
  uint8_t chip_id_raw = 0U;
  uint8_t chip_state = 0U;
  uint8_t range = 0U;
  uint8_t bw_odr = 0U;
  uint8_t power_manage = 0U;

  memset(&g_qma, 0, sizeof(g_qma));
  QMA_SPI2_CS_HIGH();
  HAL_Delay(10U);

  if (QMA6100P_Initialize() != HAL_OK)
  {
    printf("[QMA6100P] init failed\r\n");
    return HAL_ERROR;
  }

  /* CHIP_ID is in the upper nibble of register 0x00 */
  for (uint8_t i = 0U; i < 3U; i++)
  {
    if ((QMA6100P_ReadReg(QMA6100P_REG_CHIP_ID, &chip_id_raw, 1U) == HAL_OK) &&
        ((uint8_t)(chip_id_raw >> 4) == QMA6100P_DEVICE_ID))
    {
      g_qma.chip_id = QMA6100P_DEVICE_ID;
      printf("[QMA6100P] init ok (+/-4g 1600Hz)\r\n");
      return HAL_OK;
    }
    HAL_Delay(1U);
  }

  /* CHIP_ID via SPI sometimes glitches; fall back to verifying configured
   * registers match what QMA6100P_Initialize wrote. */
  if ((QMA6100P_ReadReg(0x45U, &chip_state, 1U) == HAL_OK) &&
      (QMA6100P_ReadReg(QMA6100P_REG_RANGE, &range, 1U) == HAL_OK) &&
      (QMA6100P_ReadReg(QMA6100P_REG_BW_ODR, &bw_odr, 1U) == HAL_OK) &&
      (QMA6100P_ReadReg(QMA6100P_REG_POWER_MANAGE, &power_manage, 1U) == HAL_OK) &&
      ((chip_state & 0xF0U) == 0xC0U) &&
      (range == (uint8_t)QMA6100P_RANGE_4G) &&
      (bw_odr == (uint8_t)QMA6100P_BW_1600) &&
      (power_manage == 0x84U))
  {
    g_qma.chip_id = QMA6100P_DEVICE_ID;
    printf("[QMA6100P] init ok (+/-4g 1600Hz)\r\n");
    return HAL_OK;
  }

  printf("[QMA6100P] init failed (CHIP_ID=0x%02X)\r\n", chip_id_raw);
  return HAL_ERROR;
}

HAL_StatusTypeDef QMA6100P_Configure(const QMA6100P_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return HAL_ERROR;
  }

  /* Go to Standby before changing BW/RANGE — the datasheet (similar to
   * MMA8451 architecture) requires these registers to be written in Standby
   * mode.  0x80 = NVM load trigger + Standby. */
  if (QMA6100P_WriteReg(QMA6100P_REG_POWER_MANAGE, 0x80U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(2U);

  if (QMA6100P_SetRange(cfg->range) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (QMA6100P_SetBW(cfg->bw) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (QMA6100P_SetActiveMode() != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(2U);
  return HAL_OK;
}

HAL_StatusTypeDef QMA6100P_ReadRawXYZ(QMA6100P_Data_t *data)
{
  uint8_t buf[6] = {0};
  uint8_t retry = 0U;

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  while (QMA6100P_ReadReg_DMA(QMA6100P_REG_XOUTL, buf, 6U) != HAL_OK)
  {
    if (++retry > 1U)
    {
      return HAL_ERROR;
    }
    HAL_Delay(2U);
  }

  data->raw[0] = (int16_t)((((int16_t)buf[1]) << 8) | buf[0]) >> 2;
  data->raw[1] = (int16_t)((((int16_t)buf[3]) << 8) | buf[2]) >> 2;
  data->raw[2] = (int16_t)((((int16_t)buf[5]) << 8) | buf[4]) >> 2;

  g_qma.raw[0] = data->raw[0];
  g_qma.raw[1] = data->raw[1];
  g_qma.raw[2] = data->raw[2];

  return HAL_OK;
}

HAL_StatusTypeDef QMA6100P_ReadAccXYZ(QMA6100P_Data_t *data)
{
  if (data == NULL)
  {
    return HAL_ERROR;
  }

  if (QMA6100P_ReadRawXYZ(data) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (g_qma.lsb_1g == 0U)
  {
    return HAL_ERROR;
  }

  data->acc_mg[0] = (float)data->raw[0] * 1000.0f / (float)g_qma.lsb_1g;
  data->acc_mg[1] = (float)data->raw[1] * 1000.0f / (float)g_qma.lsb_1g;
  data->acc_mg[2] = (float)data->raw[2] * 1000.0f / (float)g_qma.lsb_1g;

  return HAL_OK;
}

HAL_StatusTypeDef QMA6100P_FIFO_Config(uint8_t wtm_samples)
{
  /* STREAM mode: FIFO is a circular buffer; the 2ms timeout poll self-heals any
   * lost watermark edge (avoids the Bypass→FIFO rearm race). NOTE: ~31% of
   * emitted rows are duplicates because FIFO_CNT over-reports the available
   * frames slightly, so each read grabs a few stale tail frames — this is in the
   * level read, not the FIFO mode (FIFO mode gave the identical 31%). The unique
   * data is the true 1600 Hz; the duplicate tail is filtered downstream. */
  if (wtm_samples == 0U || wtm_samples > 63U) wtm_samples = 16U;

  if (QMA6100P_WriteReg(QMA6100P_REG_FIFO_CFG, QMA6100P_FIFO_MODE_BYPASS | QMA6100P_FIFO_CH_XYZ) != HAL_OK) return HAL_ERROR;
  HAL_Delay(2);
  if (QMA6100P_WriteReg(QMA6100P_REG_FIFO_WM, wtm_samples) != HAL_OK) return HAL_ERROR;
  HAL_Delay(2);
  if (QMA6100P_WriteReg(QMA6100P_INT_MAP_REG, QMA6100P_INT_WM_BIT) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_WriteReg(QMA6100P_REG_INT_EN1, QMA6100P_INT_WM_BIT) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_WriteReg(QMA6100P_REG_INT_CFG, 0x01U) != HAL_OK) return HAL_ERROR;  /* LATCH_INT=1 */
  if (QMA6100P_WriteReg(QMA6100P_REG_FIFO_CFG, QMA6100P_FIFO_MODE_STREAM | QMA6100P_FIFO_CH_XYZ) != HAL_OK) return HAL_ERROR;

  return HAL_OK;
}

HAL_StatusTypeDef QMA6100P_FIFO_GetLevel(uint8_t *level)
{
  if (level == NULL) return HAL_ERROR;
  return QMA6100P_ReadReg(QMA6100P_REG_FIFO_CNT, level, 1U);
}

HAL_StatusTypeDef QMA6100P_FIFO_ReadBlock(uint8_t *buf, uint8_t n_frames)
{
  /* MUST be a per-byte read. QMA6100P FIFO_DATA is a single streaming register
   * that advances one byte per SPI access; a multi-byte burst re-reads the same
   * frame (measured 61.6% duplicate frames, inflating the apparent ODR). The
   * generic ReadReg does the correct 1-byte-per-access stream. (LSM's 7-byte
   * block read works because its FIFO is a 7-register auto-increment window —
   * different mechanism, do NOT copy it here.) */
  if (buf == NULL || n_frames == 0U) return HAL_ERROR;
  return QMA6100P_ReadReg(QMA6100P_REG_FIFO_DATA, buf, (uint16_t)(n_frames * 6U));
}
