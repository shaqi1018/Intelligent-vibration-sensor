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

static HAL_StatusTypeDef QMA6100P_ReadRegTrace(uint8_t reg,
                                               uint8_t *val,
                                               uint8_t tx[2],
                                               uint8_t rx[2])
{
  HAL_StatusTypeDef ret;

  if ((val == NULL) || (tx == NULL) || (rx == NULL))
  {
    return HAL_ERROR;
  }

  tx[0] = QMA6100P_SPI_MAKE_CMD(reg, 1U);
  tx[1] = 0x00U;
  rx[0] = 0x00U;
  rx[1] = 0x00U;

  ret = QMA6100P_Transfer2(tx, rx);
  if (ret == HAL_OK)
  {
    *val = rx[1];
  }

  return ret;
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

static const char *QMA6100P_DiagPullName(uint32_t pull)
{
  switch (pull)
  {
    case GPIO_PULLUP:   return "PULLUP";
    case GPIO_PULLDOWN: return "PULLDOWN";
    default:            return "NOPULL";
  }
}

static void QMA6100P_DiagShortDelay(volatile uint32_t cycles)
{
  while (cycles--)
  {
    __NOP();
  }
}

static void QMA6100P_DiagConfigureMisoPull(uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = SENSOR_SPI2_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = SENSOR_SPI2_MISO_AF;
  HAL_GPIO_Init(SENSOR_SPI2_MISO_PORT, &GPIO_InitStruct);
}

static HAL_StatusTypeDef QMA6100P_DiagReadChipIdRaw(uint8_t *rx0, uint8_t *rx1)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2] = {QMA6100P_SPI_MAKE_CMD(QMA6100P_REG_CHIP_ID, 1U), 0x00U};
  uint8_t rx[2] = {0};

  QMA_SPI2_CS_HIGH();
  QMA6100P_DiagShortDelay(200U);
  QMA_SPI2_CS_LOW();
  QMA6100P_DiagShortDelay(200U);
  ret = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, QMA6100P_SPI_TIMEOUT_MS);
  QMA_SPI2_CS_HIGH();
  QMA6100P_DiagShortDelay(200U);

  if (rx0 != NULL)
  {
    *rx0 = rx[0];
  }
  if (rx1 != NULL)
  {
    *rx1 = rx[1];
  }

  return ret;
}

static void QMA6100P_DiagPrintHwSpiSweep(void)
{
  const uint32_t pulls[] = {GPIO_PULLUP, GPIO_PULLDOWN, GPIO_NOPULL};
  uint32_t pull_idx;

  printf("[QMA6100P DIAG] HW-SPI CHIP_ID sweep\r\n");

  for (pull_idx = 0U; pull_idx < (uint32_t)(sizeof(pulls) / sizeof(pulls[0])); pull_idx++)
  {
    uint8_t attempt;

    QMA6100P_DiagConfigureMisoPull(pulls[pull_idx]);
    HAL_Delay(1U);

    for (attempt = 0U; attempt < 3U; attempt++)
    {
      uint8_t rx0 = 0U;
      uint8_t rx1 = 0U;
      HAL_StatusTypeDef ret = QMA6100P_DiagReadChipIdRaw(&rx0, &rx1);

      printf("[QMA6100P DIAG] HW pull=%s try=%u ret=%d TX=[80 00] RX=[%02X %02X]\r\n",
             QMA6100P_DiagPullName(pulls[pull_idx]),
             (unsigned int)(attempt + 1U),
             (int)ret,
             rx0,
             rx1);
      HAL_Delay(2U);
    }
  }
}

static HAL_StatusTypeDef QMA6100P_DiagSetHwSpiMode(uint32_t polarity, uint32_t phase)
{
  HAL_StatusTypeDef ret;

  ret = HAL_SPI_DeInit(&hspi2);
  if ((ret != HAL_OK) && (ret != HAL_ERROR))
  {
    return ret;
  }

  hspi2.Init.CLKPolarity = polarity;
  hspi2.Init.CLKPhase = phase;

  return HAL_SPI_Init(&hspi2);
}

static void QMA6100P_DiagPrintHwSpiModeSweep(void)
{
  static const struct
  {
    const char *name;
    uint32_t polarity;
    uint32_t phase;
  } mode_map[] = {
    {"MODE0", SPI_POLARITY_LOW,  SPI_PHASE_1EDGE},
    {"MODE1", SPI_POLARITY_LOW,  SPI_PHASE_2EDGE},
    {"MODE2", SPI_POLARITY_HIGH, SPI_PHASE_1EDGE},
    {"MODE3", SPI_POLARITY_HIGH, SPI_PHASE_2EDGE},
  };

  uint32_t mode_idx;

  printf("[QMA6100P DIAG] HW-SPI mode sweep\r\n");
  QMA6100P_DiagConfigureMisoPull(GPIO_PULLUP);

  for (mode_idx = 0U; mode_idx < (uint32_t)(sizeof(mode_map) / sizeof(mode_map[0])); mode_idx++)
  {
    uint8_t rx0 = 0U;
    uint8_t rx1 = 0U;
    HAL_StatusTypeDef ret;

    ret = QMA6100P_DiagSetHwSpiMode(mode_map[mode_idx].polarity, mode_map[mode_idx].phase);
    if (ret == HAL_OK)
    {
      HAL_Delay(1U);
      ret = QMA6100P_DiagReadChipIdRaw(&rx0, &rx1);
    }

    printf("[QMA6100P DIAG] HW mode=%s ret=%d TX=[80 00] RX=[%02X %02X]\r\n",
           mode_map[mode_idx].name,
           (int)ret,
           rx0,
           rx1);
    HAL_Delay(2U);
  }

  MX_SPI2_Init();
}

static void QMA6100P_DiagConfigureBitBangPins(uint32_t miso_pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_SPI_DeInit(&hspi2);
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = H3_SPI2_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(H3_SPI2_CS_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = QMA_SPI2_CS_PIN;
  HAL_GPIO_Init(QMA_SPI2_CS_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR_SPI2_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SENSOR_SPI2_SCK_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR_SPI2_MOSI_PIN;
  HAL_GPIO_Init(SENSOR_SPI2_MOSI_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SENSOR_SPI2_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = miso_pull;
  HAL_GPIO_Init(SENSOR_SPI2_MISO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(H3_SPI2_CS_GPIO_PORT, H3_SPI2_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(QMA_SPI2_CS_GPIO_PORT, QMA_SPI2_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR_SPI2_SCK_PORT, SENSOR_SPI2_SCK_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SENSOR_SPI2_MOSI_PORT, SENSOR_SPI2_MOSI_PIN, GPIO_PIN_RESET);
}

static uint8_t QMA6100P_DiagBitBangTransfer(uint8_t tx)
{
  uint8_t rx = 0U;
  uint8_t bit;

  for (bit = 0U; bit < 8U; bit++)
  {
    uint8_t mask = (uint8_t)(0x80U >> bit);

    HAL_GPIO_WritePin(SENSOR_SPI2_SCK_PORT, SENSOR_SPI2_SCK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENSOR_SPI2_MOSI_PORT,
                      SENSOR_SPI2_MOSI_PIN,
                      (tx & mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    QMA6100P_DiagShortDelay(2000U);

    HAL_GPIO_WritePin(SENSOR_SPI2_SCK_PORT, SENSOR_SPI2_SCK_PIN, GPIO_PIN_SET);
    QMA6100P_DiagShortDelay(2000U);

    if (HAL_GPIO_ReadPin(SENSOR_SPI2_MISO_PORT, SENSOR_SPI2_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= mask;
    }
  }

  return rx;
}

static void QMA6100P_DiagRunBitBangRead(uint32_t miso_pull)
{
  uint8_t rx0;
  uint8_t rx1;

  QMA6100P_DiagConfigureBitBangPins(miso_pull);
  QMA6100P_DiagShortDelay(4000U);

  HAL_GPIO_WritePin(QMA_SPI2_CS_GPIO_PORT, QMA_SPI2_CS_PIN, GPIO_PIN_RESET);
  QMA6100P_DiagShortDelay(4000U);
  rx0 = QMA6100P_DiagBitBangTransfer(QMA6100P_SPI_MAKE_CMD(QMA6100P_REG_CHIP_ID, 1U));
  rx1 = QMA6100P_DiagBitBangTransfer(0x00U);
  HAL_GPIO_WritePin(QMA_SPI2_CS_GPIO_PORT, QMA_SPI2_CS_PIN, GPIO_PIN_SET);
  QMA6100P_DiagShortDelay(4000U);

  printf("[QMA6100P DIAG] BITBANG pull=%s TX=[80 00] RX=[%02X %02X]\r\n",
         QMA6100P_DiagPullName(miso_pull),
         rx0,
         rx1);

  MX_SPI2_Init();
}

static void QMA6100P_PrintInitDiagnostics(void)
{
  static const struct
  {
    const char *name;
    uint8_t reg;
  } diag_regs[] = {
    {"CHIP_ID",      QMA6100P_REG_CHIP_ID},
    {"RANGE",        QMA6100P_REG_RANGE},
    {"BW_ODR",       QMA6100P_REG_BW_ODR},
    {"POWER_MANAGE", QMA6100P_REG_POWER_MANAGE},
    {"NVM",          QMA6100P_REG_NVM},
    {"CHIP_STATE",   0x45U},
  };

  uint32_t i;

  printf("\r\n[QMA6100P DIAG] ===== begin =====\r\n");
  printf("[QMA6100P DIAG] Pins: CS=PA4 SCK=PB10 MISO=PC2 MOSI=PC1\r\n");
  printf("[QMA6100P DIAG] CS idle level=%u\r\n",
         (unsigned int)HAL_GPIO_ReadPin(QMA_SPI2_CS_GPIO_PORT, QMA_SPI2_CS_PIN));

  for (i = 0U; i < (sizeof(diag_regs) / sizeof(diag_regs[0])); i++)
  {
    uint8_t value = 0x00U;
    uint8_t tx[2];
    uint8_t rx[2];
    HAL_StatusTypeDef ret = QMA6100P_ReadRegTrace(diag_regs[i].reg, &value, tx, rx);

    printf("[QMA6100P DIAG] %-12s reg=0x%02X ret=%d TX=[%02X %02X] RX=[%02X %02X]\r\n",
           diag_regs[i].name,
           diag_regs[i].reg,
           (int)ret,
           tx[0],
           tx[1],
           rx[0],
           rx[1]);
  }

  QMA6100P_DiagPrintHwSpiSweep();
  QMA6100P_DiagPrintHwSpiModeSweep();
  QMA6100P_DiagRunBitBangRead(GPIO_NOPULL);
  QMA6100P_DiagRunBitBangRead(GPIO_PULLDOWN);

  QMA6100P_DiagConfigureMisoPull(GPIO_PULLUP);
  printf("[QMA6100P DIAG] Hint: RX=[FF FF] usually means the slave is not driving MISO or CS is ineffective.\r\n");
  printf("[QMA6100P DIAG] Hint: pull-up->FF and pull-down->00 usually means MISO is floating.\r\n");
  printf("[QMA6100P DIAG] ===== end =====\r\n\r\n");
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
  if (QMA6100P_SetBW(QMA6100P_BW_100) != HAL_OK) return HAL_ERROR;
  if (QMA6100P_SetActiveMode() != HAL_OK) return HAL_ERROR;

  return HAL_OK;
}

HAL_StatusTypeDef QMA6100P_Init(void)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint8_t chip_id_raw = 0U;
  uint8_t chip_state = 0U;
  uint8_t range = 0U;
  uint8_t bw_odr = 0U;
  uint8_t power_manage = 0U;
  uint8_t i;

  memset(&g_qma, 0, sizeof(g_qma));
  QMA_SPI2_CS_HIGH();
  HAL_Delay(10U);

  ret = QMA6100P_Initialize();
  if (ret != HAL_OK)
  {
    printf("[QMA6100P] init failed\r\n");
    QMA6100P_PrintInitDiagnostics();
    return HAL_ERROR;
  }

  for (i = 0U; i < 3U; i++)
  {
    uint8_t tx[2];
    uint8_t rx[2];

    chip_id_raw = 0U;
    ret = QMA6100P_ReadRegTrace(QMA6100P_REG_CHIP_ID, &chip_id_raw, tx, rx);
    printf("[QMA6100P DIAG] CHIP_ID try=%u ret=%d TX=[%02X %02X] RX=[%02X %02X]\r\n",
           (unsigned int)(i + 1U),
           (int)ret,
           tx[0],
           tx[1],
           rx[0],
           rx[1]);

    chip_id_raw = 0U;
    ret = QMA6100P_ReadReg(QMA6100P_REG_CHIP_ID, &chip_id_raw, 1U);

    if (ret == HAL_OK)
    {
      g_qma.chip_id = (uint8_t)(chip_id_raw >> 4);
      if (g_qma.chip_id == QMA6100P_DEVICE_ID)
      {
        printf("[QMA6100P] init ok (+/-4g 100Hz)\r\n");
        return HAL_OK;
      }
    }

    HAL_Delay(1U);
  }

  if ((QMA6100P_ReadReg(0x45U, &chip_state, 1U) == HAL_OK) &&
      (QMA6100P_ReadReg(QMA6100P_REG_RANGE, &range, 1U) == HAL_OK) &&
      (QMA6100P_ReadReg(QMA6100P_REG_BW_ODR, &bw_odr, 1U) == HAL_OK) &&
      (QMA6100P_ReadReg(QMA6100P_REG_POWER_MANAGE, &power_manage, 1U) == HAL_OK) &&
      ((chip_state & 0xF0U) == 0xC0U) &&
      (range == (uint8_t)QMA6100P_RANGE_4G) &&
      (bw_odr == (uint8_t)QMA6100P_BW_100) &&
      (power_manage == 0x84U))
  {
    g_qma.chip_id = QMA6100P_DEVICE_ID;
    printf("[QMA6100P] init ok (+/-4g 100Hz)\r\n");
    return HAL_OK;
  }

  printf("[QMA6100P] init failed (CHIP_ID=0x%02X upper=0x%02X expected_upper=0x%02X)\r\n",
         chip_id_raw,
         (uint8_t)(chip_id_raw >> 4),
         QMA6100P_DEVICE_ID);
  QMA6100P_PrintInitDiagnostics();
  return HAL_ERROR;
}

HAL_StatusTypeDef QMA6100P_Configure(const QMA6100P_Config_t *cfg)
{
  if (cfg == NULL)
  {
    return HAL_ERROR;
  }

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

  while (QMA6100P_ReadReg(QMA6100P_REG_XOUTL, buf, 6U) != HAL_OK)
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

HAL_StatusTypeDef QMA6100P_ReadChipID(uint8_t *id)
{
  if (id == NULL)
  {
    return HAL_ERROR;
  }

  return QMA6100P_ReadReg(QMA6100P_REG_CHIP_ID, id, 1U);
}

HAL_StatusTypeDef QMA6100P_ReadStatus(uint8_t *status)
{
  if (status == NULL)
  {
    return HAL_ERROR;
  }

  return QMA6100P_ReadReg(QMA6100P_REG_INT_STATUS_2, status, 1U);
}

void QMA6100P_DumpRegs(void)
{
  uint8_t reg_val = 0U;
  uint8_t i;
  const uint8_t reg_map[] = {
    0x00U, 0x0FU, 0x10U, 0x11U, 0x17U, 0x18U, 0x1AU, 0x1CU,
    0x20U, 0x43U, 0x45U, 0x4AU, 0x50U, 0x56U, 0x57U
  };

  printf("[QMA6100P] Register dump:\r\n");
  for (i = 0U; i < (uint8_t)(sizeof(reg_map) / sizeof(reg_map[0])); i++)
  {
    if (QMA6100P_ReadReg(reg_map[i], &reg_val, 1U) == HAL_OK)
    {
      printf("  reg[0x%02X] = 0x%02X\r\n", reg_map[i], reg_val);
    }
    else
    {
      printf("  reg[0x%02X] = <read fail>\r\n", reg_map[i]);
    }
  }
  printf("\r\n");
}
