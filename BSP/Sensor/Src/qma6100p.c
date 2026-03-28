/**
  ******************************************************************************
  * @file    qma6100p.c
  * @brief   QMA6100P accelerometer driver (SPI2 + mutex protected)
  *
  *   Bus  : SPI2  SCK=PB13  MISO=PB14  MOSI=PB15
  *   CS   : PB0 (GPIO, active low)
  ******************************************************************************
  */

#include "qma6100p.h"
#include <string.h>

#define QMA6100P_SPI_TIMEOUT_MS  SENSOR_SPI2_TIMEOUT_MS

typedef struct {
  uint8_t chip_id;
  int32_t lsb_1g;
  int16_t raw[3];
} QMA6100P_Drv_t;

static QMA6100P_Drv_t g_qma;

static HAL_StatusTypeDef QMA6100P_WriteReg(uint8_t reg, uint8_t val)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2];
  uint8_t rx_dummy[2] = {0};

  tx[0] = QMA6100P_SPI_MAKE_CMD(reg, 0U);
  tx[1] = val;

  QMA_SPI2_CS_LOW();
  ret = HAL_SPI_TransmitReceive(&hspi2, tx, rx_dummy, 2U, QMA6100P_SPI_TIMEOUT_MS);
  QMA_SPI2_CS_HIGH();

  return ret;
}

static HAL_StatusTypeDef QMA6100P_ReadReg(uint8_t reg, uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[7] = {0};
  uint8_t rx[7] = {0};

  if ((buf == NULL) || (len == 0U) || (len > 6U))
  {
    return HAL_ERROR;
  }

  tx[0] = QMA6100P_SPI_MAKE_CMD(reg, 1U);

  QMA_SPI2_CS_LOW();
  ret = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 1U), QMA6100P_SPI_TIMEOUT_MS);
  QMA_SPI2_CS_HIGH();

  if (ret == HAL_OK)
  {
    memcpy(buf, &rx[1], len);
  }

  return ret;
}

static void QMA6100P_DebugHardware(void)
{
  QMA_SPI2_CS_HIGH();
  HAL_Delay(1U);
}

static void QMA6100P_DumpKeyRegs(void)
{
  uint8_t reg_val = 0U;
  HAL_StatusTypeDef ret;
  uint8_t i;
  const struct {
    uint8_t reg;
    const char *name;
  } reg_map[] = {
    {0x00U, "CHIP_ID"},
    {0x0FU, "RANGE"},
    {0x10U, "BW_ODR"},
    {0x11U, "POWER_MANAGE"},
    {0x33U, "NVM"},
    {0x36U, "RESET"},
    {0x45U, "CHIP_STATE"},
    {0x4AU, "REG_4A"},
    {0x56U, "REG_56"},
    {0x5FU, "REG_5F"}
  };

  printf("[QMA6100P] === KEY REGS ===\r\n");
  for (i = 0U; i < (uint8_t)(sizeof(reg_map) / sizeof(reg_map[0])); i++)
  {
    ret = QMA6100P_ReadReg(reg_map[i].reg, &reg_val, 1U);
    if (ret == HAL_OK)
    {
      printf("[QMA6100P] %s(0x%02X) = 0x%02X\r\n",
             reg_map[i].name,
             reg_map[i].reg,
             reg_val);
    }
    else
    {
      printf("[QMA6100P] %s(0x%02X) = <read fail>\r\n",
             reg_map[i].name,
             reg_map[i].reg);
    }
  }
  printf("[QMA6100P] === KEY REGS END ===\r\n\r\n");
}

static HAL_StatusTypeDef QMA6100P_SetRange(QMA6100P_Range_t range)
{
  switch (range)
  {
    case QMA6100P_RANGE_4G:  g_qma.lsb_1g = 2048; break;
    case QMA6100P_RANGE_8G:  g_qma.lsb_1g = 1024; break;
    case QMA6100P_RANGE_16G: g_qma.lsb_1g = 512;  break;
    case QMA6100P_RANGE_32G: g_qma.lsb_1g = 256;  break;
    default:                 g_qma.lsb_1g = 4096; break;
  }

  return QMA6100P_WriteReg(QMA6100P_REG_RANGE, (uint8_t)range);
}

static HAL_StatusTypeDef QMA6100P_SetBW(QMA6100P_Bandwidth_t bw)
{
  return QMA6100P_WriteReg(QMA6100P_REG_BW_ODR, (uint8_t)bw);
}

static HAL_StatusTypeDef QMA6100P_SetActiveMode(void)
{
  /* 0x84 = active mode + MCLK 51.2kHz */
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
//      printf("[QMA6100P] POWER_MANAGE settle timeout: 0x%02X\r\n", reg_val);
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
//      printf("[QMA6100P] OTP load timeout.\r\n");
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
//      printf("[QMA6100P] CHIP_STATE timeout: 0x%02X\r\n", chip_state);
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

  QMA6100P_DebugHardware();

  ret = QMA6100P_Initialize();
  if (ret != HAL_OK)
  {
//    printf("[QMA6100P] Init sequence failed.\r\n");
    QMA6100P_DumpKeyRegs();
    QMA6100P_DumpRegs();
    return HAL_ERROR;
  }

  for (i = 0U; i < 3U; i++)
  {
    chip_id_raw = 0U;
    ret = QMA6100P_ReadReg(QMA6100P_REG_CHIP_ID, &chip_id_raw, 1U);

    if (ret == HAL_OK)
    {
      g_qma.chip_id = (uint8_t)(chip_id_raw >> 4);
      if (g_qma.chip_id == QMA6100P_DEVICE_ID)
      {
//        printf("[QMA6100P] CHIP_ID OK: raw=0x%02X, high=0x%X\r\n",
//               chip_id_raw, g_qma.chip_id);
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
    return HAL_OK;
  }

  QMA6100P_DumpKeyRegs();
  QMA6100P_DumpRegs();
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
      printf("[QMA6100P] Read raw XYZ failed.\r\n");
      return HAL_ERROR;
    }
    HAL_Delay(2U);
  }

  /* 14-bit signed, packed in 16-bit, lower 2 bits unused */
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

  if (g_qma.lsb_1g == 0)
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
