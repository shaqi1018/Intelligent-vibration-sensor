/**
  ******************************************************************************
  * @file    h3lis100dl.c
  * @brief   H3LIS100DL 100g accelerometer driver (SPI2)
  *
  *   Bus  : SPI2  SCK=PB10(AF5)  MISO=PC2(AF5)  MOSI=PC1(AF5)
  *   CS   : PC5 (GPIO, active low)
  *   Range: fixed +/-100g, sensitivity 780 mg/LSB (8-bit signed output)
  ******************************************************************************
  */

#include "h3lis100dl.h"
#include "bsp_spi.h"
#include "dma_sampling.h"
#include <string.h>

#define H3LIS100DL_SPI_TIMEOUT_MS  SENSOR_SPI2_TIMEOUT_MS

typedef struct
{
  uint8_t who_am_i;
} H3LIS100DL_Drv_t;

static H3LIS100DL_Drv_t g_h3lis;

static HAL_StatusTypeDef H3LIS100DL_Transfer2(const uint8_t tx[2], uint8_t rx[2])
{
  HAL_StatusTypeDef ret;

  H3_SPI2_CS_LOW();
  ret = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, 2U, H3LIS100DL_SPI_TIMEOUT_MS);
  H3_SPI2_CS_HIGH();

  return ret;
}

static void H3LIS100DL_CsLow(void)
{
  H3_SPI2_CS_LOW();
}

static void H3LIS100DL_CsHigh(void)
{
  H3_SPI2_CS_HIGH();
}

static HAL_StatusTypeDef H3LIS100DL_WriteReg(uint8_t reg, uint8_t val)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2];

  tx[0] = H3LIS100DL_SPI_MAKE_CMD(reg, 0U, 0U);
  tx[1] = val;

  H3_SPI2_CS_LOW();
  ret = HAL_SPI_Transmit(&hspi2, tx, 2U, H3LIS100DL_SPI_TIMEOUT_MS);
  H3_SPI2_CS_HIGH();

  return ret;
}

static HAL_StatusTypeDef H3LIS100DL_ReadRegTrace(uint8_t reg,
                                                 uint8_t *val,
                                                 uint8_t tx[2],
                                                 uint8_t rx[2])
{
  HAL_StatusTypeDef ret;

  if ((val == NULL) || (tx == NULL) || (rx == NULL))
  {
    return HAL_ERROR;
  }

  tx[0] = H3LIS100DL_SPI_MAKE_CMD(reg, 1U, 0U);
  tx[1] = 0x00U;
  rx[0] = 0x00U;
  rx[1] = 0x00U;

  ret = H3LIS100DL_Transfer2(tx, rx);
  if (ret == HAL_OK)
  {
    *val = rx[1];
  }

  return ret;
}

static HAL_StatusTypeDef H3LIS100DL_ReadRegs_DMA(uint8_t reg, uint8_t *buf, uint16_t len)
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
  tx_buf[0] = H3LIS100DL_SPI_MAKE_CMD(reg, 1U, (len > 1U) ? 1U : 0U);

  ret = Sensor_SPI2_TransmitReceive_DMA(SENSOR_SPI2_DMA_OWNER_H3LIS100DL,
                                        H3LIS100DL_CsLow,
                                        H3LIS100DL_CsHigh,
                                        tx_buf,
                                        rx_buf,
                                        (uint16_t)(len + 1U),
                                        H3LIS100DL_SPI_TIMEOUT_MS);
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

static HAL_StatusTypeDef H3LIS100DL_ReadReg(uint8_t reg, uint8_t *val)
{
  uint8_t tx[2];
  uint8_t rx[2];

  if (val == NULL)
  {
    return HAL_ERROR;
  }

  return H3LIS100DL_ReadRegTrace(reg, val, tx, rx);
}

static HAL_StatusTypeDef H3LIS100DL_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return H3LIS100DL_ReadRegs_DMA(reg, buf, len);
}

static const char *H3LIS100DL_DiagPullName(uint32_t pull)
{
  switch (pull)
  {
    case GPIO_PULLUP:   return "PULLUP";
    case GPIO_PULLDOWN: return "PULLDOWN";
    default:            return "NOPULL";
  }
}

static void H3LIS100DL_DiagShortDelay(volatile uint32_t cycles)
{
  while (cycles--)
  {
    __NOP();
  }
}

static void H3LIS100DL_DiagConfigureMisoPull(uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = SENSOR_SPI2_MISO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = SENSOR_SPI2_MISO_AF;
  HAL_GPIO_Init(SENSOR_SPI2_MISO_PORT, &GPIO_InitStruct);
}

static HAL_StatusTypeDef H3LIS100DL_DiagReadWhoAmIRaw(uint8_t *rx0, uint8_t *rx1)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2] = {H3LIS100DL_SPI_MAKE_CMD(H3LIS100DL_REG_WHO_AM_I, 1U, 0U), 0x00U};
  uint8_t rx[2] = {0};

  H3_SPI2_CS_HIGH();
  H3LIS100DL_DiagShortDelay(200U);
  H3_SPI2_CS_LOW();
  H3LIS100DL_DiagShortDelay(200U);
  ret = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, H3LIS100DL_SPI_TIMEOUT_MS);
  H3_SPI2_CS_HIGH();
  H3LIS100DL_DiagShortDelay(200U);

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

static void H3LIS100DL_DiagPrintHwSpiSweep(void)
{
  const uint32_t pulls[] = {GPIO_PULLUP, GPIO_PULLDOWN, GPIO_NOPULL};
  uint32_t pull_idx;

  printf("[H3LIS100DL DIAG] HW-SPI WHO_AM_I sweep\r\n");

  for (pull_idx = 0U; pull_idx < (uint32_t)(sizeof(pulls) / sizeof(pulls[0])); pull_idx++)
  {
    uint8_t attempt;

    H3LIS100DL_DiagConfigureMisoPull(pulls[pull_idx]);
    HAL_Delay(1U);

    for (attempt = 0U; attempt < 3U; attempt++)
    {
      uint8_t rx0 = 0U;
      uint8_t rx1 = 0U;
      HAL_StatusTypeDef ret = H3LIS100DL_DiagReadWhoAmIRaw(&rx0, &rx1);

      printf("[H3LIS100DL DIAG] HW pull=%s try=%u ret=%d TX=[8F 00] RX=[%02X %02X]\r\n",
             H3LIS100DL_DiagPullName(pulls[pull_idx]),
             (unsigned int)(attempt + 1U),
             (int)ret,
             rx0,
             rx1);
      HAL_Delay(2U);
    }
  }
}

static HAL_StatusTypeDef H3LIS100DL_DiagSetHwSpiMode(uint32_t polarity, uint32_t phase)
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

static void H3LIS100DL_DiagPrintHwSpiModeSweep(void)
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

  printf("[H3LIS100DL DIAG] HW-SPI mode sweep\r\n");
  H3LIS100DL_DiagConfigureMisoPull(GPIO_PULLUP);

  for (mode_idx = 0U; mode_idx < (uint32_t)(sizeof(mode_map) / sizeof(mode_map[0])); mode_idx++)
  {
    uint8_t rx0 = 0U;
    uint8_t rx1 = 0U;
    HAL_StatusTypeDef ret;

    ret = H3LIS100DL_DiagSetHwSpiMode(mode_map[mode_idx].polarity, mode_map[mode_idx].phase);
    if (ret == HAL_OK)
    {
      HAL_Delay(1U);
      ret = H3LIS100DL_DiagReadWhoAmIRaw(&rx0, &rx1);
    }

    printf("[H3LIS100DL DIAG] HW mode=%s ret=%d TX=[8F 00] RX=[%02X %02X]\r\n",
           mode_map[mode_idx].name,
           (int)ret,
           rx0,
           rx1);
    HAL_Delay(2U);
  }

  MX_SPI2_Init();
}

static void H3LIS100DL_DiagConfigureBitBangPins(uint32_t miso_pull)
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

static uint8_t H3LIS100DL_DiagBitBangTransfer(uint8_t tx)
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
    H3LIS100DL_DiagShortDelay(2000U);

    HAL_GPIO_WritePin(SENSOR_SPI2_SCK_PORT, SENSOR_SPI2_SCK_PIN, GPIO_PIN_SET);
    H3LIS100DL_DiagShortDelay(2000U);

    if (HAL_GPIO_ReadPin(SENSOR_SPI2_MISO_PORT, SENSOR_SPI2_MISO_PIN) == GPIO_PIN_SET)
    {
      rx |= mask;
    }
  }

  return rx;
}

static void H3LIS100DL_DiagRunBitBangRead(uint32_t miso_pull)
{
  uint8_t rx0;
  uint8_t rx1;

  H3LIS100DL_DiagConfigureBitBangPins(miso_pull);
  H3LIS100DL_DiagShortDelay(4000U);

  HAL_GPIO_WritePin(H3_SPI2_CS_GPIO_PORT, H3_SPI2_CS_PIN, GPIO_PIN_RESET);
  H3LIS100DL_DiagShortDelay(4000U);
  rx0 = H3LIS100DL_DiagBitBangTransfer(H3LIS100DL_SPI_MAKE_CMD(H3LIS100DL_REG_WHO_AM_I, 1U, 0U));
  rx1 = H3LIS100DL_DiagBitBangTransfer(0x00U);
  HAL_GPIO_WritePin(H3_SPI2_CS_GPIO_PORT, H3_SPI2_CS_PIN, GPIO_PIN_SET);
  H3LIS100DL_DiagShortDelay(4000U);

  printf("[H3LIS100DL DIAG] BITBANG pull=%s TX=[8F 00] RX=[%02X %02X]\r\n",
         H3LIS100DL_DiagPullName(miso_pull),
         rx0,
         rx1);

  MX_SPI2_Init();
}

static void H3LIS100DL_PrintInitDiagnostics(void)
{
  static const struct
  {
    const char *name;
    uint8_t reg;
  } diag_regs[] = {
    {"WHO_AM_I",  H3LIS100DL_REG_WHO_AM_I},
    {"CTRL_REG1", H3LIS100DL_REG_CTRL_REG1},
    {"CTRL_REG2", H3LIS100DL_REG_CTRL_REG2},
    {"CTRL_REG4", H3LIS100DL_REG_CTRL_REG4},
    {"STATUS",    H3LIS100DL_REG_STATUS},
  };

  uint32_t i;

  printf("\r\n[H3LIS100DL DIAG] ===== begin =====\r\n");
  printf("[H3LIS100DL DIAG] Pins: CS=PC5 SCK=PB10 MISO=PC2 MOSI=PC1\r\n");
  printf("[H3LIS100DL DIAG] CS idle level=%u\r\n",
         (unsigned int)HAL_GPIO_ReadPin(H3_SPI2_CS_GPIO_PORT, H3_SPI2_CS_PIN));

  for (i = 0U; i < (sizeof(diag_regs) / sizeof(diag_regs[0])); i++)
  {
    uint8_t value = 0x00U;
    uint8_t tx[2];
    uint8_t rx[2];
    HAL_StatusTypeDef ret = H3LIS100DL_ReadRegTrace(diag_regs[i].reg, &value, tx, rx);

    printf("[H3LIS100DL DIAG] %-9s reg=0x%02X ret=%d TX=[%02X %02X] RX=[%02X %02X]\r\n",
           diag_regs[i].name,
           diag_regs[i].reg,
           ret,
           tx[0],
           tx[1],
           rx[0],
           rx[1]);
  }

  H3LIS100DL_DiagPrintHwSpiSweep();
  H3LIS100DL_DiagPrintHwSpiModeSweep();
  H3LIS100DL_DiagRunBitBangRead(GPIO_NOPULL);
  H3LIS100DL_DiagRunBitBangRead(GPIO_PULLDOWN);

  H3LIS100DL_DiagConfigureMisoPull(GPIO_PULLUP);
  printf("[H3LIS100DL DIAG] Hint: RX=[FF FF] usually means the slave is not driving MISO or CS is ineffective.\r\n");
  printf("[H3LIS100DL DIAG] Hint: pull-up->FF and pull-down->00 usually means MISO is floating.\r\n");
  printf("[H3LIS100DL DIAG] ===== end =====\r\n\r\n");
}

int H3LIS100DL_Init(void)
{
  HAL_StatusTypeDef ret;
  uint8_t who = 0U;
  uint8_t retry;

  memset(&g_h3lis, 0, sizeof(g_h3lis));

  H3_SPI2_CS_HIGH();
  HAL_Delay(10U);

  for (retry = 0U; retry < 3U; retry++)
  {
    uint8_t tx[2];
    uint8_t rx[2];

    who = 0U;
    ret = H3LIS100DL_ReadRegTrace(H3LIS100DL_REG_WHO_AM_I, &who, tx, rx);
    printf("[H3LIS100DL DIAG] WHO_AM_I try=%u ret=%d TX=[%02X %02X] RX=[%02X %02X]\r\n",
           (unsigned int)(retry + 1U),
           ret,
           tx[0],
           tx[1],
           rx[0],
           rx[1]);
    if ((ret == HAL_OK) && (who == H3LIS100DL_WHO_AM_I_VALUE))
    {
      break;
    }
    HAL_Delay(5U);
  }

  if (who != H3LIS100DL_WHO_AM_I_VALUE)
  {
    printf("[H3LIS100DL] init failed (WHO_AM_I=0x%02X expected=0x%02X)\r\n",
           who,
           H3LIS100DL_WHO_AM_I_VALUE);
    H3LIS100DL_PrintInitDiagnostics();
    return -1;
  }

  g_h3lis.who_am_i = who;

  ret = H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG2, 0x80U);
  if (ret != HAL_OK)
  {
    printf("[H3LIS100DL] init failed (BOOT)\r\n");
    return -1;
  }
  HAL_Delay(10U);

  ret = H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG1, 0x37U);
  if (ret != HAL_OK)
  {
    printf("[H3LIS100DL] init failed (CTRL_REG1)\r\n");
    return -1;
  }

  ret = H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG4, 0x00U);
  if (ret != HAL_OK)
  {
    printf("[H3LIS100DL] init failed (CTRL_REG4)\r\n");
    return -1;
  }

  {
    uint8_t v1 = 0U;

    H3LIS100DL_ReadReg(H3LIS100DL_REG_CTRL_REG1, &v1);
    if (v1 != 0x37U)
    {
      printf("[H3LIS100DL] init failed (CTRL_REG1 verify, read=0x%02X)\r\n", v1);
      H3LIS100DL_PrintInitDiagnostics();
      return -1;
    }
  }

  printf("[H3LIS100DL] init ok (+/-100g 400Hz)\r\n");
  return 0;
}

int H3LIS100DL_Configure(const H3LIS100DL_Config_t *config)
{
  uint8_t pm;
  uint8_t dr;
  uint8_t reg_val;

  if (config == NULL)
  {
    return -1;
  }

  switch (config->odr)
  {
    case H3LIS100DL_ODR_OFF:      pm = 0U; dr = 0U; break;
    case H3LIS100DL_ODR_50HZ:     pm = 1U; dr = 0U; break;
    case H3LIS100DL_ODR_100HZ:    pm = 1U; dr = 1U; break;
    case H3LIS100DL_ODR_400HZ:    pm = 1U; dr = 2U; break;
    case H3LIS100DL_ODR_LP_05HZ:  pm = 2U; dr = 0U; break;
    case H3LIS100DL_ODR_LP_1HZ:   pm = 3U; dr = 0U; break;
    case H3LIS100DL_ODR_LP_2HZ:   pm = 4U; dr = 0U; break;
    case H3LIS100DL_ODR_LP_5HZ:   pm = 5U; dr = 0U; break;
    case H3LIS100DL_ODR_LP_10HZ:  pm = 6U; dr = 0U; break;
    default:                      pm = 1U; dr = 1U; break;
  }

  reg_val = (uint8_t)((pm << 5) | (dr << 3) | H3LIS100DL_CR1_XYZ_EN);

  if (H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG1, reg_val) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

int H3LIS100DL_ReadAccXYZ(H3LIS100DL_Data_t *data)
{
  uint8_t status = 0U;
  uint8_t raw_xyz[3] = {0};

  if (data == NULL)
  {
    return -1;
  }

  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_STATUS, &status) != HAL_OK)
  {
    return -1;
  }
  if ((status & H3LIS100DL_STATUS_ZYXDA) == 0U)
  {
    return -2;
  }

  if (H3LIS100DL_ReadRegs(H3LIS100DL_REG_OUT_X, raw_xyz, 3U) != HAL_OK)
  {
    return -1;
  }

  data->raw[0] = (int8_t)raw_xyz[0];
  data->raw[1] = (int8_t)raw_xyz[1];
  data->raw[2] = (int8_t)raw_xyz[2];

  data->acc_mg[0] = (float)data->raw[0] * H3LIS100DL_SENSITIVITY_MG;
  data->acc_mg[1] = (float)data->raw[1] * H3LIS100DL_SENSITIVITY_MG;
  data->acc_mg[2] = (float)data->raw[2] * H3LIS100DL_SENSITIVITY_MG;

  return 0;
}

int H3LIS100DL_ReadStatus(void)
{
  uint8_t status = 0U;

  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_STATUS, &status) != HAL_OK)
  {
    return -1;
  }

  return (int)status;
}

int H3LIS100DL_ReadWhoAmI(void)
{
  uint8_t who = 0U;

  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_WHO_AM_I, &who) != HAL_OK)
  {
    return -1;
  }

  return (int)who;
}

void H3LIS100DL_DumpRegs(void)
{
  uint8_t val;
  uint8_t xyz[3] = {0};

  printf("\r\n===== H3LIS100DL Register Dump =====\r\n");

  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_WHO_AM_I, &val) == HAL_OK)
  {
    printf("WHO_AM_I  (0x0F) = 0x%02X\r\n", val);
  }
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_CTRL_REG1, &val) == HAL_OK)
  {
    printf("CTRL_REG1 (0x20) = 0x%02X\r\n", val);
  }
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_CTRL_REG2, &val) == HAL_OK)
  {
    printf("CTRL_REG2 (0x21) = 0x%02X\r\n", val);
  }
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_CTRL_REG4, &val) == HAL_OK)
  {
    printf("CTRL_REG4 (0x23) = 0x%02X\r\n", val);
  }
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_STATUS, &val) == HAL_OK)
  {
    printf("STATUS    (0x27) = 0x%02X (ZYXDA=%d, ZYXOR=%d)\r\n",
           val,
           (val >> 3) & 0x01,
           (val >> 7) & 0x01);
  }

  if (H3LIS100DL_ReadRegs(H3LIS100DL_REG_OUT_X, xyz, 1U) == HAL_OK)
  {
    printf("OUT_X     (0x29) = 0x%02X (%d)\r\n", xyz[0], (int8_t)xyz[0]);
  }
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_Y, &val) == HAL_OK)
  {
    printf("OUT_Y     (0x2B) = 0x%02X (%d)\r\n", val, (int8_t)val);
  }
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_Z, &val) == HAL_OK)
  {
    printf("OUT_Z     (0x2D) = 0x%02X (%d)\r\n", val, (int8_t)val);
  }

  printf("====================================\r\n\r\n");
}
