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

static HAL_StatusTypeDef H3LIS100DL_ReadReg(uint8_t reg, uint8_t *val)
{
  uint8_t tx[2] = { H3LIS100DL_SPI_MAKE_CMD(reg, 1U, 0U), 0x00U };
  uint8_t rx[2] = { 0 };
  HAL_StatusTypeDef ret;

  if (val == NULL) return HAL_ERROR;

  ret = H3LIS100DL_Transfer2(tx, rx);
  if (ret == HAL_OK) *val = rx[1];
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

#define H3LIS100DL_ReadRegs(reg, buf, len)  H3LIS100DL_ReadRegs_DMA(reg, buf, len)


int H3LIS100DL_Init(void)
{
  uint8_t who = 0U;

  memset(&g_h3lis, 0, sizeof(g_h3lis));
  H3_SPI2_CS_HIGH();
  HAL_Delay(10U);

  for (uint8_t retry = 0U; retry < 3U; retry++)
  {
    if ((H3LIS100DL_ReadReg(H3LIS100DL_REG_WHO_AM_I, &who) == HAL_OK) &&
        (who == H3LIS100DL_WHO_AM_I_VALUE))
    {
      break;
    }
    HAL_Delay(5U);
  }
  if (who != H3LIS100DL_WHO_AM_I_VALUE)
  {
    printf("[H3LIS100DL] WHO_AM_I=0x%02X expected=0x%02X\r\n", who, H3LIS100DL_WHO_AM_I_VALUE);
    return -1;
  }
  g_h3lis.who_am_i = who;

  if ((H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG2, 0x80U) != HAL_OK) ||  /* BOOT */
      (H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG1, 0x37U) != HAL_OK) ||  /* PM=001 normal, DR=10 400Hz, XYZ EN */
      (H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG4, 0x00U) != HAL_OK) ||  /* default range */
      (H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG3, H3LIS100DL_CTRL_REG3_DRDY_INT1) != HAL_OK))
  {
    printf("[H3LIS100DL] init write failed\r\n");
    return -1;
  }
  HAL_Delay(10U);

  printf("[H3LIS100DL] init ok (+/-100g 400Hz, DRDY->INT1/PB%u)\r\n",
         (unsigned)__builtin_ctz(H3LIS100DL_INT_PIN));
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
  if (data == NULL) return -1;

  /* Read OUT_X (0x29), OUT_Y (0x2B), OUT_Z (0x2D) individually. The
   * non-contiguous addressing makes a single MS-burst unreliable here:
   * when MS auto-increment crosses the reserved bytes (0x2A/0x2C) the chip
   * sometimes does not consider the data set "consumed" and leaves the DRDY
   * line latched high — three single-byte reads always clear it. */
  uint8_t x = 0, y = 0, z = 0;
  if ((H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_X, &x) != HAL_OK) ||
      (H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_Y, &y) != HAL_OK) ||
      (H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_Z, &z) != HAL_OK))
  {
    return -1;
  }

  data->raw[0] = (int8_t)x;
  data->raw[1] = (int8_t)y;
  data->raw[2] = (int8_t)z;

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
