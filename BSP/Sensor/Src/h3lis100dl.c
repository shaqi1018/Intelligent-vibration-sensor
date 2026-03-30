/**
  ******************************************************************************
  * @file    h3lis100dl.c
  * @brief   H3LIS100DL 100g accelerometer driver (SPI2)
  *
  *   Bus  : SPI2  SCK=PB13(AF5)  MISO=PB14(AF5)  MOSI=PB15(AF5)
  *   CS   : PB12 (GPIO, active low)
  *   Range: fixed 卤100g, sensitivity 780 mg/LSB (8-bit signed output)
  ******************************************************************************
  */

#include "h3lis100dl.h"
#include <string.h>

/* 锟斤拷锟斤拷 SPI 锟斤拷锟斤拷时时锟戒（锟斤拷锟诫） */
#define H3LIS100DL_SPI_TIMEOUT_MS  SENSOR_SPI2_TIMEOUT_MS

typedef struct {
  uint8_t who_am_i;
} H3LIS100DL_Drv_t;

static H3LIS100DL_Drv_t g_h3lis;

/**
  * @brief  通锟斤拷 SPI2 写锟诫单锟斤拷锟侥达拷锟斤拷锟斤拷
  * @param  reg 锟侥达拷锟斤拷锟斤拷址锟斤拷0x00~0x3F锟斤拷
  * @param  val 锟斤拷要写锟斤拷锟斤拷锟街�
  */
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

/**
  * @brief  通锟斤拷 SPI2 锟斤拷取锟斤拷锟斤拷锟侥达拷锟斤拷锟斤拷
  * @param  reg 锟侥达拷锟斤拷锟斤拷址锟斤拷0x00~0x3F锟斤拷
  * @param  val 锟斤拷锟斤拷值锟斤拷锟街革拷锟�
  */
static HAL_StatusTypeDef H3LIS100DL_ReadReg(uint8_t reg, uint8_t *val)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2] = {0};
  uint8_t rx[2] = {0};

  if (val == NULL)
  {
    return HAL_ERROR;
  }

  tx[0] = H3LIS100DL_SPI_MAKE_CMD(reg, 1U, 0U);

  H3_SPI2_CS_LOW();
  ret = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, H3LIS100DL_SPI_TIMEOUT_MS);
  H3_SPI2_CS_HIGH();

  if (ret == HAL_OK)
  {
    *val = rx[1];
  }

  return ret;
}

/**
  * @brief  通锟斤拷 SPI2 锟斤拷锟斤拷锟斤拷取锟斤拷锟斤拷拇锟斤拷锟斤拷锟�
  * @note   锟斤拷锟街节讹拷取时锟斤拷锟斤拷锟斤拷锟斤拷 bit1锟斤拷MS锟斤拷锟斤拷锟斤拷锟斤拷 1锟斤拷
  */
static HAL_StatusTypeDef H3LIS100DL_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef ret;
  uint8_t cmd;

  if ((buf == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  cmd = H3LIS100DL_SPI_MAKE_CMD(reg, 1U, 1U);

  H3_SPI2_CS_LOW();

  ret = HAL_SPI_Transmit(&hspi2, &cmd, 1U, H3LIS100DL_SPI_TIMEOUT_MS);
  if (ret == HAL_OK)
  {
    uint16_t i;
    for (i = 0U; i < len; i++)
    {
      uint8_t tx_dummy = 0x00U;
      uint8_t rx_data = 0x00U;

      ret = HAL_SPI_TransmitReceive(&hspi2, &tx_dummy, &rx_data, 1U, H3LIS100DL_SPI_TIMEOUT_MS);
      if (ret != HAL_OK)
      {
        break;
      }
      buf[i] = rx_data;
    }
  }

  H3_SPI2_CS_HIGH();

  return ret;
}

int H3LIS100DL_Init(void)
{
  HAL_StatusTypeDef ret;
  uint8_t who = 0U;
  uint8_t retry;

  memset(&g_h3lis, 0, sizeof(g_h3lis));

  H3_SPI2_CS_HIGH();
  HAL_Delay(10U);

  /* Read WHO_AM_I with up to 3 retries */
  for (retry = 0U; retry < 3U; retry++)
  {
    who = 0U;
    ret = H3LIS100DL_ReadReg(H3LIS100DL_REG_WHO_AM_I, &who);
    if ((ret == HAL_OK) && (who == H3LIS100DL_WHO_AM_I_VALUE))
    {
      break;
    }
    HAL_Delay(5U);
  }

  if (who != H3LIS100DL_WHO_AM_I_VALUE)
  {
    printf("[H3LIS100DL] 初始化失败 (WHO_AM_I=0x%02X)\r\n", who);
    return -1;
  }

  g_h3lis.who_am_i = who;

  /* Boot sequence */
  ret = H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG2, 0x80U);
  if (ret != HAL_OK)
  {
    printf("[H3LIS100DL] 初始化失败 (BOOT)\r\n");
    return -1;
  }
  HAL_Delay(10U);

  /* Configure: normal mode 100Hz, XYZ enabled */
  ret = H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG1, 0x2FU);
  if (ret != HAL_OK)
  {
    printf("[H3LIS100DL] 初始化失败 (CTRL_REG1)\r\n");
    return -1;
  }

  ret = H3LIS100DL_WriteReg(H3LIS100DL_REG_CTRL_REG4, 0x00U);
  if (ret != HAL_OK)
  {
    printf("[H3LIS100DL] 初始化失败 (CTRL_REG4)\r\n");
    return -1;
  }

  /* Verify configuration */
  {
    uint8_t v1 = 0U;
    H3LIS100DL_ReadReg(H3LIS100DL_REG_CTRL_REG1, &v1);
    if (v1 != 0x2FU)
    {
      printf("[H3LIS100DL] 初始化失败 (CTRL_REG1 verify)\r\n");
      return -1;
    }
  }

  printf("[H3LIS100DL] 初始化成功 (+/-100g 100Hz)\r\n");
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
  uint8_t raw_u8;
  uint8_t status = 0U;

  if (data == NULL)
  {
    return -1;
  }

  /* 妫€鏌ユ暟鎹氨缁爣蹇� STATUS_REG[3]=ZYXDA */
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_STATUS, &status) != HAL_OK)
  {
    return -1;
  }
  if ((status & H3LIS100DL_STATUS_ZYXDA) == 0U)
  {
    return -2;  /* 鏁版嵁灏氭湭灏辩华, 璋冪敤鏂瑰彲閫夋嫨閲嶈瘯 */
  }

  /* OUT_X/Y/Z 鍦板潃涓嶈繛缁� (0x29/0x2B/0x2D), 閫愪竴璇诲彇 */
  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_X, &raw_u8) != HAL_OK) return -1;
  data->raw[0] = (int8_t)raw_u8;

  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_Y, &raw_u8) != HAL_OK) return -1;
  data->raw[1] = (int8_t)raw_u8;

  if (H3LIS100DL_ReadReg(H3LIS100DL_REG_OUT_Z, &raw_u8) != HAL_OK) return -1;
  data->raw[2] = (int8_t)raw_u8;

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
