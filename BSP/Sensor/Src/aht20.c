#include "aht20.h"
#include "i2c.h"
#include <stdio.h>

#define AHT20_ADDR_8BIT     (0x38U << 1U)   /* 0x70 */
#define AHT20_STATUS_BUSY   0x80U
#define AHT20_STATUS_CAL    0x08U

extern I2C_HandleTypeDef hi2c1;

/* CRC8：poly=0x31 (x^8+x^5+x^4+1), init=0xFF（手册第12页） */
static uint8_t aht20_crc8(const uint8_t *d, uint8_t n)
{
  uint8_t crc = 0xFFU;
  for (uint8_t i = 0U; i < n; i++)
  {
    crc ^= d[i];
    for (uint8_t b = 0U; b < 8U; b++)
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
  }
  return crc;
}

HAL_StatusTypeDef AHT20_Init(void)
{
  uint8_t st = 0U;
  HAL_Delay(40U);   /* 上电 ≥40ms（一次性，初始化阶段忙等可接受） */

  if (HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR_8BIT, &st, 1U, 100U) != HAL_OK)
    return HAL_ERROR;

  if ((st & AHT20_STATUS_CAL) == 0U)
  {
    uint8_t init_cmd[3] = { 0xBEU, 0x08U, 0x00U };  /* 校准/初始化命令 */
    if (HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR_8BIT, init_cmd, 3U, 100U) != HAL_OK)
      return HAL_ERROR;
    HAL_Delay(10U);
  }
  printf("[AHT20] init ok (status=0x%02X)\r\n", st);
  return HAL_OK;
}

HAL_StatusTypeDef AHT20_TriggerMeasure(void)
{
  uint8_t cmd[3] = { 0xACU, 0x33U, 0x00U };
  return HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR_8BIT, cmd, 3U, 100U);
}

HAL_StatusTypeDef AHT20_ReadResult(float *temp_c, float *humidity_pct)
{
  uint8_t buf[7] = {0};
  if ((temp_c == NULL) || (humidity_pct == NULL)) return HAL_ERROR;

  if (HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR_8BIT, buf, 7U, 100U) != HAL_OK)
    return HAL_ERROR;

  if ((buf[0] & AHT20_STATUS_BUSY) != 0U)   /* 仍在测量，数据无效 */
    return HAL_BUSY;

  if (aht20_crc8(buf, 6U) != buf[6])        /* CRC 校验失败 */
    return HAL_ERROR;

  uint32_t rh = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | ((uint32_t)buf[3] >> 4);
  uint32_t t  = (((uint32_t)buf[3] & 0x0FU) << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];

  *humidity_pct = (float)rh * 100.0f  / 1048576.0f;
  *temp_c       = (float)t  * 200.0f  / 1048576.0f - 50.0f;
  return HAL_OK;
}
