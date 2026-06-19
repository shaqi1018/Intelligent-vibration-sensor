#include "lis2mdl.h"
#include "i2c.h"
#include <stdio.h>

#define LIS2MDL_ADDR_8BIT   (0x1EU << 1U)   /* 0x3C */
#define LIS2MDL_WHO_AM_I    0x4FU            /* =0x40 */
#define LIS2MDL_CFG_REG_A   0x60U
#define LIS2MDL_CFG_REG_B   0x61U
#define LIS2MDL_CFG_REG_C   0x62U
#define LIS2MDL_STATUS_REG  0x67U
#define LIS2MDL_OUTX_L      0x68U
#define LIS2MDL_AUTO_INC    0x80U            /* 子地址 MSB=1 → 多字节自增 */

#define LIS2MDL_WHOAMI_VAL  0x40U
#define LIS2MDL_SENS_MG_LSB 1.5f             /* 1.5 mgauss/LSB */

extern I2C_HandleTypeDef hi2c1;

static HAL_StatusTypeDef lis_wr(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, LIS2MDL_ADDR_8BIT, reg,
                           I2C_MEMADD_SIZE_8BIT, &val, 1U, 100U);
}
static HAL_StatusTypeDef lis_rd(uint8_t reg, uint8_t *val)
{
  return HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_ADDR_8BIT, reg,
                          I2C_MEMADD_SIZE_8BIT, val, 1U, 100U);
}

/* odr_hz → CFG_REG_A bit[3:2] */
static uint8_t lis_odr_bits(uint16_t odr_hz)
{
  switch (odr_hz)
  {
    case 10:  return 0x00U;
    case 20:  return 0x04U;
    case 50:  return 0x08U;
    case 100: return 0x0CU;
    default:  return 0x0CU;   /* 缺省 100Hz */
  }
}

HAL_StatusTypeDef LIS2MDL_Init(uint16_t odr_hz)
{
  uint8_t who = 0U;
  if (lis_rd(LIS2MDL_WHO_AM_I, &who) != HAL_OK) return HAL_ERROR;
  if (who != LIS2MDL_WHOAMI_VAL)
  {
    printf("[LIS2MDL] WHO_AM_I mismatch: 0x%02X (expect 0x40)\r\n", who);
    return HAL_ERROR;
  }

  /* CFG_A: COMP_TEMP_EN(0x80) | ODR | MD=00(连续) */
  if (lis_wr(LIS2MDL_CFG_REG_A, (uint8_t)(0x80U | lis_odr_bits(odr_hz))) != HAL_OK) return HAL_ERROR;
  /* CFG_B: 默认 0x00（不开 LPF/offset-cancel） */
  if (lis_wr(LIS2MDL_CFG_REG_B, 0x00U) != HAL_OK) return HAL_ERROR;
  /* CFG_C: BDU(0x10) | DRDY_on_PIN(0x01) → DRDY 输出到 PC13 */
  if (lis_wr(LIS2MDL_CFG_REG_C, 0x11U) != HAL_OK) return HAL_ERROR;

  printf("[LIS2MDL] init ok (WHO_AM_I=0x40, ODR=%uHz)\r\n", (unsigned)odr_hz);
  return HAL_OK;
}

uint8_t LIS2MDL_DataReady(void)
{
  uint8_t st = 0U;
  if (lis_rd(LIS2MDL_STATUS_REG, &st) != HAL_OK) return 0U;
  return (st & 0x08U) ? 1U : 0U;   /* Zyxda */
}

HAL_StatusTypeDef LIS2MDL_ReadMag(int16_t raw[3], float mag_mg[3])
{
  uint8_t b[6] = {0};
  /* OUTX_L 起读 6 字节，子地址带 auto-inc 位 */
  if (HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_ADDR_8BIT,
                       (uint16_t)(LIS2MDL_OUTX_L | LIS2MDL_AUTO_INC),
                       I2C_MEMADD_SIZE_8BIT, b, 6U, 100U) != HAL_OK)
    return HAL_ERROR;

  raw[0] = (int16_t)((uint16_t)b[1] << 8 | b[0]);
  raw[1] = (int16_t)((uint16_t)b[3] << 8 | b[2]);
  raw[2] = (int16_t)((uint16_t)b[5] << 8 | b[4]);
  mag_mg[0] = (float)raw[0] * LIS2MDL_SENS_MG_LSB;
  mag_mg[1] = (float)raw[1] * LIS2MDL_SENS_MG_LSB;
  mag_mg[2] = (float)raw[2] * LIS2MDL_SENS_MG_LSB;
  return HAL_OK;
}
