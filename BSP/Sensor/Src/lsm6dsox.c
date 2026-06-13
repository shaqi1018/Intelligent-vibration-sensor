/**
  ******************************************************************************
  * @file    lsm6dsox.c
 * @brief   LSM6DSOX driver (SPI1, PA5=SCK, PA6=MISO, PA7=MOSI, PC4=CS)
  *          Default: accel +/-4g 104Hz, gyro +/-2000dps 104Hz
  ******************************************************************************
  */

#include "lsm6dsox.h"
#include "dma_sampling.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"

#define LSM6DSOX_SPI_READ_FLAG   0x80

static float xl_sensitivity = LSM6DSOX_XL_SENSITIVITY_4G;
static float g_sensitivity  = LSM6DSOX_G_SENSITIVITY_2000;
static volatile uint32_t dma_call_count = 0;

/* ======================== 微秒级延时(粗略) ============================== */
static void LSM6DSOX_DelayUs(volatile uint32_t us)
{
  /* 160MHz 主频下约 32 个循环 ≈ 1us, 此处取保守值 */
  volatile uint32_t cnt = us * 20;
  while (cnt--) { __NOP(); }
}

/* ======================== 底层 SPI 读写函数 ============================== */

static HAL_StatusTypeDef LSM6DSOX_WriteReg(uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2];

  tx[0] = reg & ~LSM6DSOX_SPI_READ_FLAG;  /* bit7=0: 写 */
  tx[1] = data;

  LSM_SPI_CS_LOW();
  LSM6DSOX_DelayUs(1);                     /* tsu(CS)和 建立时间 */
  ret = HAL_SPI_Transmit(&hspi1, tx, 2, LSM_SPI_TIMEOUT_MS);
  LSM_SPI_CS_HIGH();
  LSM6DSOX_DelayUs(1);                     /* CS 间隔 */

  return ret;
}

static HAL_StatusTypeDef LSM6DSOX_ReadReg(uint8_t reg, uint8_t *data)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[2], rx[2];

  tx[0] = reg | LSM6DSOX_SPI_READ_FLAG;   /* bit7=1: 读 */
  tx[1] = 0x00;                            /* dummy */

  LSM_SPI_CS_LOW();
  LSM6DSOX_DelayUs(1);                     /* tsu(CS) 建立时间 */
  ret = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, LSM_SPI_TIMEOUT_MS);
  LSM_SPI_CS_HIGH();
  LSM6DSOX_DelayUs(1);                     /* CS 间隔 */

  if (ret == HAL_OK)
    *data = rx[1];

  return ret;
}

static HAL_StatusTypeDef LSM6DSOX_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[15] = {0};
  uint8_t rx[15] = {0};

  tx[0] = reg | LSM6DSOX_SPI_READ_FLAG;

  LSM_SPI_CS_LOW();
  LSM6DSOX_DelayUs(1);                     /* tsu(CS) 建立时间 */
  ret = HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 1, LSM_SPI_TIMEOUT_MS);
  LSM_SPI_CS_HIGH();
  LSM6DSOX_DelayUs(1);                     /* CS 间隔 */

  if (ret == HAL_OK)
    for (uint16_t i = 0; i < len; i++)
      buf[i] = rx[i + 1];

  return ret;
}

/* ======================== Public API functions ============================ */

HAL_StatusTypeDef LSM6DSOX_ReadID(uint8_t *id)
{
  return LSM6DSOX_ReadReg(LSM6DSOX_REG_WHO_AM_I, id);
}

static void LSM6DSOX_UpdateSensitivity(uint8_t xl_fs, uint8_t g_fs)
{
  switch (xl_fs) {
    case LSM6DSOX_XL_FS_2G:  xl_sensitivity = LSM6DSOX_XL_SENSITIVITY_2G;  break;
    case LSM6DSOX_XL_FS_4G:  xl_sensitivity = LSM6DSOX_XL_SENSITIVITY_4G;  break;
    case LSM6DSOX_XL_FS_8G:  xl_sensitivity = LSM6DSOX_XL_SENSITIVITY_8G;  break;
    case LSM6DSOX_XL_FS_16G: xl_sensitivity = LSM6DSOX_XL_SENSITIVITY_16G; break;
    default:                 xl_sensitivity = LSM6DSOX_XL_SENSITIVITY_4G;  break;
  }
  switch (g_fs) {
    case LSM6DSOX_G_FS_250DPS:  g_sensitivity = LSM6DSOX_G_SENSITIVITY_250;  break;
    case LSM6DSOX_G_FS_500DPS:  g_sensitivity = LSM6DSOX_G_SENSITIVITY_500;  break;
    case LSM6DSOX_G_FS_1000DPS: g_sensitivity = LSM6DSOX_G_SENSITIVITY_1000; break;
    case LSM6DSOX_G_FS_2000DPS: g_sensitivity = LSM6DSOX_G_SENSITIVITY_2000; break;
    default:                    g_sensitivity = LSM6DSOX_G_SENSITIVITY_2000; break;
  }
}

HAL_StatusTypeDef LSM6DSOX_ReadStatus(uint8_t *status)
{
  return LSM6DSOX_ReadReg(LSM6DSOX_REG_STATUS, status);
}

HAL_StatusTypeDef LSM6DSOX_Configure(const LSM6DSOX_Config_t *cfg)
{
  uint8_t ctrl3 = 0;
  if (cfg == NULL) return HAL_ERROR;
  if (cfg->enable_bdu) ctrl3 |= LSM6DSOX_CTRL3_BDU;
  if (cfg->enable_inc) ctrl3 |= LSM6DSOX_CTRL3_IF_INC;
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_CTRL3_C, ctrl3) != HAL_OK) return HAL_ERROR;
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_CTRL1_XL, cfg->xl_odr | cfg->xl_fs) != HAL_OK) return HAL_ERROR;
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_CTRL2_G,  cfg->g_odr  | cfg->g_fs)  != HAL_OK) return HAL_ERROR;

  /* Read back CTRL1_XL to verify FS bits were actually written (chip may be damaged/hot) */
  {
    uint8_t actual = 0;
    uint8_t actual_fs;
    if (LSM6DSOX_ReadReg(LSM6DSOX_REG_CTRL1_XL, &actual) != HAL_OK) return HAL_ERROR;
    actual_fs = actual & 0x0CU;
    if (actual_fs != (cfg->xl_fs & 0x0CU))
    {
      printf("[LSM6DSOX] WARN: CTRL1_XL FS mismatch wrote=0x%02X read=0x%02X, using actual\r\n",
             (unsigned int)(cfg->xl_odr | cfg->xl_fs), (unsigned int)actual);
    }
    LSM6DSOX_UpdateSensitivity(actual_fs, cfg->g_fs);
  }
  return HAL_OK;
}

HAL_StatusTypeDef LSM6DSOX_Init(void)
{
  uint8_t id = 0, reg_val = 0;
  uint8_t retry;
  LSM6DSOX_Config_t default_cfg = {
    .xl_odr = LSM6DSOX_XL_ODR_6664Hz, .xl_fs = LSM6DSOX_XL_FS_4G,
    .g_odr = LSM6DSOX_G_ODR_6664Hz,   .g_fs = LSM6DSOX_G_FS_2000DPS,
    .enable_bdu = 1, .enable_inc = 1
  };

  /* 等待传感器上电 (datasheet: boot time <= 10ms) */
  HAL_Delay(20);

  if (LSM6DSOX_ReadID(&id) != HAL_OK || id != LSM6DSOX_WHO_AM_I_VALUE) {
    printf("[LSM6DSOX] WHO_AM_I 失败: 读=0x%02X 期望=0x%02X\r\n",
           id, LSM6DSOX_WHO_AM_I_VALUE);
    return HAL_ERROR;
  }

  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_CTRL3_C, LSM6DSOX_CTRL3_SW_RESET) != HAL_OK) return HAL_ERROR;
  do { HAL_Delay(1);
    if (LSM6DSOX_ReadReg(LSM6DSOX_REG_CTRL3_C, &reg_val) != HAL_OK) return HAL_ERROR;
  } while (reg_val & LSM6DSOX_CTRL3_SW_RESET);

  /* 软件复位后再次等待 */
  HAL_Delay(10);

  if (LSM6DSOX_Configure(&default_cfg) != HAL_OK) return HAL_ERROR;
  printf("[LSM6DSOX] 初始化成功 (加速度:+/-4g 6664Hz, 陀螺仪:+/-2000dps 6664Hz)\r\n");
  return HAL_OK;
}



HAL_StatusTypeDef LSM6DSOX_ReadTemp(float *temp_C)
{
  uint8_t buf[2];
  int16_t raw_temp;
  HAL_StatusTypeDef status;

  status = LSM6DSOX_ReadRegs(LSM6DSOX_REG_OUT_TEMP_L, buf, 2);
  if (status != HAL_OK) return status;

  raw_temp = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
  *temp_C = (float)raw_temp / LSM6DSOX_TEMP_SENSITIVITY + LSM6DSOX_TEMP_OFFSET;

  return HAL_OK;
}

HAL_StatusTypeDef LSM6DSOX_ReadAllData(LSM6DSOX_AllData_t *all)
{
  uint8_t buf[14];
  int16_t raw;
  HAL_StatusTypeDef status;

  status = LSM6DSOX_ReadRegs(LSM6DSOX_REG_OUT_TEMP_L, buf, 14);
  if (status != HAL_OK) return status;

  /* Temperature (buf[0..1]) */
  raw = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
  all->temp_C = (float)raw / LSM6DSOX_TEMP_SENSITIVITY + LSM6DSOX_TEMP_OFFSET;

  /* Gyro (buf[2..7]) */
  raw = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
  all->gyro.x = (float)raw * g_sensitivity;
  raw = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
  all->gyro.y = (float)raw * g_sensitivity;
  raw = (int16_t)((uint16_t)buf[7] << 8 | buf[6]);
  all->gyro.z = (float)raw * g_sensitivity;

  /* Accel (buf[8..13]) */
  raw = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
  all->acc.x = (float)raw * xl_sensitivity;
  raw = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);
  all->acc.y = (float)raw * xl_sensitivity;
  raw = (int16_t)((uint16_t)buf[13] << 8 | buf[12]);
  all->acc.z = (float)raw * xl_sensitivity;

  return HAL_OK;
}


uint32_t LSM6DSOX_GetDmaCallCount(void)
{
  return dma_call_count;
}

/* ======================== FIFO public API ================================== */

HAL_StatusTypeDef LSM6DSOX_FIFO_Config(uint16_t wtm_samples,
                                       uint8_t bdr_xl_code,
                                       uint8_t bdr_gy_code)
{
  /* Step 1: bypass mode to reset FIFO */
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_FIFO_CTRL4, LSM6DSOX_FIFO_MODE_BYPASS) != HAL_OK) return HAL_ERROR;

  /* Step 2: watermark threshold (9-bit) */
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_FIFO_CTRL1, (uint8_t)(wtm_samples & 0xFFU)) != HAL_OK) return HAL_ERROR;
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_FIFO_CTRL2, (uint8_t)((wtm_samples >> 8) & 0x01U)) != HAL_OK) return HAL_ERROR;

  /* Step 3: BDR for accel + gyro (BDR_GY high nibble, BDR_XL low nibble) */
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_FIFO_CTRL3,
                        (uint8_t)((bdr_gy_code << 4) | (bdr_xl_code & 0x0FU))) != HAL_OK) return HAL_ERROR;

  /* Step 4: route FIFO threshold flag to INT1 pin */
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_INT1_CTRL, LSM6DSOX_INT1_FIFO_TH) != HAL_OK) return HAL_ERROR;

  /* Step 5: enter continuous mode */
  if (LSM6DSOX_WriteReg(LSM6DSOX_REG_FIFO_CTRL4, LSM6DSOX_FIFO_MODE_CONTINUOUS) != HAL_OK) return HAL_ERROR;

  return HAL_OK;
}

HAL_StatusTypeDef LSM6DSOX_FIFO_GetLevel(uint16_t *level)
{
  uint8_t s1 = 0, s2 = 0;
  if (level == NULL) return HAL_ERROR;
  if (LSM6DSOX_ReadReg(LSM6DSOX_REG_FIFO_STATUS1, &s1) != HAL_OK) return HAL_ERROR;
  if (LSM6DSOX_ReadReg(LSM6DSOX_REG_FIFO_STATUS2, &s2) != HAL_OK) return HAL_ERROR;
  *level = (uint16_t)(((uint16_t)(s2 & 0x03U) << 8) | s1);
  return HAL_OK;
}

HAL_StatusTypeDef LSM6DSOX_FIFO_ReadBlock(uint8_t *buf, uint16_t n_words)
{
  /* Each FIFO word = 7 bytes (1 tag + 6 data). Auto-increment is enabled
   * (CTRL3_C IF_INC); a continuous burst from 0x78 streams the FIFO, but the
   * device needs a brief gap at each word boundary (0x7E->0x78 wrap) for the
   * FIFO to advance to the next sample. A fully-continuous bulk read (no gaps)
   * only returned the first word + non-advancing junk (~122 Hz). The old
   * per-byte loop worked but did 1792 HAL calls/batch (too slow → ~55% capture
   * at 6664 Hz). Read ONE 7-byte word per HAL call instead: the inter-call gap
   * lands exactly on the word boundary, and it is 7x fewer calls (256/batch). */
  static const uint8_t dummy7[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint8_t tx_cmd;
  HAL_StatusTypeDef ret;

  if ((buf == NULL) || (n_words == 0U)) return HAL_ERROR;

  tx_cmd = (uint8_t)(LSM6DSOX_REG_FIFO_DATA_TAG | LSM6DSOX_SPI_READ_FLAG);

  LSM_SPI_CS_LOW();
  LSM6DSOX_DelayUs(1);
  ret = HAL_SPI_Transmit(&hspi1, &tx_cmd, 1, LSM_SPI_TIMEOUT_MS);
  if (ret == HAL_OK)
  {
    for (uint16_t w = 0U; w < n_words; w++)
    {
      ret = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)dummy7, &buf[w * 7U], 7U,
                                    LSM_SPI_TIMEOUT_MS);
      if (ret != HAL_OK) break;
    }
  }
  LSM_SPI_CS_HIGH();
  LSM6DSOX_DelayUs(1);
  return ret;
}
