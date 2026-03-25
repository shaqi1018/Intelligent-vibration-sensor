/**
  ******************************************************************************
  * @file    lsm6dsox.c
  * @brief   LSM6DSOX driver (SPI1, PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS)
  *          Default: accel +/-4g 104Hz, gyro +/-2000dps 104Hz
  ******************************************************************************
  */

#include "lsm6dsox.h"
#include <stdio.h>

#define LSM6DSOX_SPI_READ_FLAG   0x80

static float xl_sensitivity = LSM6DSOX_XL_SENSITIVITY_4G;
static float g_sensitivity  = LSM6DSOX_G_SENSITIVITY_2000;

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
  LSM6DSOX_DelayUs(1);                     /* tsu(CS) 建立时间 */
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
  LSM6DSOX_UpdateSensitivity(cfg->xl_fs, cfg->g_fs);
  return HAL_OK;
}

HAL_StatusTypeDef LSM6DSOX_Init(void)
{
  uint8_t id = 0, reg_val = 0;
  uint8_t retry;
  LSM6DSOX_Config_t default_cfg = {
    .xl_odr = LSM6DSOX_XL_ODR_104Hz, .xl_fs = LSM6DSOX_XL_FS_4G,
    .g_odr = LSM6DSOX_G_ODR_104Hz,   .g_fs = LSM6DSOX_G_FS_2000DPS,
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
  printf("[LSM6DSOX] 初始化成功 (加速度:+/-4g 104Hz, 陀螺仪:+/-2000dps 104Hz)\r\n");
  return HAL_OK;
}



HAL_StatusTypeDef LSM6DSOX_ReadAccel(LSM6DSOX_Data_t *data)
{
  uint8_t buf[6];
  int16_t raw_x, raw_y, raw_z;
  HAL_StatusTypeDef status;

  status = LSM6DSOX_ReadRegs(LSM6DSOX_REG_OUTX_L_A, buf, 6);
  if (status != HAL_OK) return status;

  raw_x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
  raw_y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
  raw_z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

  data->x = (float)raw_x * xl_sensitivity;
  data->y = (float)raw_y * xl_sensitivity;
  data->z = (float)raw_z * xl_sensitivity;

  return HAL_OK;
}

HAL_StatusTypeDef LSM6DSOX_ReadGyro(LSM6DSOX_Data_t *data)
{
  uint8_t buf[6];
  int16_t raw_x, raw_y, raw_z;
  HAL_StatusTypeDef status;

  status = LSM6DSOX_ReadRegs(LSM6DSOX_REG_OUTX_L_G, buf, 6);
  if (status != HAL_OK) return status;

  raw_x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
  raw_y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
  raw_z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

  data->x = (float)raw_x * g_sensitivity;
  data->y = (float)raw_y * g_sensitivity;
  data->z = (float)raw_z * g_sensitivity;

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