/**
  ******************************************************************************
  * @file    lsm6dsox.h
  * @brief   LSM6DSOX 6-axis IMU (accelerometer + gyroscope) driver header
  *
  *          Sensor  : LSM6DSOX (ST MEMS)
  *          Adapter : STEVAL-MKI197V1 (DIL24)
  *          Bus     : SPI1
  *
  *   DIL24 wiring to STM32U575:
  *     VDD(Pin1)->3.3V   VDD_IO(Pin2)->3.3V   GND(Pin5)->GND
 *     CS(Pin14)->PC4    SCL/SPC(Pin9)->PA5    SDA/SDI(Pin11)->PA7
  *     SDO/SA0(Pin12)->PA6   INT1(Pin8)->PE0   INT2(Pin7)->PE1
  ******************************************************************************
  */
#ifndef __LSM6DSOX_H__
#define __LSM6DSOX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_spi.h"
#include <stdint.h>

/* ======================== SPI R/W address bits ============================= */
#define LSM6DSOX_SPI_READ           0x80
#define LSM6DSOX_SPI_WRITE          0x00

/* ======================== Register addresses =============================== */
#define LSM6DSOX_REG_WHO_AM_I       0x0F
#define LSM6DSOX_WHO_AM_I_VALUE     0x6C

#define LSM6DSOX_REG_CTRL1_XL       0x10
#define LSM6DSOX_REG_CTRL2_G        0x11
#define LSM6DSOX_REG_CTRL3_C        0x12
#define LSM6DSOX_REG_CTRL4_C        0x13
#define LSM6DSOX_REG_CTRL5_C        0x14
#define LSM6DSOX_REG_CTRL6_C        0x15
#define LSM6DSOX_REG_CTRL7_G        0x16
#define LSM6DSOX_REG_CTRL8_XL       0x17

#define LSM6DSOX_REG_STATUS         0x1E
#define LSM6DSOX_REG_OUT_TEMP_L     0x20
#define LSM6DSOX_REG_OUT_TEMP_H     0x21

#define LSM6DSOX_REG_OUTX_L_G       0x22
#define LSM6DSOX_REG_OUTX_H_G       0x23
#define LSM6DSOX_REG_OUTY_L_G       0x24
#define LSM6DSOX_REG_OUTY_H_G       0x25
#define LSM6DSOX_REG_OUTZ_L_G       0x26
#define LSM6DSOX_REG_OUTZ_H_G       0x27

#define LSM6DSOX_REG_OUTX_L_A       0x28
#define LSM6DSOX_REG_OUTX_H_A       0x29
#define LSM6DSOX_REG_OUTY_L_A       0x2A
#define LSM6DSOX_REG_OUTY_H_A       0x2B
#define LSM6DSOX_REG_OUTZ_L_A       0x2C
#define LSM6DSOX_REG_OUTZ_H_A       0x2D

/* ======================== FIFO registers =================================== */
#define LSM6DSOX_REG_FIFO_CTRL1     0x07  /* WTM[7:0] */
#define LSM6DSOX_REG_FIFO_CTRL2     0x08  /* WTM[8] in bit0 */
#define LSM6DSOX_REG_FIFO_CTRL3     0x09  /* BDR_GY[3:0] | BDR_XL[3:0] */
#define LSM6DSOX_REG_FIFO_CTRL4     0x0A  /* DEC_TS|ODR_T|0|FIFO_MODE[2:0] */
#define LSM6DSOX_REG_INT1_CTRL      0x0D
#define LSM6DSOX_REG_FIFO_STATUS1   0x3A  /* DIFF_FIFO[7:0] */
#define LSM6DSOX_REG_FIFO_STATUS2   0x3B  /* DIFF_FIFO[9:8] in bits[1:0] */
#define LSM6DSOX_REG_FIFO_DATA_TAG  0x78  /* tag byte; +1..+6 = 6 data bytes */

/* FIFO_CTRL3 BDR codes (matched to ODR codes) */
#define LSM6DSOX_BDR_OFF            0x00
#define LSM6DSOX_BDR_12_5Hz         0x01
#define LSM6DSOX_BDR_26Hz           0x02
#define LSM6DSOX_BDR_52Hz           0x03
#define LSM6DSOX_BDR_104Hz          0x04
#define LSM6DSOX_BDR_208Hz          0x05
#define LSM6DSOX_BDR_417Hz          0x06
#define LSM6DSOX_BDR_833Hz          0x07
#define LSM6DSOX_BDR_1667Hz         0x08
#define LSM6DSOX_BDR_3333Hz         0x09
#define LSM6DSOX_BDR_6667Hz         0x0A

/* FIFO_CTRL4 modes */
#define LSM6DSOX_FIFO_MODE_BYPASS     0x00
#define LSM6DSOX_FIFO_MODE_FIFO       0x01
#define LSM6DSOX_FIFO_MODE_CONTINUOUS 0x06

/* INT1_CTRL bits */
#define LSM6DSOX_INT1_FIFO_TH       0x08

/* FIFO_STATUS2 bits */
#define LSM6DSOX_FIFO_WTM_IA_BIT    0x80

/* FIFO TAG: tag_byte >> 3, see datasheet table 197 */
#define LSM6DSOX_TAG_GYRO_NC        0x01
#define LSM6DSOX_TAG_ACC_NC         0x02
#define LSM6DSOX_TAG_TEMP           0x03
#define LSM6DSOX_TAG_TIMESTAMP      0x04

/* ======================== CTRL1_XL: Accel ODR & FS ========================= */
#define LSM6DSOX_XL_ODR_OFF         0x00
#define LSM6DSOX_XL_ODR_12_5Hz      0x10
#define LSM6DSOX_XL_ODR_26Hz        0x20
#define LSM6DSOX_XL_ODR_52Hz        0x30
#define LSM6DSOX_XL_ODR_104Hz       0x40
#define LSM6DSOX_XL_ODR_208Hz       0x50
#define LSM6DSOX_XL_ODR_416Hz       0x60
#define LSM6DSOX_XL_ODR_833Hz       0x70
#define LSM6DSOX_XL_ODR_1666Hz      0x80
#define LSM6DSOX_XL_ODR_3332Hz      0x90
#define LSM6DSOX_XL_ODR_6664Hz      0xA0

#define LSM6DSOX_XL_FS_2G           0x00
#define LSM6DSOX_XL_FS_4G           0x08
#define LSM6DSOX_XL_FS_8G           0x0C
#define LSM6DSOX_XL_FS_16G          0x04

/* ======================== CTRL2_G: Gyro ODR & FS =========================== */
#define LSM6DSOX_G_ODR_OFF          0x00
#define LSM6DSOX_G_ODR_12_5Hz       0x10
#define LSM6DSOX_G_ODR_26Hz         0x20
#define LSM6DSOX_G_ODR_52Hz         0x30
#define LSM6DSOX_G_ODR_104Hz        0x40
#define LSM6DSOX_G_ODR_208Hz        0x50
#define LSM6DSOX_G_ODR_416Hz        0x60
#define LSM6DSOX_G_ODR_833Hz        0x70
#define LSM6DSOX_G_ODR_1666Hz       0x80
#define LSM6DSOX_G_ODR_3332Hz       0x90
#define LSM6DSOX_G_ODR_6664Hz       0xA0

#define LSM6DSOX_G_FS_250DPS        0x00
#define LSM6DSOX_G_FS_500DPS        0x04
#define LSM6DSOX_G_FS_1000DPS       0x08
#define LSM6DSOX_G_FS_2000DPS       0x0C

/* ======================== CTRL3_C bits ===================================== */
#define LSM6DSOX_CTRL3_BDU          0x40
#define LSM6DSOX_CTRL3_IF_INC       0x04
#define LSM6DSOX_CTRL3_SW_RESET     0x01

/* ======================== STATUS bits ====================================== */
#define LSM6DSOX_STATUS_XLDA        0x01
#define LSM6DSOX_STATUS_GDA         0x02
#define LSM6DSOX_STATUS_TDA         0x04

/* ======================== Sensitivity constants ============================ */
#define LSM6DSOX_XL_SENSITIVITY_2G    0.061f   /* mg/LSB  */
#define LSM6DSOX_XL_SENSITIVITY_4G    0.122f
#define LSM6DSOX_XL_SENSITIVITY_8G    0.244f
#define LSM6DSOX_XL_SENSITIVITY_16G   0.488f

#define LSM6DSOX_G_SENSITIVITY_250    8.750f   /* mdps/LSB */
#define LSM6DSOX_G_SENSITIVITY_500    17.500f
#define LSM6DSOX_G_SENSITIVITY_1000   35.000f
#define LSM6DSOX_G_SENSITIVITY_2000   70.000f

#define LSM6DSOX_TEMP_SENSITIVITY     256.0f   /* LSB/degC */
#define LSM6DSOX_TEMP_OFFSET          25.0f
#define LSM6DSOX_DMA_DEBUG_LOG        0

/* ======================== INT pins ========================================= */
#define LSM6DSOX_INT1_PIN           GPIO_PIN_1
#define LSM6DSOX_INT1_GPIO_PORT     GPIOB
#define LSM6DSOX_INT2_PIN           GPIO_PIN_0
#define LSM6DSOX_INT2_GPIO_PORT     GPIOB

/* ======================== Data types ======================================= */
typedef struct { int16_t x, y, z; } LSM6DSOX_RawData_t;
typedef struct { float   x, y, z; } LSM6DSOX_Data_t;

typedef struct {
  LSM6DSOX_Data_t acc;
  LSM6DSOX_Data_t gyro;
  float           temp_C;
} LSM6DSOX_AllData_t;

typedef struct {
  uint8_t xl_odr, xl_fs, g_odr, g_fs;
  uint8_t enable_bdu, enable_inc;
} LSM6DSOX_Config_t;

/* ======================== API functions ===================================== */
HAL_StatusTypeDef LSM6DSOX_Init(void);
HAL_StatusTypeDef LSM6DSOX_Configure(const LSM6DSOX_Config_t *cfg);
HAL_StatusTypeDef LSM6DSOX_ReadID(uint8_t *id);
HAL_StatusTypeDef LSM6DSOX_ReadStatus(uint8_t *status);
HAL_StatusTypeDef LSM6DSOX_ReadAccel(LSM6DSOX_Data_t *data);
HAL_StatusTypeDef LSM6DSOX_ReadGyro(LSM6DSOX_Data_t *data);
HAL_StatusTypeDef LSM6DSOX_ReadTemp(float *temp_C);
HAL_StatusTypeDef LSM6DSOX_ReadAllData(LSM6DSOX_AllData_t *all);
HAL_StatusTypeDef LSM6DSOX_ReadAllData_DMA(LSM6DSOX_AllData_t *all);
uint32_t LSM6DSOX_GetDmaCallCount(void);

/* ======================== FIFO API ========================================== */
/**
 * @brief Configure LSM6DSOX FIFO: continuous mode, ACC+GYRO batched at given
 *        BDR code, watermark = wtm_samples (1..511), watermark interrupt
 *        routed to INT1 pin (push-pull, active high).
 *        Call after LSM6DSOX_Init().
 */
HAL_StatusTypeDef LSM6DSOX_FIFO_Config(uint16_t wtm_samples,
                                       uint8_t bdr_xl_code,
                                       uint8_t bdr_gy_code);

/**
 * @brief Reset FIFO content (set bypass mode then continuous mode).
 */
HAL_StatusTypeDef LSM6DSOX_FIFO_Reset(void);

/**
 * @brief Read FIFO fill level (number of TAG+6 byte words pending).
 */
HAL_StatusTypeDef LSM6DSOX_FIFO_GetLevel(uint16_t *level);

/**
 * @brief Read N FIFO words back-to-back. buf must hold n_words * 7 bytes.
 *        Each word: [tag][xL][xH][yL][yH][zL][zH].
 */
HAL_StatusTypeDef LSM6DSOX_FIFO_ReadBlock(uint8_t *buf, uint16_t n_words);

/**
 * @brief Get current accelerometer sensitivity (mg/LSB) — for converting raw
 *        FIFO ACC samples after reading them.
 */
float LSM6DSOX_GetAccSensitivity(void);

/**
 * @brief Get current gyroscope sensitivity (mdps/LSB).
 */
float LSM6DSOX_GetGyroSensitivity(void);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DSOX_H__ */
