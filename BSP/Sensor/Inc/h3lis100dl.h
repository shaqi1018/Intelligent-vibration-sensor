/**
  ******************************************************************************
  * @file    h3lis100dl.h
  * @brief   H3LIS100DL 100g high-g accelerometer driver (SPI2)
  *
  *   Wiring (STM32U575RIT6 LQFP64):
  *     SCK  = PB13 (AF5, SPI2_SCK)
  *     MISO = PB14 (AF5, SPI2_MISO)  SDO/SA0 pin on sensor
  *     MOSI = PB15 (AF5, SPI2_MOSI)  SDA/SDI pin on sensor
  *     CS   = PB12 (GPIO push-pull, active low)
  *     VDD  = 3.3V    GND = GND
  *
  *   H3LIS100DL SPI command byte format (per datasheet):
  *     bit7     : RW   (1=read, 0=write)
  *     bit6     : MS   (1=auto-increment addr, 0=single byte)
  *     bit[5:0] : AD5:AD0  (6-bit register address)
  ******************************************************************************
  */

#ifndef __H3LIS100DL_H__
#define __H3LIS100DL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_spi.h"
#include <stdint.h>
#include <stdio.h>

/* оƬ����ֵ */
#define H3LIS100DL_WHO_AM_I_VALUE     0x32U

/* �Ĵ�����ַӳ�� */
#define H3LIS100DL_REG_WHO_AM_I        0x0FU
#define H3LIS100DL_REG_CTRL_REG1       0x20U
#define H3LIS100DL_REG_CTRL_REG2       0x21U
#define H3LIS100DL_REG_CTRL_REG3       0x22U
#define H3LIS100DL_REG_CTRL_REG4       0x23U
#define H3LIS100DL_REG_CTRL_REG5       0x24U
#define H3LIS100DL_REG_HP_FILTER_RESET 0x25U
#define H3LIS100DL_REG_REFERENCE       0x26U
#define H3LIS100DL_REG_STATUS          0x27U
#define H3LIS100DL_REG_OUT_X           0x29U
#define H3LIS100DL_REG_OUT_Y           0x2BU
#define H3LIS100DL_REG_OUT_Z           0x2DU

#define H3LIS100DL_REG_INT1_CFG        0x30U
#define H3LIS100DL_REG_INT1_SRC        0x31U
#define H3LIS100DL_REG_INT1_THS        0x32U
#define H3LIS100DL_REG_INT1_DURATION   0x33U

#define H3LIS100DL_REG_INT2_CFG        0x34U
#define H3LIS100DL_REG_INT2_SRC        0x35U
#define H3LIS100DL_REG_INT2_THS        0x36U
#define H3LIS100DL_REG_INT2_DURATION   0x37U

/* STATUS_REG λ���� */
#define H3LIS100DL_STATUS_ZYXDA        0x08U
#define H3LIS100DL_STATUS_ZYXOR        0x80U

/* CTRL_REG1 ����ʹ��λ */
#define H3LIS100DL_CR1_XEN             0x01U
#define H3LIS100DL_CR1_YEN             0x02U
#define H3LIS100DL_CR1_ZEN             0x04U
#define H3LIS100DL_CR1_XYZ_EN          0x07U

/* �̶���100g�����µ������ȣ�8λ�з�������� */
#define H3LIS100DL_SENSITIVITY_MG      780.0f

/* SPI command byte helper (standard ST format: RW[7] | MS[6] | ADDR[5:0]) */
#define H3LIS100DL_SPI_RW_READ         0x80U
#define H3LIS100DL_SPI_RW_WRITE        0x00U
#define H3LIS100DL_SPI_MS              0x40U
#define H3LIS100DL_SPI_ADDR_MASK       0x3FU
#define H3LIS100DL_SPI_MAKE_CMD(reg, rw, ms) \
  (uint8_t)(((rw) ? H3LIS100DL_SPI_RW_READ : H3LIS100DL_SPI_RW_WRITE) | \
            ((ms) ? H3LIS100DL_SPI_MS : 0U) | \
            ((uint8_t)(reg) & H3LIS100DL_SPI_ADDR_MASK))

/* CTRL_REG1 ��Ӧ�� ODR + ��Դģʽ���� */
typedef enum {
  H3LIS100DL_ODR_OFF     = 0x00,
  H3LIS100DL_ODR_50HZ    = 0x01,
  H3LIS100DL_ODR_100HZ   = 0x11,
  H3LIS100DL_ODR_400HZ   = 0x21,
  H3LIS100DL_ODR_LP_05HZ = 0x02,
  H3LIS100DL_ODR_LP_1HZ  = 0x03,
  H3LIS100DL_ODR_LP_2HZ  = 0x04,
  H3LIS100DL_ODR_LP_5HZ  = 0x05,
  H3LIS100DL_ODR_LP_10HZ = 0x06
} H3LIS100DL_ODR_t;

typedef struct {
  H3LIS100DL_ODR_t odr;
} H3LIS100DL_Config_t;

typedef struct {
  int8_t raw[3];
  float acc_mg[3];
} H3LIS100DL_Data_t;

int H3LIS100DL_Init(void);
int H3LIS100DL_Configure(const H3LIS100DL_Config_t *config);
int H3LIS100DL_ReadAccXYZ(H3LIS100DL_Data_t *data);
int H3LIS100DL_ReadStatus(void);
int H3LIS100DL_ReadWhoAmI(void);
void H3LIS100DL_DumpRegs(void);

#ifdef __cplusplus
}
#endif

#endif /* __H3LIS100DL_H__ */
