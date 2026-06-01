/**
  ******************************************************************************
  * @file    h3lis100dl.h
  * @brief   H3LIS100DL 100g high-g accelerometer driver (SPI2)
  *
  *   Wiring (STM32U575RIT6 LQFP64):
 *     SCK  = PB10 (AF5, SPI2_SCK)
 *     MISO = PC2  (AF5, SPI2_MISO)  SDO/SA0 pin on sensor
 *     MOSI = PC1  (AF5, SPI2_MOSI)  SDA/SDI pin on sensor
 *     CS   = PC5 (GPIO push-pull, active low)
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

#include "main.h"      /* for IRQn enum (EXTI4_IRQn etc.) */
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
#define H3LIS100DL_REG_STATUS          0x27U
#define H3LIS100DL_REG_OUT_X           0x29U
#define H3LIS100DL_REG_OUT_Y           0x2BU
#define H3LIS100DL_REG_OUT_Z           0x2DU

/* CTRL_REG1 axis enable */
#define H3LIS100DL_CR1_XYZ_EN          0x07U

/* �̶���100g�����µ������ȣ�8λ�з�������� */
#define H3LIS100DL_SENSITIVITY_MG      780.0f

/* ======================== INT pin wiring (PCB-specific) ====================
 * Schematic: H3LIS100DL_INT1 -> PB4, INT2 -> PB5. CTRL_REG3 (0x22) routes
 * data-ready to INT1 when bits[1:0]=01 (= 0x01). The pin is active-high
 * push-pull by default. Switching to INT2 / a different GPIO only requires
 * editing this block. */
#define H3LIS100DL_INT_PIN          GPIO_PIN_4
#define H3LIS100DL_INT_GPIO_PORT    GPIOB
#define H3LIS100DL_INT_EXTI_IRQn    EXTI4_IRQn
#define H3LIS100DL_CTRL_REG3_DRDY_INT1   0x02U  /* I1_CFG[1:0]=10 → DRDY on INT1 (datasheet Table 27) */

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

#ifdef __cplusplus
}
#endif

#endif /* __H3LIS100DL_H__ */
