/**
  ******************************************************************************
  * @file    lis2mdl.h
  * @brief   LIS2MDLTR 三轴磁力计驱动（I2C1，7-bit 地址 0x1E）
  *
  *  连续模式 100Hz、高分辨率、温补开；DRDY 输出到 PC13(EXTI13)。
  *  灵敏度 1.5 mgauss/LSB。多字节读须子地址 MSB=1(auto-inc)。
  ******************************************************************************
  */
#ifndef __LIS2MDL_H__
#define __LIS2MDL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* DRDY 引脚（PCB：LIS2MDL_INT → PC13 / EXTI13） */
#define LIS2MDL_INT_PIN          GPIO_PIN_13
#define LIS2MDL_INT_GPIO_PORT    GPIOC
#define LIS2MDL_INT_EXTI_IRQn    EXTI13_IRQn

/* 初始化：校验 WHO_AM_I=0x40，按 odr_hz(10/20/50/100) 配连续模式。
 * 返回 HAL_OK=成功。 */
HAL_StatusTypeDef LIS2MDL_Init(uint16_t odr_hz);

/* 读三轴磁场。raw=原始 LSB，mag_mg=毫高斯(mG)。返回 HAL_OK=成功。 */
HAL_StatusTypeDef LIS2MDL_ReadMag(int16_t raw[3], float mag_mg[3]);

/* 查询 STATUS_REG 的 Zyxda(新数据就绪)。返回 1=就绪。 */
uint8_t LIS2MDL_DataReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIS2MDL_H__ */
