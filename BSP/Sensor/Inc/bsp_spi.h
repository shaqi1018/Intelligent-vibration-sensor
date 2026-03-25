/**
  ******************************************************************************
  * @file    bsp_spi.h
  * @brief   SPI bus and chip-select pin definitions for sensors
  *
  *   SPI1 (LSM6DSOX dedicated):
  *     SCK  = PA5 (AF5)   MISO = PA6 (AF5)   MOSI = PA7 (AF5)
  *     CS   = PA4 (GPIO push-pull, active low)
  *
  *   SPI2 (H3LIS100DL + QMA6100P shared bus):
  *     SCK  = PB13 (AF5)  MISO = PB14 (AF5)  MOSI = PB15 (AF5)
  *     CS_H3  = PB12 (GPIO, active low)
  *     CS_QMA = PB0 (GPIO, active low)
  *
  *   All buses: Master, Full-Duplex, 8-bit, MSB-first, Mode3 (CPOL=1,CPHA=1)
  *
  *   LSM6DSOX DIL24 wiring:
  *     VDD(Pin1)->3.3V  VDD_IO(Pin2)->3.3V  GND(Pin5)->GND
  *     CS(Pin14)->PA4   SCL/SPC(Pin9)->PA5   SDA/SDI(Pin11)->PA7
  *     SDO/SA0(Pin12)->PA6   INT1(Pin8)->PE0(opt)  INT2(Pin7)->PE1(opt)
  ******************************************************************************
  */
#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================== SPI1 (LSM6DSOX) ========================= */
#define LSM_SPI_INSTANCE         SPI1
#define LSM_SPI_SCK_PIN          GPIO_PIN_5        /* PA5 = SPI1_SCK (AF5) */
#define LSM_SPI_SCK_GPIO_PORT    GPIOA
#define LSM_SPI_SCK_AF           GPIO_AF5_SPI1
#define LSM_SPI_MISO_PIN         GPIO_PIN_6
#define LSM_SPI_MISO_GPIO_PORT   GPIOA
#define LSM_SPI_MISO_AF          GPIO_AF5_SPI1
#define LSM_SPI_MOSI_PIN         GPIO_PIN_7
#define LSM_SPI_MOSI_GPIO_PORT   GPIOA
#define LSM_SPI_MOSI_AF          GPIO_AF5_SPI1
#define LSM_SPI_CS_PIN           GPIO_PIN_4
#define LSM_SPI_CS_GPIO_PORT     GPIOA

#define LSM_SPI_CS_LOW()         HAL_GPIO_WritePin(LSM_SPI_CS_GPIO_PORT, LSM_SPI_CS_PIN, GPIO_PIN_RESET)
#define LSM_SPI_CS_HIGH()        HAL_GPIO_WritePin(LSM_SPI_CS_GPIO_PORT, LSM_SPI_CS_PIN, GPIO_PIN_SET)

/* ============================== SPI2 (H3LIS+QMA) ======================== */
/* SPI2 bus pins:
 *   SCK  = PB13 (AF5, SPI2_SCK)
 *   MISO = PB14 (AF5, SPI2_MISO)
 *   MOSI = PB15 (AF5, SPI2_MOSI)
 *   CS_H3  = PB12 (GPIO push-pull, active low)
 *   CS_QMA = PB0 (GPIO push-pull, active low)
 */
#define SENSOR_SPI2_INSTANCE     SPI2
#define SENSOR_SPI2_SCK_PIN      GPIO_PIN_13
#define SENSOR_SPI2_SCK_PORT     GPIOB
#define SENSOR_SPI2_SCK_AF       GPIO_AF5_SPI2
#define SENSOR_SPI2_MISO_PIN     GPIO_PIN_14
#define SENSOR_SPI2_MISO_PORT    GPIOB
#define SENSOR_SPI2_MISO_AF      GPIO_AF5_SPI2
#define SENSOR_SPI2_MOSI_PIN     GPIO_PIN_15
#define SENSOR_SPI2_MOSI_PORT    GPIOB
#define SENSOR_SPI2_MOSI_AF      GPIO_AF5_SPI2

#define H3_SPI2_CS_PIN           GPIO_PIN_12
#define H3_SPI2_CS_GPIO_PORT     GPIOB
#define QMA_SPI2_CS_PIN          GPIO_PIN_0
#define QMA_SPI2_CS_GPIO_PORT    GPIOB

#define H3_SPI2_CS_LOW()         HAL_GPIO_WritePin(H3_SPI2_CS_GPIO_PORT, H3_SPI2_CS_PIN, GPIO_PIN_RESET)
#define H3_SPI2_CS_HIGH()        HAL_GPIO_WritePin(H3_SPI2_CS_GPIO_PORT, H3_SPI2_CS_PIN, GPIO_PIN_SET)
#define QMA_SPI2_CS_LOW()        HAL_GPIO_WritePin(QMA_SPI2_CS_GPIO_PORT, QMA_SPI2_CS_PIN, GPIO_PIN_RESET)
#define QMA_SPI2_CS_HIGH()       HAL_GPIO_WritePin(QMA_SPI2_CS_GPIO_PORT, QMA_SPI2_CS_PIN, GPIO_PIN_SET)

/* ============================== Common =================================== */
#define LSM_SPI_TIMEOUT_MS       10U
#define SENSOR_SPI2_TIMEOUT_MS   10U

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H__ */

