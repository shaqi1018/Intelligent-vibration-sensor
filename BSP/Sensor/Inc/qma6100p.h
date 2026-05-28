/**
  ******************************************************************************
  * @file    qma6100p.h
  * @brief   QMA6100P ���ٶȼ������ӿڣ�SPI2��ˣ���
  *
  * �������е� SPI ���ˣ�
 *   - �������ߣ�SPI2��PB10 SCK, PC2 MISO, PC1 MOSI��
 *   - ����Ƭѡ��PA4���͵�ƽ��Ч��
  *
  * QMA6100P SPI �����ָ�ʽ��
  *   bit0     : RW   ��1=����0=д��
  *   bit[7:1] : �Ĵ�����ַ
  ******************************************************************************
  */

#ifndef __QMA6100P_H__
#define __QMA6100P_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"      /* for IRQn enum (EXTI15_IRQn etc.) */
#include "bsp_spi.h"
#include <stdint.h>
#include <stdio.h>

/* оƬ���ֵ��CHIP_ID ��4λӦΪ 0x09 */
#define QMA6100P_DEVICE_ID          0x09U

/* �Ĵ�����ַӳ�� */
#define QMA6100P_REG_CHIP_ID        0x00U
#define QMA6100P_REG_XOUTL          0x01U
#define QMA6100P_REG_XOUTH          0x02U
#define QMA6100P_REG_YOUTL          0x03U
#define QMA6100P_REG_YOUTH          0x04U
#define QMA6100P_REG_ZOUTL          0x05U
#define QMA6100P_REG_ZOUTH          0x06U
#define QMA6100P_REG_INT_STATUS_0   0x09U
#define QMA6100P_REG_INT_STATUS_1   0x0AU
#define QMA6100P_REG_INT_STATUS_2   0x0BU
#define QMA6100P_REG_INT_STATUS_3   0x0CU
#define QMA6100P_REG_RANGE          0x0FU
#define QMA6100P_REG_BW_ODR         0x10U
#define QMA6100P_REG_POWER_MANAGE   0x11U
#define QMA6100P_REG_NVM            0x33U
#define QMA6100P_REG_RESET          0x36U

/* SPI �����ָ����� */
#define QMA6100P_SPI_RW_WRITE       0x00U
#define QMA6100P_SPI_RW_READ        0x80U
#define QMA6100P_SPI_MAKE_CMD(reg, rw) \
  (uint8_t)(((uint8_t)(reg) & 0x7FU) | ((rw) ? QMA6100P_SPI_RW_READ : QMA6100P_SPI_RW_WRITE))

/* ����ѡ�� */
typedef enum {
  QMA6100P_RANGE_2G  = 0x01,
  QMA6100P_RANGE_4G  = 0x02,
  QMA6100P_RANGE_8G  = 0x04,
  QMA6100P_RANGE_16G = 0x08,
  QMA6100P_RANGE_32G = 0x0F
} QMA6100P_Range_t;

/* ���� / ODR ѡ�� */
typedef enum {
  QMA6100P_BW_100   = 0x00,
  QMA6100P_BW_200   = 0x01,
  QMA6100P_BW_400   = 0x02,
  QMA6100P_BW_800   = 0x03,
  QMA6100P_BW_1600  = 0x04,
  QMA6100P_BW_50    = 0x05,
  QMA6100P_BW_25    = 0x06,
  QMA6100P_BW_12_5  = 0x07
} QMA6100P_Bandwidth_t;

#define QMA6100P_DRDY_BIT           0x10U

/* ======================== FIFO registers =================================== */
#define QMA6100P_REG_INT_ST         0x0BU  /* bit7=FIFO_OR, bit6=FIFO_WM_INT, bit5=FIFO_FULL */
#define QMA6100P_REG_FIFO_CNT       0x0EU  /* FIFO_FRAME_COUNTER[7:0] */
#define QMA6100P_REG_INT_EN1        0x17U  /* bit6=INT_FWM_EN, bit5=INT_FFULL_EN */
#define QMA6100P_REG_INT_MAP1       0x1AU  /* bit6=INT1_FWM, bit5=INT1_FFULL */
#define QMA6100P_REG_INT_MAP2       0x1CU  /* bit6=INT2_FWM, bit5=INT2_FFULL */
#define QMA6100P_REG_INT_PIN_CFG    0x20U
#define QMA6100P_REG_INT_CFG        0x21U  /* bit0=LATCH_INT */
#define QMA6100P_REG_FIFO_WM        0x31U  /* watermark level [7:0] */
#define QMA6100P_REG_FIFO_CFG       0x3EU  /* bit[7:6]=FIFO_MODE, bit[2:0]=FIFO_CH */
#define QMA6100P_REG_FIFO_DATA      0x3FU

#define QMA6100P_FIFO_MODE_BYPASS   0x00U
#define QMA6100P_FIFO_MODE_FIFO     0x40U
#define QMA6100P_FIFO_MODE_STREAM   0x80U
#define QMA6100P_FIFO_CH_XYZ        0x07U

#define QMA6100P_INT_WM_BIT         0x40U

/* ======================== INT pin wiring (PCB-specific) =================== */
/* Current board: INT2 → PB15 works; INT1 → PB12 is non-functional. To swap
 * back to INT1 on a fixed board, change QMA6100P_INT_MAP_REG to INT_MAP1
 * and update QMA6100P_INT_PIN/IRQn to match the new GPIO. */
#define QMA6100P_INT_PIN            GPIO_PIN_15
#define QMA6100P_INT_GPIO_PORT      GPIOB
#define QMA6100P_INT_EXTI_IRQn      EXTI15_IRQn
#define QMA6100P_INT_MAP_REG        QMA6100P_REG_INT_MAP2

typedef struct {
  QMA6100P_Range_t range;
  QMA6100P_Bandwidth_t bw;
} QMA6100P_Config_t;

typedef struct {
  int16_t raw[3];
  float acc_mg[3];
} QMA6100P_Data_t;

HAL_StatusTypeDef QMA6100P_Init(void);
HAL_StatusTypeDef QMA6100P_Configure(const QMA6100P_Config_t *cfg);
HAL_StatusTypeDef QMA6100P_ReadRawXYZ(QMA6100P_Data_t *data);
HAL_StatusTypeDef QMA6100P_ReadAccXYZ(QMA6100P_Data_t *data);
HAL_StatusTypeDef QMA6100P_ReadChipID(uint8_t *id);
HAL_StatusTypeDef QMA6100P_ReadStatus(uint8_t *status);
void QMA6100P_DumpRegs(void);

/* ======================== FIFO API ========================================= */
HAL_StatusTypeDef QMA6100P_FIFO_Config(uint8_t wtm_samples);
HAL_StatusTypeDef QMA6100P_FIFO_Rearm(void);
HAL_StatusTypeDef QMA6100P_FIFO_GetLevel(uint8_t *level);
HAL_StatusTypeDef QMA6100P_FIFO_ReadBlock(uint8_t *buf, uint8_t n_frames);
uint16_t QMA6100P_GetLsbPer1g(void);

#ifdef __cplusplus
}
#endif

#endif /* __QMA6100P_H__ */
