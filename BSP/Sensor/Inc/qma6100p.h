/**
  ******************************************************************************
  * @file    qma6100p.h
  * @brief   QMA6100P ���ٶȼ������ӿڣ�SPI2��ˣ���
  *
  * �������е� SPI ���ˣ�
  *   - �������ߣ�SPI2��PB13 SCK, PB14 MISO, PB15 MOSI��
  *   - ����Ƭѡ��PB0���͵�ƽ��Ч��
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

#ifdef __cplusplus
}
#endif

#endif /* __QMA6100P_H__ */
