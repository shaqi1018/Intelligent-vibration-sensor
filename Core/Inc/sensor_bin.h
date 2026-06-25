/**
  ******************************************************************************
  * @file    sensor_bin.h
  * @brief   RAW 二进制帧格式定义(用于 SD 卡 BIN 模式)
  *
  *          每个传感器的二进制帧格式:
  *          - frame_id: 4 字节,递增序号
  *          - timestamp_us: 8 字节,微秒时间戳
  *          - 原始数据: 传感器相关的 int16_t 数组
  *          - crc32: 4 字节,整帧 CRC32 校验(不含 crc32 字段本身)
  *
  *          所有字段采用小端序(Little Endian)存储。
  ******************************************************************************
  */

#ifndef __SENSOR_BIN_H__
#define __SENSOR_BIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* LSM6DSOX IMU 二进制帧: ACC(x,y,z) + GYR(x,y,z) */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  uint64_t timestamp_us;
  int16_t  acc_x;
  int16_t  acc_y;
  int16_t  acc_z;
  int16_t  gyr_x;
  int16_t  gyr_y;
  int16_t  gyr_z;
  uint32_t crc32;
} SensorBinFrame_LSM_t;

/* H3LIS100DL 高g加速度二进制帧: ACC(x,y,z) */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  uint64_t timestamp_us;
  int16_t  acc_x;
  int16_t  acc_y;
  int16_t  acc_z;
  uint32_t crc32;
} SensorBinFrame_H3_t;

/* QMA6100P 加速度二进制帧: ACC(x,y,z) */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  uint64_t timestamp_us;
  int16_t  acc_x;
  int16_t  acc_y;
  int16_t  acc_z;
  uint32_t crc32;
} SensorBinFrame_QMA_t;

/* AHT20 温湿度二进制帧: TEMP + HUMIDITY (保留原始 uint32_t,需解析) */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  uint64_t timestamp_us;
  uint32_t temp_raw;      /* AHT20 原始温度值 */
  uint32_t humidity_raw;  /* AHT20 原始湿度值 */
  uint32_t crc32;
} SensorBinFrame_AHT_t;

/* LIS2MDL 磁力计二进制帧: MAG(x,y,z) */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  uint64_t timestamp_us;
  int16_t  mag_x;
  int16_t  mag_y;
  int16_t  mag_z;
  uint32_t crc32;
} SensorBinFrame_MAG_t;

/* 通用二进制帧类型(用于 frame_ring.c 兼容,实际使用上面的具体类型) */
typedef SensorBinFrame_LSM_t SensorBinFrame_t;

/* CRC32 计算辅助函数声明(实现在 sensor_bin.c) */
uint32_t SensorBin_CalcCRC32(const void *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_BIN_H__ */
