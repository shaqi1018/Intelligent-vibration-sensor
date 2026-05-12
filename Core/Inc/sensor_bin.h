/**
  ******************************************************************************
  * @file    sensor_bin.h
  * @brief   传感器紧凑二进制帧定义（用于 USB / SD 高速数据流）
  *
  *          每帧固定 64 字节（4 字节对齐，8 字节边界 padding 友好），
  *          以一个 32-bit magic 起始，方便上位机做帧同步；尾部带 CRC32 校验。
  *
  *          字段顺序在协议中固定，禁止随意修改/重排。版本字段 version 用于
  *          后续兼容扩展。
  *
  *          原始数据：保存的是传感器寄存器读出的原始 16/8 位数值，
  *          不做任何降采样或滤波；上位机自行根据 range 字段换算物理量。
  ******************************************************************************
  */

#ifndef __SENSOR_BIN_H__
#define __SENSOR_BIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 帧同步 magic = 'SENS' (little-endian) */
#define SENSOR_BIN_MAGIC              0x534E4553UL
#define SENSOR_BIN_VERSION            0x01U

/* present_mask / enabled_mask 位定义 */
#define SENSOR_BIN_FLAG_LSM6DSOX      0x01U
#define SENSOR_BIN_FLAG_H3LIS100DL    0x02U
#define SENSOR_BIN_FLAG_QMA6100P      0x04U
#define SENSOR_BIN_FLAG_ALL           (SENSOR_BIN_FLAG_LSM6DSOX | \
                                       SENSOR_BIN_FLAG_H3LIS100DL | \
                                       SENSOR_BIN_FLAG_QMA6100P)

/* flags 字段（offset 07）位定义 */
#define SENSOR_BIN_FLAGS_TRIGGER      0x01U  /* 外部触发事件 */
#define SENSOR_BIN_FLAGS_H3_FRESH     0x02U  /* H3LIS100DL 本 tick 为新鲜采样 */
#define SENSOR_BIN_FLAGS_QMA_FRESH    0x04U  /* QMA6100P   本 tick 为新鲜采样 */

#define SENSOR_BIN_FRAME_BYTES        64U

#pragma pack(push, 1)
/**
 * 64 字节帧布局：
 *   off 00: u32 magic           = 'S','E','N','S' (0x534E4553)
 *   off 04: u8  version         = 0x01
 *   off 05: u8  enabled_mask    传感器使能位
 *   off 06: u8  present_mask    本帧实际带有效数据的位
 *   off 07: u8  flags           保留 / 触发标志（bit0=trigger_event）
 *   off 08: u32 frame_id        递增帧序号（从 1 开始）
 *   off 12: u32 sample_rate_hz  当前采样率（用于上位机确认）
 *   off 16: u64 tick_us         自采集启动以来的微秒数
 *   off 24: i16 lsm_acc[3]      LSM6DSOX 加速度原始 X/Y/Z（int16 LSB）
 *   off 30: i16 lsm_gyro[3]     LSM6DSOX 陀螺仪原始 X/Y/Z（int16 LSB）
 *   off 36: i16 lsm_temp        LSM6DSOX 温度原始（int16 LSB）
 *   off 38: u8  lsm_range_xl    XL 量程寄存器编码（LSM6DSOX_XL_FS_xxx，如 0x08=±4g）
 *   off 39: u8  lsm_range_g     G  量程寄存器编码（LSM6DSOX_G_FS_xxx， 如 0x0C=±2000dps）
 *   off 40: i8  h3_acc[3]       H3LIS100DL 加速度原始 X/Y/Z（int8 LSB, ±100g）
 *   off 43: u8  h3_range        固定 100g
 *   off 44: i16 qma_acc[3]      QMA6100P 加速度原始 X/Y/Z（int16 LSB）
 *   off 50: u8  qma_range       量程编码
 *   off 51: u8  reserved0
 *   off 52: u32 reserved1
 *   off 56: u32 reserved2
 *   off 60: u32 crc32           整帧前 60 字节的 CRC32（多项式 0xEDB88320，初值 0xFFFFFFFF，输出取反）
 */
typedef struct {
  uint32_t magic;
  uint8_t  version;
  uint8_t  enabled_mask;
  uint8_t  present_mask;
  uint8_t  flags;

  uint32_t frame_id;
  uint32_t sample_rate_hz;
  uint64_t tick_us;

  int16_t  lsm_acc[3];
  int16_t  lsm_gyro[3];
  int16_t  lsm_temp;
  uint8_t  lsm_range_xl;
  uint8_t  lsm_range_g;

  int8_t   h3_acc[3];
  uint8_t  h3_range;

  int16_t  qma_acc[3];
  uint8_t  qma_range;
  uint8_t  reserved0;

  uint32_t reserved1;
  uint32_t reserved2;

  uint32_t crc32;
} SensorBinFrame_t;
#pragma pack(pop)

/* 编译期校验：必须正好 64 字节 */
typedef char __sensor_bin_size_check[(sizeof(SensorBinFrame_t) == SENSOR_BIN_FRAME_BYTES) ? 1 : -1];

/**
 * 标准 CRC32（多项式 0xEDB88320，反射输入输出，最终取反）。
 * 用于帧校验，不依赖硬件 CRC 单元。
 */
uint32_t SensorBin_CRC32(const void *data, uint32_t len);

/**
 * 计算并写入帧的 CRC32 字段（基于前 60 字节）。
 */
void SensorBin_FillCRC(SensorBinFrame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_BIN_H__ */
