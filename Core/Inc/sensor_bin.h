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

/* LSM6DSOX IMU 二进制帧(旧:acc+gyr 合并)。BIN 模式已改为 acc/gyr 拆两文件写
 * (见下 SensorBinFrame_LSMAcc_t / _LSMGyr_t),此合并结构不再写入 SD,仅保留给
 * frame_ring.c 的 SensorBinFrame_t 别名兼容。 */
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

/* LSM6DSOX 低量程加速度二进制帧(BIN 模式,拆分后写 ACC_LOW):ACC(x,y,z),14B。
 * ★2026-07-14 去掉逐帧 timestamp_us(8B):它是合成值 base_ts + frame_id×odr_interval,
 * 与 frame_id 严格线性绑定=纯冗余。PC 端从 CONFIG.JSN 的 base/interval + frame_id 完美还原,
 * 时间精度零损失。删它把 LSM ACC 产出从 147→93KB/s(两条链共省 108KB/s),直击带宽物理墙。 */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  int16_t  acc_x;
  int16_t  acc_y;
  int16_t  acc_z;
  uint32_t crc32;
} SensorBinFrame_LSMAcc_t;

/* LSM6DSOX 低量程角速度二进制帧(BIN 模式,拆分后写 GYR_LOW):GYR(x,y,z),14B。
 * 与 ACC 同 frame_id 对齐。★2026-07-14 同去 timestamp_us(见上)。 */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  int16_t  gyr_x;
  int16_t  gyr_y;
  int16_t  gyr_z;
  uint32_t crc32;
} SensorBinFrame_LSMGyr_t;

/* H3LIS100DL 高g加速度二进制帧: ACC(x,y,z),14B。
 * ★2026-07-14 去 timestamp_us(合成值,同 LSM,由 base+fid×interval 还原)。 */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
  int16_t  acc_x;
  int16_t  acc_y;
  int16_t  acc_z;
  uint32_t crc32;
} SensorBinFrame_H3_t;

/* QMA6100P 加速度二进制帧: ACC(x,y,z),14B。
 * ★2026-07-14 去 timestamp_us(合成值,同 LSM,由 base+fid×interval 还原)。 */
typedef struct __attribute__((packed)) {
  uint32_t frame_id;
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

/* LIS2MDL 磁力计二进制帧: MAG(x,y,z),22B。★2026-07-14 保留 timestamp_us:它是
 * AppTime_GetEpochUs() 真实时钟(非合成),不可由 frame_id 还原;且 100Hz 仅 2.2KB/s、
 * 掉帧本就 0%,无带宽压力。故不瘦身。 */
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
