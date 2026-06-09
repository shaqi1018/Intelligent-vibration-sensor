/**
  ******************************************************************************
  * @file    acq_config.h
  * @brief   采集系统运行时配置（线程安全）
  *
  *          管理：采样率、数据出口（USB/SD 双通道并存）、SD 存储模式
  *          （线性追加 / 环形覆盖）、触发模式（无 / 外部 GPIO / 定时）、
  *          采集时长，以及三颗传感器各自的量程 / ODR。
  ******************************************************************************
  */

#ifndef __ACQ_CONFIG_H__
#define __ACQ_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* AcqSensorCfg_t 仅使用 uint16_t 字段，不依赖 BSP 枚举类型；
 * range/range2 存物理量（g 值 / dps），由 app_freertos 的 AppLsmXlRangeToReg 等
 * 辅助函数换算为寄存器编码后写入驱动。                                          */

/* sink mask（可同时启用多个） */
#define ACQ_SINK_NONE                 0x00U
#define ACQ_SINK_USB                  0x01U
#define ACQ_SINK_SD                   0x02U
#define ACQ_SINK_BOTH                 (ACQ_SINK_USB | ACQ_SINK_SD)

/* SD 存储模式 */
typedef enum {
  ACQ_STORAGE_LINEAR = 0,   /* 顺序追加，写满即停 */
  ACQ_STORAGE_RING   = 1    /* 循环覆盖，达到上限后从头覆写 */
} AcqStorageMode_t;

/* 触发模式 */
typedef enum {
  ACQ_TRIGGER_NONE     = 0, /* 立即开始 */
  ACQ_TRIGGER_EXTERNAL = 1, /* 等待外部 GPIO 边沿 */
  ACQ_TRIGGER_TIMER    = 2  /* 等待 trigger_delay_ms 后开始 */
} AcqTriggerMode_t;

/* 单个传感器配置 */
typedef struct {
  uint8_t  enabled;         /* 1=参与采集                                  */
  uint16_t range;           /* XL/加速度物理量程（g值：2/4/8/16/100）     */
  uint16_t range2;          /* 陀螺仪量程（dps：250/500/1000/2000）        */
                            /* 仅 LSM6DSOX 有效；其他传感器设 0            */
  uint16_t odr_hz;          /* 期望 ODR Hz（驱动选最接近的支持挡位）      */
} AcqSensorCfg_t;

/* 麦克风（ES8311）配置 */
typedef struct {
  uint8_t  enabled;         /* 1 = 参与采集 */
  uint32_t sample_rate_hz;  /* 8000 / 16000 / 48000 */
  uint16_t bits;            /* 固定 16 */
  uint16_t gain_db;         /* 0..42 */
} AcqMicCfg_t;

typedef struct {
  uint32_t sample_rate_hz;             /* 1..10000 */
  uint8_t  sink_mask;                  /* USB / SD / BOTH */
  AcqStorageMode_t storage_mode;
  AcqTriggerMode_t trigger_mode;
  uint32_t trigger_delay_ms;           /* TIMER 触发时延迟，外部触发为去抖 */
  uint32_t duration_ms;                /* 0 = 无限期；>0 自动停止 */
  uint32_t sd_ring_max_bytes;          /* RING 模式时环形区大小（字节，0 = 默认 64MB） */

  AcqSensorCfg_t lsm6dsox;
  AcqSensorCfg_t h3lis100dl;
  AcqSensorCfg_t qma6100p;
  AcqMicCfg_t    es8311;
} AcqConfig_t;

typedef enum {
  ACQ_STATE_IDLE      = 0,
  ACQ_STATE_ARMED     = 1,   /* 等待触发 */
  ACQ_STATE_RUNNING   = 2,
  ACQ_STATE_STOPPING  = 3
} AcqState_t;

/**
 * @brief 初始化为缺省配置（1KHz / USB / 线性 / 无触发 / 三传感器全开）。
 *        必须在 RTOS 启动后、第一次访问前调用一次。
 */
void AcqConfig_Init(void);

/**
 * @brief 拷贝当前配置（线程安全）。
 */
void AcqConfig_GetCopy(AcqConfig_t *out);

/**
 * @brief 整体替换配置（线程安全，会校验范围）。
 *        若采集正在运行只接受 sample_rate / duration 的更新。
 * @return 0=成功；非0=拒绝
 */
int  AcqConfig_Set(const AcqConfig_t *cfg);

/* 单字段快速 setter，便于 USB 命令行直接调用 */
int  AcqConfig_SetSampleRate(uint32_t hz);
int  AcqConfig_SetSinkMask(uint8_t mask);
int  AcqConfig_SetStorageMode(AcqStorageMode_t mode);
int  AcqConfig_SetTrigger(AcqTriggerMode_t mode, uint32_t arg_ms);
int  AcqConfig_SetDuration(uint32_t duration_ms);
int  AcqConfig_SetRingSize(uint32_t bytes);
int  AcqConfig_SetSensor(uint8_t which /*0=lsm,1=h3,2=qma*/, uint8_t enabled, uint16_t range, uint16_t odr_hz);

/**
 * @brief 设置麦克风（ES8311）配置：使能 / 采样率 / 增益。
 *        采样率仅接受 8000/16000/48000；gain_db ≤ 42；bits 固定 16。
 *        sample_rate_hz 传 0 表示保持不变。
 * @return 0=成功；非0=参数非法
 */
int  AcqConfig_SetMic(uint8_t enabled, uint32_t sample_rate_hz, uint16_t gain_db);

/**
 * @brief 扩展版 SetSensor：同时设置 range2（陀螺仪量程 dps，仅 LSM6DSOX 有效）。
 *        range  = XL 量程（g 值：2/4/8/16）；range2 = 陀螺仪量程（dps：250/500/1000/2000）。
 *        传 0 表示保持不变。
 */
int  AcqConfig_SetSensorFull(uint8_t which, uint8_t enabled,
                              uint16_t range, uint16_t range2, uint16_t odr_hz);

/* 状态机：仅 sensorAcqTask 调用 */
AcqState_t AcqConfig_GetState(void);
void       AcqConfig_SetState(AcqState_t st);

/* 帧序号：由 sensorAcqTask 递增；start 时归零 */
uint32_t AcqConfig_NextFrameId(void);   /* 递增后返回（每帧调用一次）   */
void     AcqConfig_ResetFrameId(void);

/**
 * @brief 启动采集。
 *        会切换状态到 ARMED 或 RUNNING 并重置 frame_id。
 *        采集任务自身判定状态决定是否开始读取传感器。
 */
int AcqConfig_Start(void);

/**
 * @brief 停止采集。立即把状态切到 STOPPING（任务读到后清理收尾，回到 IDLE）。
 */
int AcqConfig_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __ACQ_CONFIG_H__ */
