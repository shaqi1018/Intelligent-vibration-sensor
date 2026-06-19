/**
  ******************************************************************************
  * @file    acq_config.c
  * @brief   采集系统运行时配置实现（线程安全）
  ******************************************************************************
  */

#include "acq_config.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include <stdio.h>
#include <string.h>

#define ACQ_CFG_DEFAULT_RATE_HZ           1000U
#define ACQ_CFG_DEFAULT_RING_BYTES        (64U * 1024U * 1024U)  /* 64 MB */

#define ACQ_CFG_HZ_MIN                    1U
#define ACQ_CFG_HZ_MAX                    10000U

static osMutexId_t s_cfg_mutex = NULL;
static const osMutexAttr_t s_cfg_mutex_attr = {
  .name      = "acqCfgMutex",
  .attr_bits = osMutexPrioInherit
};

static AcqConfig_t s_cfg;
static volatile AcqState_t s_state = ACQ_STATE_IDLE;
static volatile uint32_t s_frame_id = 0U;

static void AcqCfgLoadDefaults(void)
{
  memset(&s_cfg, 0, sizeof(s_cfg));
  s_cfg.sample_rate_hz   = ACQ_CFG_DEFAULT_RATE_HZ;
  s_cfg.sink_mask        = ACQ_SINK_USB;
  s_cfg.storage_mode     = ACQ_STORAGE_LINEAR;
  s_cfg.trigger_mode     = ACQ_TRIGGER_NONE;
  s_cfg.trigger_delay_ms = 0U;
  s_cfg.duration_ms      = 0U;
  s_cfg.sd_ring_max_bytes = ACQ_CFG_DEFAULT_RING_BYTES;

  /* LSM6DSOX：±4g XL, ±2000dps 陀螺，1666Hz ODR
   * range / range2 存物理量（g 值 / dps），由 AppLsmXlRangeToReg 换算寄存器编码 */
  s_cfg.lsm6dsox.enabled = 1U;
  s_cfg.lsm6dsox.range   = 4U;      /* ±4 g   */
  s_cfg.lsm6dsox.range2  = 2000U;   /* ±2000 dps */
  s_cfg.lsm6dsox.odr_hz  = 1666U;

  /* H3LIS100DL：±100g 固定，ODR 最高 400Hz */
  s_cfg.h3lis100dl.enabled = 1U;
  s_cfg.h3lis100dl.range   = 100U;  /* 物理量：100g，固定 */
  s_cfg.h3lis100dl.range2  = 0U;
  s_cfg.h3lis100dl.odr_hz  = 400U;

  /* QMA6100P：±4g，BW=100Hz
   * range 存物理 g 值，由 AppQmaRangeToReg 换算 */
  s_cfg.qma6100p.enabled = 1U;
  s_cfg.qma6100p.range   = 4U;      /* ±4 g  */
  s_cfg.qma6100p.range2  = 0U;
  s_cfg.qma6100p.odr_hz  = 100U;

  /* ES8311 麦克风：默认关闭，由 DEVCFG 打开 */
  s_cfg.es8311.enabled        = 0U;
  s_cfg.es8311.sample_rate_hz = 16000U;
  s_cfg.es8311.bits           = 16U;
  s_cfg.es8311.gain_db        = 33U;

  /* AHT20 温湿度：默认开启，1Hz（手册要求采集周期≥1s） */
  s_cfg.aht20.enabled = 1U;
  s_cfg.aht20.range   = 0U;
  s_cfg.aht20.range2  = 0U;
  s_cfg.aht20.odr_hz  = 1U;

  /* LIS2MDL 三轴磁力：默认开启，100Hz，±50 gauss（固定量程，range 仅信息性） */
  s_cfg.lis2mdl.enabled = 1U;
  s_cfg.lis2mdl.range   = 50U;
  s_cfg.lis2mdl.range2  = 0U;
  s_cfg.lis2mdl.odr_hz  = 100U;
}

void AcqConfig_Init(void)
{
  if (s_cfg_mutex == NULL)
  {
    s_cfg_mutex = osMutexNew(&s_cfg_mutex_attr);
  }
  AcqCfgLoadDefaults();
  s_state    = ACQ_STATE_IDLE;
  s_frame_id = 0U;
}

static void AcqCfgLock(void)
{
  if (s_cfg_mutex != NULL)
  {
    osMutexAcquire(s_cfg_mutex, osWaitForever);
  }
}

static void AcqCfgUnlock(void)
{
  if (s_cfg_mutex != NULL)
  {
    osMutexRelease(s_cfg_mutex);
  }
}

void AcqConfig_GetCopy(AcqConfig_t *out)
{
  if (out == NULL)
  {
    return;
  }
  AcqCfgLock();
  *out = s_cfg;
  AcqCfgUnlock();
}

static int AcqCfgValidate(const AcqConfig_t *cfg)
{
  if (cfg == NULL)
  {
    return -1;
  }
  if ((cfg->sample_rate_hz < ACQ_CFG_HZ_MIN) || (cfg->sample_rate_hz > ACQ_CFG_HZ_MAX))
  {
    return -1;
  }
  if ((cfg->sink_mask & ~ACQ_SINK_BOTH) != 0U)
  {
    return -1;
  }
  if ((cfg->storage_mode != ACQ_STORAGE_LINEAR) && (cfg->storage_mode != ACQ_STORAGE_RING))
  {
    return -1;
  }
  if ((cfg->trigger_mode != ACQ_TRIGGER_NONE) &&
      (cfg->trigger_mode != ACQ_TRIGGER_EXTERNAL) &&
      (cfg->trigger_mode != ACQ_TRIGGER_TIMER))
  {
    return -1;
  }
  if (cfg->es8311.enabled)
  {
    uint32_t sr = cfg->es8311.sample_rate_hz;
    if ((sr != 8000U) && (sr != 16000U) && (sr != 48000U) && (sr != 96000U))
    {
      return -1;
    }
    if (cfg->es8311.bits != 16U)
    {
      return -1;
    }
    if (cfg->es8311.gain_db > 42U)
    {
      return -1;
    }
  }
  return 0;
}

int AcqConfig_Set(const AcqConfig_t *cfg)
{
  if (AcqCfgValidate(cfg) != 0)
  {
    return -1;
  }

  AcqCfgLock();
  if (s_state != ACQ_STATE_IDLE)
  {
    /* 运行中只允许局部更新 */
    s_cfg.sample_rate_hz = cfg->sample_rate_hz;
    s_cfg.duration_ms    = cfg->duration_ms;
  }
  else
  {
    s_cfg = *cfg;
  }
  AcqCfgUnlock();
  return 0;
}

int AcqConfig_SetSampleRate(uint32_t hz)
{
  if ((hz < ACQ_CFG_HZ_MIN) || (hz > ACQ_CFG_HZ_MAX))
  {
    return -1;
  }
  AcqCfgLock();
  s_cfg.sample_rate_hz = hz;
  AcqCfgUnlock();
  return 0;
}

int AcqConfig_SetSinkMask(uint8_t mask)
{
  if ((mask & ~ACQ_SINK_BOTH) != 0U)
  {
    return -1;
  }
  AcqCfgLock();
  if (s_state != ACQ_STATE_IDLE)
  {
    AcqCfgUnlock();
    return -2;  /* 运行中不允许切换 sink */
  }
  s_cfg.sink_mask = mask;
  AcqCfgUnlock();
  return 0;
}

int AcqConfig_SetStorageMode(AcqStorageMode_t mode)
{
  if ((mode != ACQ_STORAGE_LINEAR) && (mode != ACQ_STORAGE_RING))
  {
    return -1;
  }
  AcqCfgLock();
  if (s_state != ACQ_STATE_IDLE)
  {
    AcqCfgUnlock();
    return -2;
  }
  s_cfg.storage_mode = mode;
  AcqCfgUnlock();
  return 0;
}

int AcqConfig_SetTrigger(AcqTriggerMode_t mode, uint32_t arg_ms)
{
  if ((mode != ACQ_TRIGGER_NONE) && (mode != ACQ_TRIGGER_EXTERNAL) && (mode != ACQ_TRIGGER_TIMER))
  {
    return -1;
  }
  AcqCfgLock();
  if (s_state != ACQ_STATE_IDLE)
  {
    AcqCfgUnlock();
    return -2;
  }
  s_cfg.trigger_mode     = mode;
  s_cfg.trigger_delay_ms = arg_ms;
  AcqCfgUnlock();
  return 0;
}

int AcqConfig_SetDuration(uint32_t duration_ms)
{
  AcqCfgLock();
  s_cfg.duration_ms = duration_ms;
  AcqCfgUnlock();
  return 0;
}

int AcqConfig_SetRingSize(uint32_t bytes)
{
  if (bytes != 0U && bytes < (1U * 1024U * 1024U))
  {
    return -1;  /* 至少 1MB */
  }
  AcqCfgLock();
  if (s_state != ACQ_STATE_IDLE)
  {
    AcqCfgUnlock();
    return -2;
  }
  s_cfg.sd_ring_max_bytes = (bytes == 0U) ? ACQ_CFG_DEFAULT_RING_BYTES : bytes;
  AcqCfgUnlock();
  return 0;
}

int AcqConfig_SetSensor(uint8_t which, uint8_t enabled, uint16_t range, uint16_t odr_hz)
{
  AcqSensorCfg_t *target = NULL;

  AcqCfgLock();
  if (s_state != ACQ_STATE_IDLE)
  {
    AcqCfgUnlock();
    return -2;
  }
  switch (which)
  {
    case 0U: target = &s_cfg.lsm6dsox;   break;
    case 1U: target = &s_cfg.h3lis100dl; break;
    case 2U: target = &s_cfg.qma6100p;   break;
    case 3U: target = &s_cfg.aht20;      break;
    case 4U: target = &s_cfg.lis2mdl;    break;
    default:
      AcqCfgUnlock();
      return -1;
  }
  target->enabled = (enabled != 0U) ? 1U : 0U;
  if (range != 0U)  { target->range = range; }
  if (odr_hz != 0U) { target->odr_hz = odr_hz; }
  AcqCfgUnlock();
  return 0;
}

/**
 * @brief 扩展版：同时设置 range（XL量程）、range2（陀螺仪量程 dps）、odr_hz。
 *        range2 仅对 LSM6DSOX（which=0）有效；其他传感器忽略。
 */
int AcqConfig_SetSensorFull(uint8_t which, uint8_t enabled,
                             uint16_t range, uint16_t range2, uint16_t odr_hz)
{
  int ret = AcqConfig_SetSensor(which, enabled, range, odr_hz);
  if (ret != 0)  { return ret; }
  if (which == 0U && range2 != 0U)
  {
    AcqCfgLock();
    s_cfg.lsm6dsox.range2 = range2;
    AcqCfgUnlock();
  }
  return 0;
}

int AcqConfig_SetMic(uint8_t enabled, uint32_t sample_rate_hz, uint16_t gain_db)
{
  if (enabled)
  {
    /* sample_rate_hz == 0 means "keep current rate" (see below); only validate
     * when a concrete rate is supplied. */
    if ((sample_rate_hz != 0U) &&
        (sample_rate_hz != 8000U) && (sample_rate_hz != 16000U) &&
        (sample_rate_hz != 48000U) && (sample_rate_hz != 96000U))
    {
      return -1;
    }
    if (gain_db > 42U)
    {
      return -1;
    }
  }
  AcqCfgLock();
  s_cfg.es8311.enabled = (enabled != 0U) ? 1U : 0U;
  if (sample_rate_hz != 0U) { s_cfg.es8311.sample_rate_hz = sample_rate_hz; }
  s_cfg.es8311.bits    = 16U;
  s_cfg.es8311.gain_db = gain_db;
  AcqCfgUnlock();
  return 0;
}

AcqState_t AcqConfig_GetState(void)
{
  return s_state;
}

void AcqConfig_SetState(AcqState_t st)
{
  s_state = st;
}

uint32_t AcqConfig_NextFrameId(void)
{
  /* 这里 s_frame_id 仅由 sensorAcqTask 单线程递增，不必加锁 */
  s_frame_id++;
  return s_frame_id;
}

void AcqConfig_ResetFrameId(void)
{
  s_frame_id = 0U;
}

int AcqConfig_Start(void)
{
  AcqCfgLock();
  if (s_state != ACQ_STATE_IDLE)
  {
    AcqCfgUnlock();
    return -1;
  }
  /* 触发模式决定首态：TIMER/EXTERNAL 先进 ARMED 等触发；NONE 直接 RUNNING */
  s_state = (s_cfg.trigger_mode == ACQ_TRIGGER_NONE) ? ACQ_STATE_RUNNING : ACQ_STATE_ARMED;
  AcqCfgUnlock();
  s_frame_id = 0U;
  return 0;
}

int AcqConfig_Stop(void)
{
  AcqCfgLock();
  if (s_state == ACQ_STATE_IDLE)
  {
    AcqCfgUnlock();
    return -1;
  }
  s_state = ACQ_STATE_STOPPING;
  AcqCfgUnlock();
  return 0;
}
