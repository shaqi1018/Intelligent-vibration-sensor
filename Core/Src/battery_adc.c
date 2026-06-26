#include "battery_adc.h"
#include "board_io.h"
#include "acq_config.h"
#include "stm32u5xx_hal.h"

/* 电池电压 ADC 配置
 * - PC0 = ADC1_IN1
 * - 12 位 ADC，参考电压 3.3V
 * - 分压电阻 R41=R42=100K，1:1 分压，VBAT_ADC = VBAT / 2
 */

#define BAT_ADC_FULL_SCALE_RAW      4095U
#define BAT_ADC_VREF_MV             3300U
#define BAT_ADC_SAMPLE_COUNT        16U
#define BAT_ADC_POLL_TIMEOUT_MS     20U

/* R41=R42=100K，1:1 分压，实际电池电压 = ADC电压 × 2 */
#define BAT_VOLTAGE_DIVIDER_RATIO   2U

/* 电池放完电时的截止电压（固定，锂电池保护板通常 3.0V 截止） */
#define BAT_VOLTAGE_EMPTY_MV        3000U

/* 各档位相对满电电压的偏移量（mV），基于典型锂电池放电曲线
 * 100% = full_mv
 *  80% = full_mv - 200
 *  60% = full_mv - 350
 *  40% = full_mv - 450
 *  20% = full_mv - 600
 *   0% = 3000 mV（固定截止电压）
 */
#define BAT_OFFSET_80_MV            200U
#define BAT_OFFSET_60_MV            350U
#define BAT_OFFSET_40_MV            450U
#define BAT_OFFSET_20_MV            600U

static ADC_HandleTypeDef s_bat_adc;
static uint8_t s_initialized = 0U;

/* ADC1_IN1 = PC0 的 GPIO 和 ADC 初始化 */
void BatteryADC_Init(void)
{
  if (s_initialized != 0U) {
    return;
  }

  /* VddA 必须在 ADC 使用前使能 */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddA();

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();

  /* PC0 配置为模拟输入 */
  GPIO_InitTypeDef g = {0};
  g.Pin  = BOARD_BAT_ADC_PIN;
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_BAT_ADC_PORT, &g);

  /* ADC1 配置 */
  s_bat_adc.Instance                   = ADC1;
  s_bat_adc.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
  s_bat_adc.Init.Resolution            = ADC_RESOLUTION_12B;
  s_bat_adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  s_bat_adc.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  s_bat_adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  s_bat_adc.Init.ContinuousConvMode    = DISABLE;
  s_bat_adc.Init.NbrOfConversion       = 1U;
  s_bat_adc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  s_bat_adc.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init(&s_bat_adc) != HAL_OK) {
    return;
  }

  /* 运行前校准，提升精度（STM32U5 建议每次 Init 后都执行）*/
  (void)HAL_ADCEx_Calibration_Start(&s_bat_adc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  /* 配置通道：PC0 = ADC1_IN1 */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_391CYCLES_5;  /* 慢速采样，100K源阻抗需要长采样时间 */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0U;

  (void)HAL_ADC_ConfigChannel(&s_bat_adc, &sConfig);

  s_initialized = 1U;
}

void BatteryADC_DeInit(void)
{
  if (s_initialized == 0U) {
    return;
  }
  HAL_ADC_DeInit(&s_bat_adc);
  s_initialized = 0U;
}

/* 读取单次 ADC 原始值 */
static uint16_t BatteryADC_ReadRawSingle(void)
{
  uint16_t raw = 0U;

  if (HAL_ADC_Start(&s_bat_adc) != HAL_OK) {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&s_bat_adc, BAT_ADC_POLL_TIMEOUT_MS) == HAL_OK) {
    raw = (uint16_t)HAL_ADC_GetValue(&s_bat_adc);
  }

  (void)HAL_ADC_Stop(&s_bat_adc);
  return raw;
}

/* 多次采样取平均，返回 ADC 原始值 (0~4095) */
uint16_t BatteryADC_ReadRaw(void)
{
  if (s_initialized == 0U) {
    BatteryADC_Init();
  }

  uint32_t sum = 0U;
  for (uint8_t i = 0U; i < BAT_ADC_SAMPLE_COUNT; i++) {
    sum += (uint32_t)BatteryADC_ReadRawSingle();
  }

  return (uint16_t)(sum / BAT_ADC_SAMPLE_COUNT);
}

/* 读取电池电压（单位：mV），已校正分压比 */
uint32_t BatteryADC_ReadMillivolts(void)
{
  uint16_t raw = BatteryADC_ReadRaw();

  /* ADC 测量电压 = raw × VREF / 4095 */
  uint32_t adc_mv = ((uint32_t)raw * BAT_ADC_VREF_MV + (BAT_ADC_FULL_SCALE_RAW / 2U))
                    / BAT_ADC_FULL_SCALE_RAW;

  /* 实际电池电压 = ADC 电压 × 分压比 */
  uint32_t battery_mv = adc_mv * BAT_VOLTAGE_DIVIDER_RATIO;

  return battery_mv;
}

/* 根据电池电压计算电量百分比 (0~100)
 * 满电电压从 DEVCFG.JSN 的 battery.full_mv 读取，其他档位按偏移量推算 */
uint8_t BatteryADC_GetPercentage(void)
{
  uint32_t mv = BatteryADC_ReadMillivolts();

  /* 从运行时配置读取满电电压 */
  AcqConfig_t cfg;
  AcqConfig_GetCopy(&cfg);
  uint32_t full_mv = (cfg.bat_full_mv >= 3500U && cfg.bat_full_mv <= 4400U)
                     ? cfg.bat_full_mv : 4200U;

  /* 各档位阈值由满电电压 + 固定偏移量推算 */
  uint32_t v100 = full_mv;
  uint32_t v80  = full_mv - BAT_OFFSET_80_MV;
  uint32_t v60  = full_mv - BAT_OFFSET_60_MV;
  uint32_t v40  = full_mv - BAT_OFFSET_40_MV;
  uint32_t v20  = full_mv - BAT_OFFSET_20_MV;
  uint32_t v0   = BAT_VOLTAGE_EMPTY_MV;

  /* 分段线性插值 */
  if (mv >= v100) { return 100U; }
  if (mv >= v80)  { return (uint8_t)(80U + (mv - v80) * 20U / (v100 - v80)); }
  if (mv >= v60)  { return (uint8_t)(60U + (mv - v60) * 20U / (v80  - v60)); }
  if (mv >= v40)  { return (uint8_t)(40U + (mv - v40) * 20U / (v60  - v40)); }
  if (mv >= v20)  { return (uint8_t)(20U + (mv - v20) * 20U / (v40  - v20)); }
  if (mv >= v0)   { return (uint8_t)((mv - v0) * 20U / (v20 - v0)); }

  return 0U;
}
