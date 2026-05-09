#include "key_adc.h"
#include "main.h"

#include <stdio.h>
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_adc.h"
#include "stm32u5xx_hal_adc_ex.h"
#include "stm32u5xx_hal_rcc_ex.h"
#include "stm32u5xx_ll_adc.h"

#define KEY_ADC_FULL_SCALE_RAW            4095U
#define KEY_ADC_VREF_MV                   3300U
#define KEY_ADC_SAMPLE_COUNT              8U
#define KEY_ADC_VBUS_ENTER_MIN_MV         2150U
#define KEY_ADC_VBUS_ENTER_MAX_MV         2550U
#define KEY_ADC_VBUS_EXIT_MIN_MV          2050U
#define KEY_ADC_VBUS_EXIT_MAX_MV          2650U
#define KEY_ADC_CONFIRM_COUNT             3U
#define KEY_ADC_POLL_TIMEOUT_MS           10U

static ADC_HandleTypeDef s_key_adc_handle;
static uint16_t s_key_adc_raw;
static uint32_t s_key_adc_mv;
static uint8_t s_key_adc_vbus_present;
static uint8_t s_key_adc_enter_count;
static uint8_t s_key_adc_exit_count;
static uint8_t s_key_adc_initialized;

static uint8_t KeyAdc_IsWithinWindow(uint32_t mv, uint32_t min_mv, uint32_t max_mv)
{
  return ((mv >= min_mv) && (mv <= max_mv)) ? 1U : 0U;
}

/*
static void KeyAdc_ConfigureClock(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
  PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_SYSCLK;
  (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  RCC->CCIPR3 = (RCC->CCIPR3 & ~0x3UL) | 0x3UL;
}

static void KeyAdc_ConfigureGpio(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = KEY_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_ADC_GPIO_Port, &GPIO_InitStruct);
}

static void KeyAdc_ConfigureAdc(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  __HAL_RCC_ADC12_CLK_ENABLE();

  s_key_adc_handle.Instance = ADC1;
  s_key_adc_handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  s_key_adc_handle.Init.Resolution = ADC_RESOLUTION_12B;
  s_key_adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  s_key_adc_handle.Init.ScanConvMode = ADC_SCAN_DISABLE;
  s_key_adc_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  s_key_adc_handle.Init.ContinuousConvMode = DISABLE;
  s_key_adc_handle.Init.NbrOfConversion = 1U;
  s_key_adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  s_key_adc_handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init(&s_key_adc_handle) != HAL_OK)
  {
    return;
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_391CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0U;

  (void)HAL_ADC_ConfigChannel(&s_key_adc_handle, &sConfig);
}
*/

static uint16_t KeyAdc_ReadOnce(void)
{
  uint32_t raw;

  if (HAL_ADC_Start(&s_key_adc_handle) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&s_key_adc_handle, KEY_ADC_POLL_TIMEOUT_MS) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&s_key_adc_handle);
    return 0U;
  }

  raw = HAL_ADC_GetValue(&s_key_adc_handle);
  (void)HAL_ADC_Stop(&s_key_adc_handle);

  return (uint16_t)raw;
}

void KeyAdc_Init(void)
{
  /* ADC硬件问题，暂时禁用初始化 */
  s_key_adc_raw = 0U;
  s_key_adc_mv = 0U;
  s_key_adc_vbus_present = 0U;
  s_key_adc_enter_count = 0U;
  s_key_adc_exit_count = 0U;
  s_key_adc_initialized = 0U;
}

void KeyAdc_Poll(void)
{
  uint32_t sum = 0U;
  uint32_t i;
  uint32_t mv;
  uint8_t in_window;

  if (s_key_adc_initialized == 0U)
  {
    return;
  }

  for (i = 0U; i < KEY_ADC_SAMPLE_COUNT; i++)
  {
    sum += KeyAdc_ReadOnce();
  }

  s_key_adc_raw = (uint16_t)(sum / KEY_ADC_SAMPLE_COUNT);
  mv = ((uint32_t)s_key_adc_raw * KEY_ADC_VREF_MV + (KEY_ADC_FULL_SCALE_RAW / 2U)) / KEY_ADC_FULL_SCALE_RAW;
  s_key_adc_mv = mv;

  if (s_key_adc_vbus_present == 0U)
  {
    in_window = KeyAdc_IsWithinWindow(mv,
                                      KEY_ADC_VBUS_ENTER_MIN_MV,
                                      KEY_ADC_VBUS_ENTER_MAX_MV);
    if (in_window != 0U)
    {
      if (s_key_adc_enter_count < KEY_ADC_CONFIRM_COUNT)
      {
        s_key_adc_enter_count++;
      }
      if (s_key_adc_enter_count >= KEY_ADC_CONFIRM_COUNT)
      {
        s_key_adc_vbus_present = 1U;
        s_key_adc_exit_count = 0U;
      }
    }
    else
    {
      s_key_adc_enter_count = 0U;
    }
  }
  else
  {
    in_window = KeyAdc_IsWithinWindow(mv,
                                      KEY_ADC_VBUS_EXIT_MIN_MV,
                                      KEY_ADC_VBUS_EXIT_MAX_MV);
    if (in_window == 0U)
    {
      if (s_key_adc_exit_count < KEY_ADC_CONFIRM_COUNT)
      {
        s_key_adc_exit_count++;
      }
      if (s_key_adc_exit_count >= KEY_ADC_CONFIRM_COUNT)
      {
        s_key_adc_vbus_present = 0U;
        s_key_adc_enter_count = 0U;
      }
    }
    else
    {
      s_key_adc_exit_count = 0U;
    }
  }
}

uint16_t KeyAdc_GetRaw(void)
{
  return s_key_adc_raw;
}

uint32_t KeyAdc_GetMillivolt(void)
{
  return s_key_adc_mv;
}

uint8_t KeyAdc_IsVbusPresent(void)
{
  return s_key_adc_vbus_present;
}
