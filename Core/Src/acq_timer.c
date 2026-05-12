/**
  ******************************************************************************
  * @file    acq_timer.c
  * @brief   TIM2 高速采样触发实现
  ******************************************************************************
  */

#include "acq_timer.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/* TIM2 输入时钟来源：APB1 timer clock。
 * 当 APB1 prescaler=1 时，TIMxCLK = HCLK = 160MHz；当不为1时为 2*PCLK1。
 * 本工程 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1，因此 TIM2CLK = 160MHz。 */
#define ACQ_TIMER_INPUT_CLK_HZ        160000000U

/* PSC 选择策略：
 *   - 高速档（>=1000Hz）：PSC = 0  -> CK_CNT = 160MHz, 1 tick = 6.25ns，10KHz 时 ARR=15999
 *   - 低速档（<1000Hz）：PSC = 159 -> CK_CNT = 1MHz,   1 tick = 1us
 * 这样在常用范围内都能取得整数 ARR。 */
#define ACQ_TIMER_HIGH_PSC            0U
#define ACQ_TIMER_HIGH_CK_HZ          ACQ_TIMER_INPUT_CLK_HZ
#define ACQ_TIMER_LOW_PSC             159U
#define ACQ_TIMER_LOW_CK_HZ           1000000U

TIM_HandleTypeDef htim2;

static volatile TaskHandle_t s_notify_task = NULL;
static volatile uint32_t s_tick_count = 0U;
static volatile uint32_t s_overflow_us = 0U;   /* 每次 update 增加的 us 数 */
static volatile uint64_t s_total_us = 0U;
static uint32_t s_effective_hz = 0U;
static uint32_t s_psc_cached = 0U;
static uint32_t s_arr_cached = 0U;

static void AcqTimer_Resolve(uint32_t sample_rate_hz, uint32_t *psc, uint32_t *arr, uint32_t *eff_hz)
{
  uint32_t psc_v;
  uint32_t ck_hz;
  uint32_t arr_v;
  uint64_t period_ticks;

  if (sample_rate_hz < ACQ_TIMER_HZ_MIN)
  {
    sample_rate_hz = ACQ_TIMER_HZ_MIN;
  }
  if (sample_rate_hz > ACQ_TIMER_HZ_MAX)
  {
    sample_rate_hz = ACQ_TIMER_HZ_MAX;
  }

  if (sample_rate_hz >= 1000U)
  {
    psc_v = ACQ_TIMER_HIGH_PSC;
    ck_hz = ACQ_TIMER_HIGH_CK_HZ;
  }
  else
  {
    psc_v = ACQ_TIMER_LOW_PSC;
    ck_hz = ACQ_TIMER_LOW_CK_HZ;
  }

  /* period_ticks = ck_hz / sample_rate_hz；ARR = period_ticks - 1 */
  period_ticks = (uint64_t)ck_hz / (uint64_t)sample_rate_hz;
  if (period_ticks == 0U)
  {
    period_ticks = 1U;
  }
  arr_v = (uint32_t)(period_ticks - 1ULL);

  *psc = psc_v;
  *arr = arr_v;
  *eff_hz = (uint32_t)((uint64_t)ck_hz / period_ticks);
}

static void AcqTimer_ApplyTiming(uint32_t psc, uint32_t arr)
{
  __HAL_TIM_DISABLE(&htim2);
  htim2.Init.Prescaler = psc;
  htim2.Init.Period = arr;
  __HAL_TIM_SET_PRESCALER(&htim2, psc);
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);

  s_psc_cached = psc;
  s_arr_cached = arr;
  s_overflow_us = (uint32_t)(((uint64_t)(arr + 1U) * 1000000ULL) /
                              ((psc == ACQ_TIMER_HIGH_PSC) ? ACQ_TIMER_HIGH_CK_HZ : ACQ_TIMER_LOW_CK_HZ));
}

HAL_StatusTypeDef AcqTimer_Init(void)
{
  TIM_ClockConfigTypeDef clk_cfg = {0};
  TIM_MasterConfigTypeDef master_cfg = {0};
  uint32_t psc = 0U, arr = 0U, eff = 0U;

  /* 时钟使能与 NVIC 优先级由 HAL_TIM_Base_MspInit 负责，
   * HAL_TIM_Base_Init 内部会自动调用，此处不再重复。 */

  AcqTimer_Resolve(ACQ_TIMER_HZ_DEFAULT, &psc, &arr, &eff);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = psc;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = arr;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  clk_cfg.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &clk_cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  master_cfg.MasterOutputTrigger = TIM_TRGO_RESET;
  master_cfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_cfg) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* IRQ 由 HAL_TIM_Base_Start_IT 启用，但 NVIC 优先级先行配置好。
   * 优先级要 >= configMAX_SYSCALL_INTERRUPT_PRIORITY (5)，否则 ISR 内不能调用 FromISR API */
  HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);

  AcqTimer_ApplyTiming(psc, arr);
  s_effective_hz = eff;
  s_tick_count = 0U;
  s_total_us = 0U;
  s_notify_task = NULL;

  printf("[AcqTimer] Init OK psc=%lu arr=%lu eff=%luHz overflow_us=%lu\r\n",
         (unsigned long)psc, (unsigned long)arr,
         (unsigned long)eff, (unsigned long)s_overflow_us);
  return HAL_OK;
}

HAL_StatusTypeDef AcqTimer_SetRate(uint32_t sample_rate_hz)
{
  uint32_t psc = 0U, arr = 0U, eff = 0U;

  if ((sample_rate_hz < ACQ_TIMER_HZ_MIN) || (sample_rate_hz > ACQ_TIMER_HZ_MAX))
  {
    return HAL_ERROR;
  }

  AcqTimer_Resolve(sample_rate_hz, &psc, &arr, &eff);
  AcqTimer_ApplyTiming(psc, arr);
  s_effective_hz = eff;
  return HAL_OK;
}

HAL_StatusTypeDef AcqTimer_Start(void)
{
  s_tick_count = 0U;
  s_total_us = 0U;
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  return HAL_TIM_Base_Start_IT(&htim2);
}

HAL_StatusTypeDef AcqTimer_Stop(void)
{
  HAL_StatusTypeDef st = HAL_TIM_Base_Stop_IT(&htim2);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  return st;
}

void AcqTimer_RegisterTask(void *task_handle)
{
  s_notify_task = (TaskHandle_t)task_handle;
}

uint32_t AcqTimer_GetEffectiveRate(void)
{
  return s_effective_hz;
}

void AcqTimer_GetTiming(uint32_t *psc, uint32_t *arr)
{
  if (psc != NULL) { *psc = s_psc_cached; }
  if (arr != NULL) { *arr = s_arr_cached; }
}

uint32_t AcqTimer_GetTickCount(void)
{
  return s_tick_count;
}

uint64_t AcqTimer_GetTimestampUs(void)
{
  uint64_t base;
  uint32_t cnt;
  uint32_t ck_hz;

  base = s_total_us;
  cnt = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
  ck_hz = (s_psc_cached == ACQ_TIMER_HIGH_PSC) ? ACQ_TIMER_HIGH_CK_HZ : ACQ_TIMER_LOW_CK_HZ;
  return base + ((uint64_t)cnt * 1000000ULL) / (uint64_t)ck_hz;
}

void AcqTimer_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

      s_tick_count++;
      s_total_us += s_overflow_us;

      if (s_notify_task != NULL)
      {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(s_notify_task, &hpw);
        portYIELD_FROM_ISR(hpw);
      }
    }
  }
}
