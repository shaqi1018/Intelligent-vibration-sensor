#include "board_io.h"

static uint8_t s_battery_latched = 0U;

void BoardIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Power button (PC14) — init first so BoardIO_StartupLatch() can sample it */
  g.Pin  = BOARD_PWR_BTN_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_PWR_BTN_PORT, &g);

  /* User button (PC15) — input, hardware pull-up already on board */
  g.Pin = BOARD_USER_BTN_PIN;
  HAL_GPIO_Init(BOARD_USER_BTN_PORT, &g);

  /* USB_DET (PC7) — input; driven by 10K/20K VBUS divider, no internal pull */
  g.Pin  = BOARD_USB_DET_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_USB_DET_PORT, &g);

  /* POWER_CTL (PB7) — output, start LOW; latch decision in BoardIO_StartupLatch() */
  g.Pin   = BOARD_POWER_CTL_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_POWER_CTL_PORT, &g);
  HAL_GPIO_WritePin(BOARD_POWER_CTL_PORT, BOARD_POWER_CTL_PIN, GPIO_PIN_RESET);

  /* LED (PB12) — output, start off */
  g.Pin = BOARD_LED_PIN;
  HAL_GPIO_Init(BOARD_LED_PORT, &g);
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_RESET);

  /* PA_EN (PB15) — push-pull output. Active-LOW: default HIGH so the codec is
   * NOT powered (saves current, safe on boot). PaEn_Set(1) pulls it LOW. */
  HAL_GPIO_WritePin(BOARD_PA_EN_PORT, BOARD_PA_EN_PIN, GPIO_PIN_SET);
  g.Pin   = BOARD_PA_EN_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_PA_EN_PORT, &g);
}

/* Called from main() before RTOS/USB starts.
 * Waits 100ms for external pull-up to settle, then requires 5 consecutive
 * LOW readings to confirm button is genuinely pressed before starting the
 * 2s latch timer.  Released before 2s → POWER_CTL stays LOW. */
void BoardIO_StartupLatch(void)
{
  HAL_Delay(100U);  /* let external pull-up charge before first sample */

  /* Require 5 consecutive LOW samples (~10ms) to confirm button is pressed */
  uint8_t confirm = 0U;
  for (uint8_t i = 0U; i < 5U; i++)
  {
    if (PwrBtn_IsPressed()) { confirm++; }
    HAL_Delay(2U);
  }
  if (confirm < 5U)
  {
    return;  /* button not pressed → USB-only boot, POWER_CTL stays LOW */
  }

  uint32_t t0 = HAL_GetTick();
  while (PwrBtn_IsPressed())
  {
    if ((HAL_GetTick() - t0) >= 2000U)
    {
      PowerCtl_Set(1U);
      s_battery_latched = 1U;
      while (PwrBtn_IsPressed()) { HAL_Delay(20U); }  /* drain button */
      return;
    }
    HAL_Delay(20U);
  }
  /* Button released before 2s: POWER_CTL stays LOW */
}

uint8_t BoardIO_IsBatteryLatched(void)
{
  return s_battery_latched;
}

void LED_Set(uint8_t on)
{
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED_Toggle(void)
{
  HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
}

void PowerCtl_Set(uint8_t on)
{
  HAL_GPIO_WritePin(BOARD_POWER_CTL_PORT, BOARD_POWER_CTL_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t UserBtn_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(BOARD_USER_BTN_PORT, BOARD_USER_BTN_PIN)
          == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t PwrBtn_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(BOARD_PWR_BTN_PORT, BOARD_PWR_BTN_PIN)
          == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t UsbDet_IsPresent(void)
{
  return (HAL_GPIO_ReadPin(BOARD_USB_DET_PORT, BOARD_USB_DET_PIN)
          == GPIO_PIN_SET) ? 1U : 0U;
}

/* PA_EN is active-LOW (AO3401A P-channel PFET gate): on=1 → drive LOW → PFET
 * conducts → PAVCC=3V3 to ES8311; on=0 → drive HIGH → codec powered off. */
void PaEn_Set(uint8_t on)
{
  HAL_GPIO_WritePin(BOARD_PA_EN_PORT, BOARD_PA_EN_PIN,
                    on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
