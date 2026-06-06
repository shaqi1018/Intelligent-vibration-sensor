#include "board_io.h"

void BoardIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* POWER_CTL (PB7) — output, immediately HIGH to latch battery supply */
  g.Pin   = BOARD_POWER_CTL_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_POWER_CTL_PORT, &g);
  HAL_GPIO_WritePin(BOARD_POWER_CTL_PORT, BOARD_POWER_CTL_PIN, GPIO_PIN_SET);

  /* LED (PB12) — output, start off */
  g.Pin = BOARD_LED_PIN;
  HAL_GPIO_Init(BOARD_LED_PORT, &g);
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_RESET);

  /* User button (PC15) — input, hardware pull-up already on board */
  g.Pin  = BOARD_USER_BTN_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_USER_BTN_PORT, &g);

  /* Power button (PC14) — input */
  g.Pin = BOARD_PWR_BTN_PIN;
  HAL_GPIO_Init(BOARD_PWR_BTN_PORT, &g);
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
