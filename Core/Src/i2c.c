#include "i2c.h"

I2C_HandleTypeDef hi2c2;

/* PB13 = I2C2_SCL (AF4), PB14 = I2C2_SDA (AF4) —— 对应原理图 MCU pin 34/35。
 * 注意：PB10 已被 SPI2_SCK 占用（见 bsp_spi.c），不可用于 I2C2。
 * 启用内部上拉作为保险（若 PCB 已有外部上拉，并联无害）。 */
void MX_I2C2_Init(void)
{
  __HAL_RCC_I2C2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_13 | GPIO_PIN_14;
  g.Mode      = GPIO_MODE_AF_OD;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &g);

  hi2c2.Instance              = I2C2;
  /* Standard Mode 100kHz，PCLK1=160MHz：
   * PRESC=3 → I2CCLK=40MHz(25ns)，SCLDEL=10(275ns)，SDADEL=2，
   * SCLH=0xA0(4025ns ≥ 4000ns)，SCLL=0xFF(6400ns ≥ 4700ns)，fSCL≈96kHz */
  hi2c2.Init.Timing           = 0x30A2A0FFU;
  hi2c2.Init.OwnAddress1      = 0U;
  hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2      = 0U;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);
}

void I2C2_BusRecover(void)
{
  HAL_I2C_DeInit(&hi2c2);
  MX_I2C2_Init();
}
