#include "i2c.h"
#include "app_locks.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c1;

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

/* I2C2 锁约定(M1)：本函数【不】自锁。它只在 Pcf85063_GetTime/SetTime 的失败重试
 * 路径里被调用，而那两个函数总是在已持有 i2c2_mutex 的上下文中运行(AppTime_Sync /
 * set_time 命令)。若在此再 AppI2c2Lock 会对非递归锁造成递归双锁 → 死锁。故调用者
 * 必须已持锁；本函数依赖调用者的锁完成串行化。 */
void I2C2_BusRecover(void)
{
  /* 简单恢复:DeInit + Init,重置 MCU I2C 外设与句柄。
   * (曾试过手动翻转 SCL/SDA 发脉冲的强力恢复,但那个反而可能干扰总线,已移除。) */
  HAL_I2C_DeInit(&hi2c2);
  MX_I2C2_Init();
}

/* 探测 RTC(0x51) 是否在 I2C2 上应答。返回 1=应答(硬件OK),0=无应答。
 * I2C2 与 ES8311 共享：经 i2c2_mutex 串行化，避免与 codec/RTC 事务在总线上交错(M1)。 */
uint8_t I2C2_ProbeRtc(void)
{
  AppI2c2Lock();
  uint8_t ok = (HAL_I2C_IsDeviceReady(&hi2c2, (0x51U << 1U), 3U, 100U) == HAL_OK) ? 1U : 0U;
  AppI2c2Unlock();
  return ok;
}

/* PB6 = I2C1_SCL (AF4), PB3 = I2C1_SDA (AF4) —— 对应原理图 MCU pin 58/55。
 * I2C1 为 AHT20(0x38) 与 LIS2MDL(0x1E) 专用总线，与 I2C2(ES8311/RTC) 物理隔离。
 * 板载已有 4.7K 外部上拉(R13/R14)；内部上拉作保险，并联无害。
 * 默认内核时钟 PCLK1=160MHz，复用 I2C2 的 0x30A2A0FF ≈ 100kHz 时序。 */
void MX_I2C1_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_6 | GPIO_PIN_3;   /* PB6=SCL, PB3=SDA */
  g.Mode      = GPIO_MODE_AF_OD;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &g);

  hi2c1.Instance              = I2C1;
  hi2c1.Init.Timing           = 0x30A2A0FFU;   /* ~100kHz @ PCLK1=160MHz */
  hi2c1.Init.OwnAddress1      = 0U;
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2      = 0U;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}

void I2C1_BusRecover(void)
{
  HAL_I2C_DeInit(&hi2c1);
  MX_I2C1_Init();
}

uint8_t I2C1_ProbeBus(void)
{
  uint8_t mask = 0U;
  if (HAL_I2C_IsDeviceReady(&hi2c1, (0x38U << 1U), 3U, 100U) == HAL_OK) { mask |= 0x01U; }
  if (HAL_I2C_IsDeviceReady(&hi2c1, (0x1EU << 1U), 3U, 100U) == HAL_OK) { mask |= 0x02U; }
  printf("[I2C1] probe: AHT20(0x38)=%s  LIS2MDL(0x1E)=%s\r\n",
         (mask & 0x01U) ? "ACK" : "----",
         (mask & 0x02U) ? "ACK" : "----");
  return mask;
}
