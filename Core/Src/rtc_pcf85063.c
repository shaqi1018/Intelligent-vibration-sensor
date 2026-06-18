#include "rtc_pcf85063.h"
#include "i2c.h"

#define PCF85063_ADDR        (0x51U << 1U)
#define PCF85063_REG_CTRL1   0x00U
#define PCF85063_REG_SECONDS 0x04U

static inline uint8_t BcdToDec(uint8_t b) { return (uint8_t)((b >> 4U) * 10U + (b & 0x0FU)); }
static inline uint8_t DecToBcd(uint8_t d) { return (uint8_t)(((d / 10U) << 4U) | (d % 10U)); }

uint8_t Pcf85063_Init(void)
{
  /* Control_1: 正常模式（bit7=0），24h 制（bit1=0），写 0x00 */
  uint8_t ctrl = 0x00U;
  return (HAL_I2C_Mem_Write(&hi2c2, PCF85063_ADDR, PCF85063_REG_CTRL1,
                             I2C_MEMADD_SIZE_8BIT, &ctrl, 1U, 50U) == HAL_OK)
         ? PCF85063_OK : PCF85063_ERR;
}

uint8_t Pcf85063_GetTime(Pcf85063_Time_t *t)
{
  uint8_t raw[7];
  /* 失败重试：若前序 I2C2 传输（如 ES8311 配置）留下错误态，第一次读会失败，
   * 恢复总线后重试即可成功 —— 避免 SD 文件时间戳整批 fallback。 */
  for (uint8_t attempt = 0U; attempt < 3U; attempt++)
  {
    if (HAL_I2C_Mem_Read(&hi2c2, PCF85063_ADDR, PCF85063_REG_SECONDS,
                         I2C_MEMADD_SIZE_8BIT, raw, 7U, 50U) == HAL_OK)
    {
      t->second = BcdToDec(raw[0] & 0x7FU);  /* 屏蔽 OS bit */
      t->minute = BcdToDec(raw[1] & 0x7FU);
      t->hour   = BcdToDec(raw[2] & 0x3FU);
      t->day    = BcdToDec(raw[3] & 0x3FU);
      /* raw[4] = weekday，忽略 */
      t->month  = BcdToDec(raw[5] & 0x1FU);
      t->year   = BcdToDec(raw[6]);
      return PCF85063_OK;
    }
    I2C2_BusRecover();   /* 恢复被前序失败污染的总线后重试 */
  }
  return PCF85063_ERR;
}

uint8_t Pcf85063_SetTime(const Pcf85063_Time_t *t)
{
  uint8_t raw[7];
  raw[0] = DecToBcd(t->second);
  raw[1] = DecToBcd(t->minute);
  raw[2] = DecToBcd(t->hour);
  raw[3] = DecToBcd(t->day);
  raw[4] = 0U;  /* weekday 不使用 */
  raw[5] = DecToBcd(t->month);
  raw[6] = DecToBcd(t->year);
  /* 重试 + 总线恢复(同 GetTime):前序 I2C2 失败可能污染总线,单次写易 fail。 */
  for (uint8_t attempt = 0U; attempt < 3U; attempt++)
  {
    if (HAL_I2C_Mem_Write(&hi2c2, PCF85063_ADDR, PCF85063_REG_SECONDS,
                          I2C_MEMADD_SIZE_8BIT, raw, 7U, 50U) == HAL_OK)
    {
      return PCF85063_OK;
    }
    I2C2_BusRecover();
  }
  return PCF85063_ERR;
}

static const uint8_t s_days_per_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

uint32_t Pcf85063_ToEpochSeconds(const Pcf85063_Time_t *t)
{
  uint32_t year4 = 2000U + t->year;
  uint32_t days  = 0U;
  for (uint32_t y = 1970U; y < year4; y++)
    days += ((y % 4U == 0U && (y % 100U != 0U || y % 400U == 0U)) ? 366U : 365U);

  uint8_t leap = (year4 % 4U == 0U && (year4 % 100U != 0U || year4 % 400U == 0U)) ? 1U : 0U;
  for (uint8_t m = 1U; m < t->month; m++) {
    days += s_days_per_month[m - 1U];
    if (m == 2U && leap) { days++; }
  }
  days += t->day - 1U;
  return days * 86400U + (uint32_t)t->hour * 3600U + (uint32_t)t->minute * 60U + t->second;
}

void Pcf85063_FromEpochSeconds(uint32_t epoch, Pcf85063_Time_t *t)
{
  uint32_t days = epoch / 86400U;
  uint32_t rem  = epoch % 86400U;
  t->hour   = (uint8_t)(rem / 3600U);
  t->minute = (uint8_t)((rem % 3600U) / 60U);
  t->second = (uint8_t)(rem % 60U);

  uint32_t year = 1970U;
  for (;;)
  {
    uint32_t diy = (year % 4U == 0U && (year % 100U != 0U || year % 400U == 0U)) ? 366U : 365U;
    if (days < diy) { break; }
    days -= diy;
    year++;
  }
  uint8_t leap = (year % 4U == 0U && (year % 100U != 0U || year % 400U == 0U)) ? 1U : 0U;
  uint8_t mon = 0U;
  for (; mon < 12U; mon++)
  {
    uint32_t dim = (uint32_t)s_days_per_month[mon] + ((mon == 1U && leap) ? 1U : 0U);
    if (days < dim) { break; }
    days -= dim;
  }
  t->year  = (uint8_t)(year - 2000U);
  t->month = (uint8_t)(mon + 1U);
  t->day   = (uint8_t)(days + 1U);
}
