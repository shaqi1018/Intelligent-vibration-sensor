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
  if (HAL_I2C_Mem_Read(&hi2c2, PCF85063_ADDR, PCF85063_REG_SECONDS,
                        I2C_MEMADD_SIZE_8BIT, raw, 7U, 50U) != HAL_OK)
    return PCF85063_ERR;

  t->second = BcdToDec(raw[0] & 0x7FU);  /* 屏蔽 OS bit */
  t->minute = BcdToDec(raw[1] & 0x7FU);
  t->hour   = BcdToDec(raw[2] & 0x3FU);
  t->day    = BcdToDec(raw[3] & 0x3FU);
  /* raw[4] = weekday，忽略 */
  t->month  = BcdToDec(raw[5] & 0x1FU);
  t->year   = BcdToDec(raw[6]);
  return PCF85063_OK;
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
  return (HAL_I2C_Mem_Write(&hi2c2, PCF85063_ADDR, PCF85063_REG_SECONDS,
                              I2C_MEMADD_SIZE_8BIT, raw, 7U, 50U) == HAL_OK)
         ? PCF85063_OK : PCF85063_ERR;
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
