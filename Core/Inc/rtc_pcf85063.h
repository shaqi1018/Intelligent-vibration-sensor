#ifndef RTC_PCF85063_H
#define RTC_PCF85063_H

#include <stdint.h>

typedef struct {
  uint8_t  year;    /* 0-99，加 2000 得公历年 */
  uint8_t  month;   /* 1-12 */
  uint8_t  day;     /* 1-31 */
  uint8_t  hour;    /* 0-23 */
  uint8_t  minute;  /* 0-59 */
  uint8_t  second;  /* 0-59 */
} Pcf85063_Time_t;

#define PCF85063_OK   0
#define PCF85063_ERR  1

uint8_t  Pcf85063_Init(void);
uint8_t  Pcf85063_GetTime(Pcf85063_Time_t *t);
uint8_t  Pcf85063_SetTime(const Pcf85063_Time_t *t);
uint32_t Pcf85063_ToEpochSeconds(const Pcf85063_Time_t *t);  /* UTC Unix 纪元秒 */
void     Pcf85063_FromEpochSeconds(uint32_t epoch_s, Pcf85063_Time_t *t);  /* 纪元秒 → 年月日时分秒 */

#endif
