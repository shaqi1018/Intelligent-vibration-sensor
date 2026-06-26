#ifndef __BATTERY_ADC_H__
#define __BATTERY_ADC_H__

#include "main.h"
#include <stdint.h>

void     BatteryADC_Init(void);
void     BatteryADC_DeInit(void);
uint16_t BatteryADC_ReadRaw(void);
uint32_t BatteryADC_ReadMillivolts(void);
uint8_t  BatteryADC_GetPercentage(void);

#endif /* __BATTERY_ADC_H__ */
