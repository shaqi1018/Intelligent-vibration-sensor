#ifndef __KEY_ADC_H__
#define __KEY_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void KeyAdc_Init(void);
void KeyAdc_Poll(void);
uint16_t KeyAdc_GetRaw(void);
uint32_t KeyAdc_GetMillivolt(void);
uint8_t KeyAdc_IsVbusPresent(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_ADC_H__ */
