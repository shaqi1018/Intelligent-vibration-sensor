#ifndef ES8311_H
#define ES8311_H
#include <stdint.h>

#define ES8311_I2C_ADDR_8BIT   0x30U   /* CE=0 → 7-bit 0x18，左移 1 位 */

int  ES8311_Probe(void);                                       /* 读寄存器自检，0=成功 */
int  ES8311_InitAdc(uint32_t sample_rate_hz, uint16_t gain_db);/* 0=成功 */
void ES8311_PowerDown(void);
#endif
