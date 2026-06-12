#include "es8311.h"
#include "i2c.h"
#include "cmsis_os2.h"

static int es_write(uint8_t reg, uint8_t val)
{
  return (HAL_I2C_Mem_Write(&hi2c2, ES8311_I2C_ADDR_8BIT, reg,
            I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK) ? 0 : -1;
}
static int es_read(uint8_t reg, uint8_t *val)
{
  return (HAL_I2C_Mem_Read(&hi2c2, ES8311_I2C_ADDR_8BIT, reg,
            I2C_MEMADD_SIZE_8BIT, val, 1, 100) == HAL_OK) ? 0 : -1;
}

int ES8311_Probe(void)
{
  uint8_t id1 = 0, id2 = 0;
  if (es_read(0xFD, &id1) != 0) return -1;   /* ES8311 CHIP_ID1 ≈ 0x83 */
  if (es_read(0xFE, &id2) != 0) return -1;   /* CHIP_ID2 ≈ 0x11 */
  return (id1 == 0x83 && id2 == 0x11) ? 0 : -2;
}

/* MCLK/Fs = 256（恒定）→ 所有采样率共用一组时钟系数（REG02/05/06/07/08/OSR）。
 * 唯一随采样率变化的是 REG03 的 fs_mode 位（bit6）：
 *   单速模式 single speed（8..48k）：fs_mode=0
 *   双速模式 double speed（64..96k）：fs_mode=1
 * 与 ESP 官方 es8311 驱动 coeff_div 表 {24.576MHz,96k} 那一档完全吻合（osr 仍 0x10）。*/
#define ES_REG02_CLK   0x00   /* pre_div=1,pre_multi=1 → 低3位保留,高位0 */
#define ES_REG03_SINGLE 0x10  /* fs_mode=0<<6 | adc_osr=0x10（8..48k） */
#define ES_REG03_DOUBLE 0x50  /* fs_mode=1<<6 | adc_osr=0x10（64..96k） */
#define ES_REG04_CLK   0x10   /* dac_osr(adc-only 无害) */
#define ES_REG05_CLK   0x00   /* (adc_div-1)<<4 | (dac_div-1) = 0 */
#define ES_REG06_CLK   0x03   /* bclk_div=4 (<19 → 写 bclk_div-1=3),高3位保留 */
#define ES_REG07_CLK   0x00   /* lrck_h */
#define ES_REG08_CLK   0xFF   /* lrck_l (lrck=0x00FF=255 → /256) */

int ES8311_InitAdc(uint32_t sample_rate_hz, uint16_t gain_db)
{
  /* fs_mode：≥64kHz 用双速模式，否则单速。其余时钟系数与采样率无关（MCLK/Fs 恒 256）。 */
  uint8_t reg03 = (sample_rate_hz >= 64000U) ? ES_REG03_DOUBLE : ES_REG03_SINGLE;
  /* 复位 */
  if (es_write(0x00, 0x1F) != 0) return -1;
  osDelay(1);
  es_write(0x45, 0x00);
  /* 时钟管理：MCLK 来源、使能 ADC 时钟 */
  es_write(0x01, 0x3F);            /* 全时钟使能：MCLK/BCLK/ADC clk/DAC clk/模拟 clk
                                    * （官方驱动值；此前误设 0x30 漏开 ADC 时钟低位位，
                                    *  导致 ADC 内核无时钟、SDOUT 恒 0、录音全静音） */
  es_write(0x02, ES_REG02_CLK);
  es_write(0x03, reg03);
  es_write(0x16, 0x24);
  es_write(0x04, ES_REG04_CLK);
  es_write(0x05, ES_REG05_CLK);
  es_write(0x06, ES_REG06_CLK);
  es_write(0x07, ES_REG07_CLK);
  es_write(0x08, ES_REG08_CLK);
  /* SDP out（ADC）格式：I2S, 16-bit */
  es_write(0x09, 0x00);
  es_write(0x0A, 0x00);            /* 0x00 = I2S 16-bit；左/右对齐改此寄存器 */
  /* 系统/模拟上电 */
  es_write(0x0D, 0x01);
  es_write(0x0E, 0x02);            /* 模拟 mic PGA 上电 */
  es_write(0x12, 0x00);
  es_write(0x13, 0x10);
  es_write(0x10, 0x03);            /* vmid */
  es_write(0x11, 0x7B);
  es_write(0x00, 0x80);            /* 从机模式、整体上电 */
  /* ADC 数字：选择模拟 mic（MIC1P/N 差分），关闭 DAC 回环 */
  es_write(0x44, 0x08);            /* adc 数据来源 = adc（非回环） */
  es_write(0x17, 0xBF);            /* ADC 数字音量（0dB 附近） */
  /* mic PGA 增益：0x14 低 4 位选增益档（约 0..42dB，3dB/档）+ 选 MIC1 通道 */
  uint8_t pga = (uint8_t)((gain_db / 3U) & 0x0FU);
  es_write(0x14, (uint8_t)(0x10 | pga));  /* 0x10=选模拟差分 mic1，低4位增益 */
  es_write(0x15, 0x00);
  es_write(0x1B, 0x0A);
  es_write(0x1C, 0x6A);            /* ADC 高通/格式 */
  return 0;
}

void ES8311_PowerDown(void)
{
  es_write(0x0E, 0xFF);   /* 模拟下电 */
  es_write(0x12, 0x02);
  es_write(0x00, 0x00);   /* 复位/下电 */
}
