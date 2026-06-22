#ifndef SAI_H
#define SAI_H

#include "stm32u5xx_hal.h"

/* SAI1 Block A: master receiver, I2S Philips, 16-bit, MCLK output (drives the
 * ES8311 codec which runs as I2S slave). Audio data (codec -> MCU) on SAI1_SD_A.
 *
 * Pins (confirmed in plan Task0 Step1, reconfirm AF against the STM32U575
 * datasheet alternate-function table during bench bring-up):
 *   SAI1_MCLK_A = PB8  (AF13)  -> ES8311 MCLK
 *   SAI1_SCK_A  = PA8  (AF13)  -> ES8311 SCLK (BCLK)
 *   SAI1_FS_A   = PB9  (AF13)  -> ES8311 LRCK
 *   SAI1_SD_A   = PC3  (AF13)  -> ES8311 ASDOUT (data in)
 *   (all four are AF13 — the AF3 value here previously caused the silent-mic bug)
 */

extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel_SAI;

/* Configure SAI1 Block A as master-RX, I2S 16-bit, MCLK output, plus the
 * circular GPDMA channel that feeds the RX FIFO into memory.
 * sample_rate_hz must be one of {8000, 16000, 48000}. Returns HAL_OK on success.
 *
 * NOTE: the SAI master clock generator produces MCLK = 256 x Fs, so MCLK is
 * 12.288 MHz only at Fs = 48000 (4.096 MHz at 16k, 2.048 MHz at 8k). See the
 * comment in sai.c and the Task0 report for the fixed-MCLK design caveat. */
HAL_StatusTypeDef MX_SAI1_Init(uint32_t sample_rate_hz);

/* Tear down SAI1 Block A and its DMA channel (idempotent). */
void MX_SAI1_DeInit(void);

#endif /* SAI_H */
