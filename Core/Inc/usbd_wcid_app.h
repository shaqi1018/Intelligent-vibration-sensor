#ifndef USBD_WCID_APP_H
#define USBD_WCID_APP_H

#include "usbd_core.h"

/* Channel indices — map to DATA_IN_EP1..3 */
#define WCID_CH_LSM_IMU   0U
#define WCID_CH_H3_ACCEL  1U
#define WCID_CH_QMA_ACCEL 2U

/* Half-buffer size per channel (bytes).
 * LSM: ~80B/sample × 2 sample/5ms = 160B, with margin.
 * H3/QMA: ~50B/sample, smaller. Unified at 256B. */
#define WCID_TX_HALF_SIZE  256U

void UsbWcidApp_Init(USBD_HandleTypeDef *pdev);
void UsbWcidApp_StartStreaming(void);
void UsbWcidApp_StopStreaming(void);

/* Sensor tasks call this to push a CSV row into the USB double-buffer.
 * Thread-safe — internally uses __disable_irq for TxBuffStatus. */
uint8_t UsbWcidApp_SendCsv(uint8_t channel, const char *csv, uint32_t len);

/* Command receive callback — invoked from USB class driver Receive callback. */
typedef void (*UsbWcidApp_CmdHandler)(const char *cmd, uint32_t len);
void UsbWcidApp_SetCmdHandler(UsbWcidApp_CmdHandler handler);

#endif
