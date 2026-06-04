#include "usbd_wcid_app.h"
#include "usbd_wcid_streaming.h"
#include <string.h>
#include <stdio.h>

static USBD_HandleTypeDef *s_pdev;
static UsbWcidApp_CmdHandler s_cmd_handler;

/* Static double-buffers: 3 channels × (WCID_TX_HALF_SIZE × 2) = 1536 bytes. */
static uint8_t s_tx_buf_lsm[WCID_TX_HALF_SIZE * 2];
static uint8_t s_tx_buf_h3[WCID_TX_HALF_SIZE * 2];
static uint8_t s_tx_buf_qma[WCID_TX_HALF_SIZE * 2];
static uint8_t s_rx_buf[64U]; /* OUT EP1 command receive buffer */

/* WCID class interface callbacks — minimal stubs. */
static int8_t WcidApp_Init(void)
{
  /* pClassData is allocated by USBD_WCID_STREAMING_Init before this is called.
   * Set up TX buffers here (not in UsbWcidApp_Init) because pClassData
   * is NULL until the class Init runs. */
  USBD_WCID_STREAMING_SetRxDataBuffer(s_pdev, s_rx_buf);
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_LSM_IMU,   s_tx_buf_lsm, WCID_TX_HALF_SIZE);
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_H3_ACCEL,  s_tx_buf_h3,  WCID_TX_HALF_SIZE);
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_QMA_ACCEL, s_tx_buf_qma, WCID_TX_HALF_SIZE);
  return 0;
}
static int8_t WcidApp_DeInit(void) { return 0; }
static int8_t WcidApp_Control(uint8_t cmd, uint8_t req, uint16_t value, uint16_t index, uint8_t *buf, uint16_t len)
{
  UNUSED(cmd); UNUSED(req); UNUSED(value); UNUSED(index); UNUSED(buf); UNUSED(len);
  return 0;
}

static int8_t WcidApp_Receive(uint8_t *buf, uint32_t len)
{
  if (s_cmd_handler != NULL && len > 0U)
  {
    s_cmd_handler((const char *)buf, len);
  }
  return 0;
}

static USBD_WCID_STREAMING_ItfTypeDef s_wcid_if = {
  WcidApp_Init,
  WcidApp_DeInit,
  WcidApp_Control,
  WcidApp_Receive
};

void UsbWcidApp_Init(USBD_HandleTypeDef *pdev)
{
  s_pdev = pdev;
  printf("[WCID-APP] RegisterInterface...\r\n");
  USBD_WCID_STREAMING_RegisterInterface(pdev, &s_wcid_if);
  printf("[WCID-APP] RegisterInterface done (buffers set in WcidApp_Init callback)\r\n");
}

void UsbWcidApp_StartStreaming(void)
{
  USBD_WCID_STREAMING_CleanTxDataBuffer(s_pdev, WCID_CH_LSM_IMU);
  USBD_WCID_STREAMING_CleanTxDataBuffer(s_pdev, WCID_CH_H3_ACCEL);
  USBD_WCID_STREAMING_CleanTxDataBuffer(s_pdev, WCID_CH_QMA_ACCEL);
  USBD_WCID_STREAMING_StartStreaming(s_pdev);
}

void UsbWcidApp_StopStreaming(void)
{
  USBD_WCID_STREAMING_StopStreaming(s_pdev);
}

uint8_t UsbWcidApp_SendCsv(uint8_t ch, const char *csv, uint32_t len)
{
  return USBD_WCID_STREAMING_FillTxDataBuffer(s_pdev, ch, (uint8_t *)csv, len);
}

void UsbWcidApp_SetCmdHandler(UsbWcidApp_CmdHandler handler)
{
  s_cmd_handler = handler;
}
