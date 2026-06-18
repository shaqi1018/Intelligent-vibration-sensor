#include "usbd_wcid_app.h"
#include "usbd_wcid_streaming.h"
#include <string.h>
#include <stdio.h>

static USBD_HandleTypeDef *s_pdev;
static UsbWcidApp_CmdHandler s_cmd_handler;

/* LSM USB 发送诊断计数器(用于定位 6664Hz 丢帧根因) */
volatile uint32_t g_lsm_usb_overrun = 0U;        /* 生产者写入冲突次数(主机读太慢) */
volatile uint32_t g_lsm_usb_sof_send = 0U;       /* SOF 发送 LSM 的次数 */
volatile uint32_t g_lsm_usb_datain_complete = 0U;/* DataIn 完成 LSM 的次数 */

/* 静态双缓冲：4 通道 × (WCID_TX_HALF_SIZE × 2 + 2)。
 * 多出的 "+2" 用于 tag 通道（最后通道 = MIC = N_IN_ENDPOINTS-1）的每半通道标记字节：
 * 中间件以 (size*2+2) 取模索引缓冲，若只声明 size*2 会越界 2 字节。
 * 为安全起见 LSM/H3/QMA（非 tag 通道）也统一声明为 size*2+2。 */
static uint8_t s_tx_buf_lsm[WCID_TX_HALF_SIZE * 2 + 2];
static uint8_t s_tx_buf_h3[WCID_TX_HALF_SIZE * 2 + 2];
static uint8_t s_tx_buf_qma[WCID_TX_HALF_SIZE * 2 + 2];
static uint8_t s_tx_buf_mic[WCID_TX_HALF_SIZE * 2 + 2]; /* ES8311 raw PCM (ch3 = last EP) */
static uint8_t s_rx_buf[64U]; /* OUT EP1 command receive buffer */

/* MIC half-buffer (bytes): 96kHz/16-bit PCM is rate-based not row-based, so use a
 * fixed half. 2048B ≈ 10.6ms audio/half; one FS bulk transfer of 2048B (~1.7ms)
 * sustains >1MB/s/EP, far above the 192KB/s the mic produces. */
#define WCID_MIC_HALF_SIZE  2048U

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
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_MIC,       s_tx_buf_mic, WCID_MIC_HALF_SIZE);
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

/* 各通道 CSV 行最大长度（含 CRLF，字节），用于按 ODR 计算半缓冲大小，略偏大以留余量。 */
#define WCID_ROW_BYTES_LSM  64U  /* 8 字段：id,tick,3轴加速,3轴陀螺 */
#define WCID_ROW_BYTES_H3   48U  /* 5 字段：id,tick,3轴加速 */
#define WCID_ROW_BYTES_QMA  48U  /* 5 字段：id,tick,3轴加速 */

/* 参考 DATALOG1 的大小计算：每半 ≈ 500ms 数据量，上限 WCID_TX_HALF_SIZE，
 * 下限保证至少能放一行数据 + 余量。 */
static uint16_t WcidComputeHalfSize(uint32_t odr_hz, uint16_t bytes_per_row)
{
  uint32_t sz = (odr_hz * (uint32_t)bytes_per_row) / 2U;
  if (sz > WCID_TX_HALF_SIZE) { sz = WCID_TX_HALF_SIZE; }
  if (sz < (uint32_t)(bytes_per_row + 8U)) { sz = (uint32_t)bytes_per_row + 8U; }
  return (uint16_t)sz;
}

void UsbWcidApp_StartStreaming(uint32_t lsm_odr_hz, uint32_t h3_odr_hz, uint32_t qma_odr_hz)
{
  /* 启动前按当前 ODR 重新设置各通道半缓冲大小。
   * 静态缓冲已按 WCID_TX_HALF_SIZE 上限分配，所有计算结果都在范围内。 */
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_LSM_IMU,   s_tx_buf_lsm,
                                      WcidComputeHalfSize(lsm_odr_hz, WCID_ROW_BYTES_LSM));
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_H3_ACCEL,  s_tx_buf_h3,
                                      WcidComputeHalfSize(h3_odr_hz, WCID_ROW_BYTES_H3));
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_QMA_ACCEL, s_tx_buf_qma,
                                      WcidComputeHalfSize(qma_odr_hz, WCID_ROW_BYTES_QMA));
  /* MIC: fixed half-buffer (rate-based PCM, not row-based). */
  USBD_WCID_STREAMING_SetTxDataBuffer(s_pdev, WCID_CH_MIC, s_tx_buf_mic, WCID_MIC_HALF_SIZE);

  USBD_WCID_STREAMING_CleanTxDataBuffer(s_pdev, WCID_CH_LSM_IMU);
  USBD_WCID_STREAMING_CleanTxDataBuffer(s_pdev, WCID_CH_H3_ACCEL);
  USBD_WCID_STREAMING_CleanTxDataBuffer(s_pdev, WCID_CH_QMA_ACCEL);
  USBD_WCID_STREAMING_CleanTxDataBuffer(s_pdev, WCID_CH_MIC);

  /* Reset LSM USB diagnostics counters for this new streaming session */
  g_lsm_usb_overrun = 0U;
  g_lsm_usb_sof_send = 0U;
  g_lsm_usb_datain_complete = 0U;

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

static uint8_t  s_resp_buf[1024];
static uint32_t s_resp_len;
void UsbWcidApp_RespReset(void)  { s_resp_len = 0U; }
void UsbWcidApp_RespAppend(const uint8_t *buf, uint32_t len)
{
  uint32_t space = sizeof(s_resp_buf) - s_resp_len;
  if (len > space) { len = space; }
  if (len != 0U) { memcpy(&s_resp_buf[s_resp_len], buf, len); s_resp_len += len; }
}
uint8_t UsbWcidApp_RespSend(void)
{
  if (s_resp_len == 0U) { return 0U; }
  uint8_t r = UsbWcidApp_Write(s_resp_buf, s_resp_len);
  s_resp_len = 0U;
  return r;
}

uint8_t UsbWcidApp_Write(const uint8_t *buf, uint32_t len)
{
  if (s_pdev == NULL) { return 1U; }
  if (len == 0U)      { return 0U; }
  if (len > 0xFFFFU)  { len = 0xFFFFU; }
  return USBD_WCID_STREAMING_SendResponse(s_pdev, buf, (uint16_t)len);
}
