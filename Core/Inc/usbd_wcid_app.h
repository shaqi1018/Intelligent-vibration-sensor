#ifndef USBD_WCID_APP_H
#define USBD_WCID_APP_H

#include "usbd_core.h"

/* Channel indices — map to DATA_IN_EP1..3 */
#define WCID_CH_LSM_IMU   0U
#define WCID_CH_H3_ACCEL  1U
#define WCID_CH_QMA_ACCEL 2U

/* 每通道半缓冲大小（字节）。
 *
 * 生产者（传感器任务）在微秒级时间内将一整批 FIFO 数据写入此缓冲；
 * 消费者（SOF 中断）每个 USB 帧发送一个半缓冲。若一批数据超过半缓冲长度，
 * 生产者会回绕并覆盖仍在发送中的半缓冲 → 字节撕裂/字段合并（损坏程度
 * 与 ODR 正相关：LSM@833Hz 严重乱码，H3/QMA@50-100Hz 正常）。
 *
 * 参考 DATALOG1 的做法：每通道半缓冲 = ODR × 每行字节数 × 0.5（500ms 数据量），
 * 上限 4096B。最坏情况（LSM/QMA CSV 行 ~50B，ODR 833/1600Hz）均达到上限，
 * 因此统一定为 4096B 上限 — 足够大，任何一批 FIFO 数据都不会绕过正在发送的半缓冲。
 * RAM 占用：3 通道 × (4096×2 + 2) ≈ 24.6KB（剩余空间充裕）。 */
#define WCID_TX_HALF_SIZE  4096U

void UsbWcidApp_Init(USBD_HandleTypeDef *pdev);

/* 启动 USB 流式传输。按各通道 ODR 动态计算半缓冲大小（约 500ms 数据量，
 * 上限 WCID_TX_HALF_SIZE），确保高 ODR 通道 FIFO 批次不会覆盖正在发送的
 * 半缓冲，同时低 ODR 通道在短采集内也能填满并发出。禁用通道传 0（使用下限）。 */
void UsbWcidApp_StartStreaming(uint32_t lsm_odr_hz, uint32_t h3_odr_hz, uint32_t qma_odr_hz);
void UsbWcidApp_StopStreaming(void);

/* Sensor tasks call this to push a CSV row into the USB double-buffer.
 * Thread-safe — internally uses __disable_irq for TxBuffStatus. */
uint8_t UsbWcidApp_SendCsv(uint8_t channel, const char *csv, uint32_t len);

/* Command receive callback — invoked from USB class driver Receive callback. */
typedef void (*UsbWcidApp_CmdHandler)(const char *cmd, uint32_t len);
void UsbWcidApp_SetCmdHandler(UsbWcidApp_CmdHandler handler);

void    UsbWcidApp_RespReset(void);
void    UsbWcidApp_RespAppend(const uint8_t *buf, uint32_t len);
uint8_t UsbWcidApp_RespSend(void);

/* Send raw bytes back to host via EP4 IN (command response channel). */
uint8_t UsbWcidApp_Write(const uint8_t *buf, uint32_t len);

#endif
