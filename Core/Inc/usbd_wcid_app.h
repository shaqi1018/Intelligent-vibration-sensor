#ifndef USBD_WCID_APP_H
#define USBD_WCID_APP_H

#include "usbd_core.h"

/* Channel indices — map to DATA_IN_EP(i+1). Natural order, all channels clean:
 * the middleware's per-half channel-tag byte is now gated to only fire when an
 * endpoint actually carries >1 channel (N_CHANNELS_MAX > N_IN_ENDPOINTS). Here
 * it is 1:1 (4 channels / 4 data EPs) so NO channel gets a tag → MIC streams as
 * pure 16-bit PCM and the sensors keep their original endpoints. */
#define WCID_CH_LSM_IMU   0U   /* EP1 0x81, clean */
#define WCID_CH_H3_ACCEL  1U   /* EP2 0x82, clean */
#define WCID_CH_QMA_ACCEL 2U   /* EP3 0x83, clean (tag removed; was tagged when 3-ch) */
#define WCID_CH_MIC       3U   /* EP4 0x84, clean raw PCM (appended) */

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

/* LSM 专用更大半缓冲:LSM 6664Hz≈386KB/s,4096B 半缓冲只 ~10.6ms 就被生产者填满,
 * 4 端点(+MIC SAI/DMA 干扰)下 SOF 发完一个半缓冲可能 >10.6ms → 生产者追上、覆盖
 * 正在发的半缓冲 → 整块~140帧倒退+半行劈开(上位机 raw.bin 实证)。加大到 16KB(~42ms)
 * 给 SOF 充足时间发完,生产者不再追上。仅 LSM 用,其它通道仍 4096。 */
#define WCID_TX_HALF_SIZE_LSM  16384U

void UsbWcidApp_Init(USBD_HandleTypeDef *pdev);

/* 启动 USB 流式传输。按各通道 ODR 动态计算半缓冲大小（约 500ms 数据量，
 * 上限 WCID_TX_HALF_SIZE），确保高 ODR 通道 FIFO 批次不会覆盖正在发送的
 * 半缓冲，同时低 ODR 通道在短采集内也能填满并发出。禁用通道传 0（使用下限）。 */
void UsbWcidApp_StartStreaming(uint32_t lsm_odr_hz, uint32_t h3_odr_hz, uint32_t qma_odr_hz);

/* Push raw bytes into a channel's double-buffer (alias of SendCsv; byte-transparent,
 * used by the MIC channel for binary PCM). */
#define UsbWcidApp_SendRaw(ch, buf, len)  UsbWcidApp_SendCsv((ch), (const char *)(buf), (len))
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

/* Send raw bytes back to host via the command-response IN endpoint (0x85).
 * Serialized by an RTOS mutex (see UsbWcidApp_InitRtos) so the command task,
 * the AHT20 task and the LIS2MDL task can all write 0x85 without racing the
 * single response buffer inside USBD_WCID_STREAMING_SendResponse. */
uint8_t UsbWcidApp_Write(const uint8_t *buf, uint32_t len);

/* 非阻塞版 0x85 写：给高频生产者(MAG @100Hz)用。端点忙/锁竞争即丢帧立即返回，
 * 绝不 osDelay 重试。MAG 走 DRDY 二值信号量的 10ms 热路径，任何 tick 级阻塞都会
 * 拖慢采集并静默丢样本(USB 路径实测掉到 ~70%)；拥塞时丢个别 USB 行远比拖垮 100Hz
 * 节拍划算。mic/LSM/H3/QMA 走各自双缓冲端点,不受影响。 */
uint8_t UsbWcidApp_WriteNonBlocking(const uint8_t *buf, uint32_t len);

/* 1 = USB 主机已连接并配置（设备处于 CONFIGURED 态），0 = 无主机。
 * 用于避免无主机时把状态行(如 DONE dir=)塞进端点 FIFO，等主机接入被陈旧顶出。 */
uint8_t UsbWcidApp_IsConfigured(void);

/* Create the mutex that serializes UsbWcidApp_Write (0x85). Call once from
 * MX_FREERTOS_Init, in task/kernel-init context (osMutexNew needs the kernel).
 * Before this runs (e.g. USB enumeration), Write falls back to no-lock, which
 * is safe because only one context sends at that stage. */
void UsbWcidApp_InitRtos(void);

#endif
