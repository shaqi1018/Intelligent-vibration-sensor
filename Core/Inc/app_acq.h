#ifndef APP_ACQ_H
#define APP_ACQ_H

#include <stdint.h>

#define APP_ACQ_SINK_USB  1U
#define APP_ACQ_SINK_SD   2U

/* Acquisition control — implemented in app_freertos.c */
uint32_t AppAcqIsRunning(void);
uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms);
uint32_t AppAcqStop(void);

/* 诚实采集指示：采集运行中且(麦克风未启用或已开录)。供采集灯使用，
 * 使"灯亮 = 所有启用的源都在采"。实现于 app_freertos.c。 */
uint8_t AppCaptureActive(void);

#endif /* APP_ACQ_H */
