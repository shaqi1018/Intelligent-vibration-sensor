#ifndef APP_ACQ_H
#define APP_ACQ_H

#include <stdint.h>

#define APP_ACQ_SINK_USB  1U
#define APP_ACQ_SINK_SD   2U

/* Acquisition control — implemented in app_freertos.c */
uint32_t AppAcqIsRunning(void);
uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms);
uint32_t AppAcqStop(void);

#endif /* APP_ACQ_H */
