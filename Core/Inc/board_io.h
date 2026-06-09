#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "main.h"
#include <stdint.h>

/* LED: PB12, active-high */
#define BOARD_LED_PIN           GPIO_PIN_12
#define BOARD_LED_PORT          GPIOB

/* POWER_CTL: PB7, output push-pull.
 * HIGH = 锁存电池供电; LOW = 不锁存（仅 USB 供电或已关机） */
#define BOARD_POWER_CTL_PIN     GPIO_PIN_7
#define BOARD_POWER_CTL_PORT    GPIOB

/* User button: PC15, active-low (10K hardware pull-up on board) */
#define BOARD_USER_BTN_PIN      GPIO_PIN_15
#define BOARD_USER_BTN_PORT     GPIOC

/* Power button: PC14, active-low (10K hardware pull-up on board) */
#define BOARD_PWR_BTN_PIN       GPIO_PIN_14
#define BOARD_PWR_BTN_PORT      GPIOC

/* USB_DET: PC7, input. VBUS via 10K/20K divider → ~3.33V when USB plugged.
 * HIGH = USB cable present; LOW = battery only. */
#define BOARD_USB_DET_PIN       GPIO_PIN_7
#define BOARD_USB_DET_PORT      GPIOC

/* PA_EN: codec 供电门控（AO3401A P 沟道 PFET 栅，脚36 = PB15）。
 * 低电平有效：拉低 → PFET 导通 → PAVCC = 3V3 给 ES8311 供电。
 * 默认拉高（关断，上电安全、省功耗）。 */
#define BOARD_PA_EN_PIN         GPIO_PIN_15
#define BOARD_PA_EN_PORT        GPIOB

void    BoardIO_Init(void);
void    BoardIO_StartupLatch(void);
uint8_t BoardIO_IsBatteryLatched(void);
void    LED_Set(uint8_t on);
void    LED_Toggle(void);
void    PowerCtl_Set(uint8_t on);
uint8_t UserBtn_IsPressed(void);
uint8_t PwrBtn_IsPressed(void);
uint8_t UsbDet_IsPresent(void);   /* 1 = USB cable plugged (PC7 HIGH) */
void    PaEn_Set(uint8_t on);     /* 1 = 给 codec 供电（内部拉低 PA_EN） */

#endif /* BOARD_IO_H */
