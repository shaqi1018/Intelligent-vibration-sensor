#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "main.h"
#include <stdint.h>

/* LED: PB12, active-high */
#define BOARD_LED_PIN           GPIO_PIN_12
#define BOARD_LED_PORT          GPIOB

/* POWER_CTL: PB7, output push-pull.
 * HIGH = 锁存供电（上电后立即拉高）; LOW = 断电 */
#define BOARD_POWER_CTL_PIN     GPIO_PIN_7
#define BOARD_POWER_CTL_PORT    GPIOB

/* User button: PC15, active-low (10K hardware pull-up on board) */
#define BOARD_USER_BTN_PIN      GPIO_PIN_15
#define BOARD_USER_BTN_PORT     GPIOC

/* Power button: PC14, active-low (10K hardware pull-up on board) */
#define BOARD_PWR_BTN_PIN       GPIO_PIN_14
#define BOARD_PWR_BTN_PORT      GPIOC

void    BoardIO_Init(void);
void    LED_Set(uint8_t on);
void    LED_Toggle(void);
void    PowerCtl_Set(uint8_t on);
uint8_t UserBtn_IsPressed(void);
uint8_t PwrBtn_IsPressed(void);

#endif /* BOARD_IO_H */
