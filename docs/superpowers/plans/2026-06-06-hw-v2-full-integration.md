# HW-v2 Full Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在已通 USB WCID 枚举的新板基础上，完成 HW-v2 引脚重映射、SD 检测适配、电源锁存、按键采集控制和 LED 闪烁。

**Architecture:** 分四个独立层次递进：(1) 提交当前 USB 基线；(2) GPIO + SD 适配（传感器 SPI 跑通）；(3) board_io 外设驱动；(4) user_ctrl 任务 + AppAcq 公开接口。每层均可独立编译验证。

**Tech Stack:** STM32U575RIT, FreeRTOS CMSIS-OS2, HAL, Keil MDK ARMCLANG V6

**调试积累的关键约束（必须遵守）：**
- USB ISR 时序：`MX_USB_OTG_FS_PCD_Init()` 后必须立即 `HAL_NVIC_DisableIRQ(OTG_FS_IRQn)`；仅在 `USBD_Start` 前 `HAL_NVIC_EnableIRQ(OTG_FS_IRQn)`
- 传感器任务与 WCID 任务同优先级(AboveNormal)；若 SPI CS 引脚错误 → SPI 永不应答 → 传感器任务挂死 → WCID 任务饿死 → USB 卡在 RegisterClass
- SD 新板无硬件卡检测（push 式，PC13 = 其他功能）；选择2：始终尝试挂载，靠结果判断

---

## 文件变更总览

| 文件 | 操作 | 说明 |
|---|---|---|
| `Core/Src/main.c` | 修改 | USB ISR disable/enable; SD 检测改为始终尝试; BoardIO_Init |
| `Core/Src/app_freertos.c` | 修改 | EXTI callback HW-v2; 重新启用传感器任务; AppAcq 去 static; 添加 UserCtrl 任务 |
| `BSP/Sensor/Inc/bsp_spi.h` | 修改 | CS 引脚宏：LSM→PB1, H3→PA4, QMA→PC5 |
| `BSP/Sensor/Src/bsp_spi.c` | 修改 | SPI1 CS GPIO 时钟：GPIOC→GPIOB |
| `BSP/Sensor/Inc/h3lis100dl.h` | 修改 | INT 引脚：PB4/EXTI4→PA1/EXTI1 |
| `BSP/Sensor/Inc/qma6100p.h` | 修改 | INT 引脚：PB15/EXTI15→PC4/EXTI4 |
| `Core/Src/gpio.c` | 修改 | EXTI 配置 HW-v2：PB0/PA1/PC4; 删 PB4/PB15 |
| `Core/Src/stm32u5xx_it.c` | 修改 | EXTI0/EXTI1/EXTI4 handler; 删 EXTI15 |
| `Core/Inc/board_io.h` | 新建 | LED/POWER_CTL/按键 pin 宏 + 函数声明 |
| `Core/Src/board_io.c` | 新建 | BoardIO_Init + LED/PowerCtl/Button 实现 |
| `Core/Inc/app_acq.h` | 新建 | AppAcqStart/Stop/IsRunning 公开声明 |
| `Core/Inc/user_ctrl.h` | 新建 | UserCtrl_Init / StartUserCtrlTask 声明 |
| `Core/Src/user_ctrl.c` | 新建 | 按键状态机 + LED 闪烁 + 电源关机 |
| `MDK-ARM/SensorProj.uvprojx` | 修改 | 添加 board_io.c / user_ctrl.c |

---

## Task 1: 提交当前 USB 基线

**当前未提交的改动（已验证 USB 正常枚举）：**
- `main.c` — `HAL_NVIC_DisableIRQ(OTG_FS_IRQn)` 在 USB PCD init 后
- `app_freertos.c` — `HAL_NVIC_EnableIRQ` 在 USBD_Start 前；传感器任务已注释
- `usbd_wcid_app.h/.c` — Resp* stubs

**Files:**
- Modify: `Core/Src/main.c`
- Modify: `Core/Src/app_freertos.c`
- Modify: `Core/Inc/usbd_wcid_app.h`
- Modify: `Core/Src/usbd_wcid_app.c`

- [ ] **Step 1: 验证当前编译通过**

```
Keil: -rebuild SensorProj.uvprojx
Expected: 0 Error(s), 0 Warning(s)
```

- [ ] **Step 2: 提交**

```bash
git add Core/Src/main.c Core/Src/app_freertos.c \
        Core/Inc/usbd_wcid_app.h Core/Src/usbd_wcid_app.c
git commit -m "fix: USB ISR timing + Resp stubs — WCID enumerates on HW-v2 board"
```

---

## Task 2: HW-v2 传感器 GPIO 重映射

原理图确认引脚（全部来自截图）：
- LSM6DSOX: CS=PB1, INT1=PB0 (EXTI0, GPIOB)
- H3LIS100DL: CS=PA4, INT1=PA1 (EXTI1, GPIOA)
- QMA6100P: CS=PC5, INT1=PC4 (EXTI4, GPIOC)

**Files:**
- Modify: `BSP/Sensor/Inc/bsp_spi.h:43-76`
- Modify: `BSP/Sensor/Src/bsp_spi.c:88-89`
- Modify: `BSP/Sensor/Inc/h3lis100dl.h:57-59`
- Modify: `BSP/Sensor/Inc/qma6100p.h:97-99`
- Modify: `Core/Src/gpio.c`
- Modify: `Core/Src/stm32u5xx_it.c`
- Modify: `Core/Src/app_freertos.c:365-388` (EXTI callback)

- [ ] **Step 1: 更新 bsp_spi.h CS 引脚宏**

```c
/* BSP/Sensor/Inc/bsp_spi.h — 只改这三个 CS 块 */

/* LSM6DSOX on SPI1 */
#define LSM_SPI_CS_PIN           GPIO_PIN_1      /* HW-v2: PB1 */
#define LSM_SPI_CS_GPIO_PORT     GPIOB

/* H3LIS100DL on SPI2 */
#define H3_SPI2_CS_PIN           GPIO_PIN_4      /* HW-v2: PA4 */
#define H3_SPI2_CS_GPIO_PORT     GPIOA

/* QMA6100P on SPI2 */
#define QMA_SPI2_CS_PIN          GPIO_PIN_5      /* HW-v2: PC5 */
#define QMA_SPI2_CS_GPIO_PORT    GPIOC
```

- [ ] **Step 2: 更新 bsp_spi.c SPI1 CS 时钟**

在 `MX_SPI1_Init()` 的 GPIO 时钟使能区段（约 line 88）：

```c
/* 旧: __HAL_RCC_GPIOC_CLK_ENABLE();  // PC4 */
/* 新: */
__HAL_RCC_GPIOB_CLK_ENABLE();   /* PB1 = LSM CS on HW-v2 */
```

SPI2 的时钟使能（GPIOA + GPIOB + GPIOC）不用改，三个口都已启用。

- [ ] **Step 3: 更新 h3lis100dl.h INT 引脚**

```c
/* BSP/Sensor/Inc/h3lis100dl.h */
#define H3LIS100DL_INT_PIN          GPIO_PIN_1      /* HW-v2: PA1 */
#define H3LIS100DL_INT_GPIO_PORT    GPIOA
#define H3LIS100DL_INT_EXTI_IRQn    EXTI1_IRQn
```

- [ ] **Step 4: 更新 qma6100p.h INT 引脚**

```c
/* BSP/Sensor/Inc/qma6100p.h */
#define QMA6100P_INT_PIN            GPIO_PIN_4      /* HW-v2: PC4 */
#define QMA6100P_INT_GPIO_PORT      GPIOC
#define QMA6100P_INT_EXTI_IRQn      EXTI4_IRQn
```

- [ ] **Step 5: 重写 gpio.c MX_GPIO_Init**

```c
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* SDMMC1 DET (PC13) — input pullup（新板无卡检测，读值仅用于日志） */
  GPIO_InitStruct.Pin  = SDMMC1_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDMMC1_DET_GPIO_Port, &GPIO_InitStruct);

  /* LSM6DSOX INT1 → PB0, EXTI0, active-high, pull-down */
  GPIO_InitStruct.Pin   = GPIO_PIN_0;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* H3LIS100DL DRDY → PA1, EXTI1, active-high, pull-down */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* QMA6100P INT1 → PC4, EXTI4, active-high, pull-up（QMA INT 默认开漏） */
  GPIO_InitStruct.Pin  = GPIO_PIN_4;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
```

- [ ] **Step 6: 更新 stm32u5xx_it.c EXTI handlers**

删除旧的 `EXTI1_IRQHandler`（旧 LSM）、`EXTI4_IRQHandler`（旧 H3）、`EXTI15_IRQHandler`（旧 QMA），替换为：

```c
void EXTI0_IRQHandler(void)   /* LSM6DSOX INT1 → PB0 */
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)   /* H3LIS100DL DRDY → PA1 */
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI4_IRQHandler(void)   /* QMA6100P INT1 → PC4 */
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
```

- [ ] **Step 7: 更新 app_freertos.c EXTI rising callback**

```c
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)       /* LSM6DSOX INT1 → PB0 */
  {
    if (s_lsm_fifo_sem != NULL) { osSemaphoreRelease(s_lsm_fifo_sem); }
  }
  else if (GPIO_Pin == GPIO_PIN_1)  /* H3LIS100DL DRDY → PA1 */
  {
    if (s_h3_drdy_sem != NULL)  { osSemaphoreRelease(s_h3_drdy_sem);  }
  }
  else if (GPIO_Pin == GPIO_PIN_4)  /* QMA6100P INT1 → PC4 */
  {
    if (s_qma_fifo_sem != NULL) { osSemaphoreRelease(s_qma_fifo_sem); }
  }
}
```

- [ ] **Step 8: 重新启用传感器任务**

在 `MX_FREERTOS_Init()` 中将传感器任务恢复（去掉注释）：

```c
  lsm6dsoxTaskHandle  = osThreadNew(StartLsm6dsoxTask,  NULL, &lsm6dsoxTask_attributes);
  h3lis100dlTaskHandle = osThreadNew(StartH3lis100dlTask, NULL, &h3lis100dlTask_attributes);
  qma6100pTaskHandle  = osThreadNew(StartQma6100pTask,  NULL, &qma6100pTask_attributes);
  loggerTaskHandle    = osThreadNew(StartLoggerTask,    NULL, &loggerTask_attributes);
```

- [ ] **Step 9: 编译验证**

```
Keil: -rebuild SensorProj.uvprojx
Expected: 0 Error(s), 0 Warning(s)
```

- [ ] **Step 10: 烧录验证**

串口日志应出现：
```
[初始化] USB OTG FS PCD 初始化完成
[WCID] USBD_RegisterClass returned 0    ← 不再卡住
[WCID] USBD_Start returned 0
[WCID] init ok — 3 IN endpoints
[QMA6100P] init ok
[LSM6DSOX] init ok
[H3LIS100DL] init ok
```
设备管理器：通用串行总线设备 → Sensor WCID Bulk

- [ ] **Step 11: 提交**

```bash
git add BSP/Sensor/Inc/bsp_spi.h BSP/Sensor/Src/bsp_spi.c \
        BSP/Sensor/Inc/h3lis100dl.h BSP/Sensor/Inc/qma6100p.h \
        Core/Src/gpio.c Core/Src/stm32u5xx_it.c Core/Src/app_freertos.c
git commit -m "feat(hw-v2): GPIO remapping — LSM CS/INT PB1/PB0, H3 PA4/PA1, QMA PC5/PC4"
```

---

## Task 3: SD 卡检测适配（无硬件卡检测）

新板 push 式 SD 槽，无 CD 引脚。策略：始终尝试 SDMMC1 init；失败 = 无卡。

**Files:**
- Modify: `Core/Src/main.c:164-180`
- Modify: `Core/Src/sdmmc.c` — MX_SDMMC1_SD_Init 改为返回 HAL_StatusTypeDef

- [ ] **Step 1: 修改 sdmmc.c，让 SD init 返回状态而非调用 Error_Handler**

找到 `MX_SDMMC1_SD_Init()` 函数，修改签名和错误处理：

```c
/* Core/Src/sdmmc.c */
HAL_StatusTypeDef MX_SDMMC1_SD_Init(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv            = 4;
  /* 原来: if (HAL_SD_Init(&hsd1) != HAL_OK) { Error_Handler(); }
   * 新板无卡时返回 HAL_ERROR 而不是死机 */
  return HAL_SD_Init(&hsd1);
}
```

并同步更新 `Core/Inc/sdmmc.h` 中的声明：

```c
HAL_StatusTypeDef MX_SDMMC1_SD_Init(void);
```

- [ ] **Step 2: 修改 main.c SD 初始化块**

```c
/* Core/Src/main.c — 替换原来的 if(SDMMC1_IsCardDetected()) 块 */

/* Push-type slot: no hardware detect. Try init; failure = no card. */
{
  HAL_StatusTypeDef sd_st = MX_SDMMC1_SD_Init();
  if (sd_st == HAL_OK)
  {
    printf("[初始化] SDMMC1 初始化完成 (SD卡已就绪)\r\n");
  }
  else
  {
    printf("[初始化] 未检测到SD卡 (init=0x%X)\r\n", (unsigned)sd_st);
  }
}
```

同时删除或注释掉原来读 PC13 的 printf 行（因为 PC13 在新板不是 SD DET）。

- [ ] **Step 3: 检查所有调用 MX_SDMMC1_SD_Init() 的地方是否需要更新**

```bash
grep -rn "MX_SDMMC1_SD_Init" Core/ --include="*.c"
```

若 MSC boot 分支也调用，需同样处理（检查返回值，失败则 reset）。

- [ ] **Step 4: 编译验证**

```
Keil: -rebuild
Expected: 0 Error(s), 0 Warning(s)
```

- [ ] **Step 5: 烧录验证**

无 SD：日志打 `[初始化] 未检测到SD卡 (init=0x...)` 且系统正常运行（不崩溃）。  
有 SD（上电前已插入）：日志打 `[初始化] SDMMC1 初始化完成`。

- [ ] **Step 6: 提交**

```bash
git add Core/Src/sdmmc.c Core/Inc/sdmmc.h Core/Src/main.c
git commit -m "fix: SD detection — always try init, no hardware detect on push-type slot"
```

---

## Task 4: board_io 外设驱动

**Files:**
- Create: `Core/Inc/board_io.h`
- Create: `Core/Src/board_io.c`
- Modify: `Core/Src/main.c` — 添加 `#include "board_io.h"` 和 `BoardIO_Init()` 调用
- Modify: `MDK-ARM/SensorProj.uvprojx` — 添加 board_io.c

- [ ] **Step 1: 创建 board_io.h**

```c
/* Core/Inc/board_io.h */
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
```

- [ ] **Step 2: 创建 board_io.c**

```c
/* Core/Src/board_io.c */
#include "board_io.h"

void BoardIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* POWER_CTL (PB7) — output, immediately HIGH to latch battery supply */
  g.Pin   = BOARD_POWER_CTL_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_POWER_CTL_PORT, &g);
  HAL_GPIO_WritePin(BOARD_POWER_CTL_PORT, BOARD_POWER_CTL_PIN, GPIO_PIN_SET);

  /* LED (PB12) — output, start off */
  g.Pin = BOARD_LED_PIN;
  HAL_GPIO_Init(BOARD_LED_PORT, &g);
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_RESET);

  /* User button (PC15) — input, hardware pull-up already on board */
  g.Pin  = BOARD_USER_BTN_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOARD_USER_BTN_PORT, &g);

  /* Power button (PC14) — input */
  g.Pin = BOARD_PWR_BTN_PIN;
  HAL_GPIO_Init(BOARD_PWR_BTN_PORT, &g);
}

void LED_Set(uint8_t on)
{
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED_Toggle(void)
{
  HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);
}

void PowerCtl_Set(uint8_t on)
{
  HAL_GPIO_WritePin(BOARD_POWER_CTL_PORT, BOARD_POWER_CTL_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t UserBtn_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(BOARD_USER_BTN_PORT, BOARD_USER_BTN_PIN)
          == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t PwrBtn_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(BOARD_PWR_BTN_PORT, BOARD_PWR_BTN_PIN)
          == GPIO_PIN_RESET) ? 1U : 0U;
}
```

- [ ] **Step 3: 在 main.c 添加 include 和调用**

在 `#include` 区添加：
```c
#include "board_io.h"
```

在 `MX_USART1_UART_Init()` 之后（GPIO/ICACHE/UART 完成后）立即调用：
```c
BoardIO_Init();  /* HW-v2: latch POWER_CTL HIGH + init LED/buttons */
```

- [ ] **Step 4: 在 SensorProj.uvprojx 中添加 board_io.c**

在 Keil 项目文件中，在 `Core/Src` 文件组里添加：
```xml
<File>
  <FileName>board_io.c</FileName>
  <FileType>1</FileType>
  <FilePath>../Core/Src/board_io.c</FilePath>
</File>
```

- [ ] **Step 5: 编译验证**

```
Keil: -rebuild
Expected: 0 Error(s), 0 Warning(s)
```

- [ ] **Step 6: 提交**

```bash
git add Core/Inc/board_io.h Core/Src/board_io.c Core/Src/main.c MDK-ARM/SensorProj.uvprojx
git commit -m "feat: board_io — LED/POWER_CTL/buttons for HW-v2"
```

---

## Task 5: AppAcq 公开接口

`AppAcqStart/Stop/IsRunning` 在 `app_freertos.c` 中是 `static`，`user_ctrl.c` 需要调用它们。

**Files:**
- Create: `Core/Inc/app_acq.h`
- Modify: `Core/Src/app_freertos.c` — 去掉三个函数的 `static`

- [ ] **Step 1: 创建 app_acq.h**

```c
/* Core/Inc/app_acq.h */
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
```

- [ ] **Step 2: 在 app_freertos.c 去掉三个函数的 static**

搜索并修改：
```c
/* 将以下三个函数的 static 关键字删除 */
/* 旧: static uint32_t AppAcqIsRunning(void) */
uint32_t AppAcqIsRunning(void)

/* 旧: static uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms) */
uint32_t AppAcqStart(uint8_t sink, uint32_t duration_ms)

/* 旧: static uint32_t AppAcqStop(void) */
uint32_t AppAcqStop(void)
```

在 app_freertos.c 顶部 includes 区加：
```c
#include "app_acq.h"
```

- [ ] **Step 3: 编译验证**

```
Keil: -rebuild
Expected: 0 Error(s), 0 Warning(s)
```

- [ ] **Step 4: 提交**

```bash
git add Core/Inc/app_acq.h Core/Src/app_freertos.c
git commit -m "feat: expose AppAcqStart/Stop/IsRunning as public API"
```

---

## Task 6: user_ctrl 任务

**Files:**
- Create: `Core/Inc/user_ctrl.h`
- Create: `Core/Src/user_ctrl.c`
- Modify: `Core/Src/app_freertos.c` — 添加 UserCtrl task 到 MX_FREERTOS_Init
- Modify: `MDK-ARM/SensorProj.uvprojx` — 添加 user_ctrl.c

- [ ] **Step 1: 创建 user_ctrl.h**

```c
/* Core/Inc/user_ctrl.h */
#ifndef USER_CTRL_H
#define USER_CTRL_H

#include "cmsis_os2.h"

void UserCtrl_Init(void);
void StartUserCtrlTask(void *argument);

#endif /* USER_CTRL_H */
```

- [ ] **Step 2: 创建 user_ctrl.c**

```c
/* Core/Src/user_ctrl.c
 *
 * 用户按键 (PC15, active-low, 10K pull-up):
 *   按住 >= 1000ms 且未采集 → AppAcqStart(SD, 0)（无限时长）
 *   按住 >= 2000ms 且采集中 → AppAcqStop() + 存 SD
 *
 * 电源按键 (PC14, active-low, 10K pull-up):
 *   按住 >= 3000ms → AppAcqStop() → 等1.2s → PowerCtl_Set(0) → 断电
 *
 * LED (PB12):
 *   采集中：每 50~150ms 随机翻转（高频无规则闪烁）
 *   非采集中：常灭
 */
#include "user_ctrl.h"
#include "board_io.h"
#include "app_acq.h"
#include "cmsis_os2.h"
#include <stdlib.h>

#define UC_USER_SHORT_MS    1000U
#define UC_USER_LONG_MS     2000U
#define UC_PWR_LONG_MS      3000U
#define UC_LED_MIN_MS         50U
#define UC_LED_MAX_MS        150U
#define UC_POLL_MS            20U

typedef enum { UC_BTN_IDLE = 0, UC_BTN_PRESSED, UC_BTN_HANDLED } BtnState_t;

static BtnState_t s_user_state = UC_BTN_IDLE;
static BtnState_t s_pwr_state  = UC_BTN_IDLE;
static uint32_t   s_user_tick  = 0U;
static uint32_t   s_pwr_tick   = 0U;
static uint32_t   s_led_next   = 0U;

void UserCtrl_Init(void) { /* no RTOS objects needed */ }

void StartUserCtrlTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    /* ── 用户按键 ── */
    uint8_t user_dn = UserBtn_IsPressed();
    if (user_dn)
    {
      if (s_user_state == UC_BTN_IDLE)
      {
        s_user_state = UC_BTN_PRESSED;
        s_user_tick  = now;
      }
      else if (s_user_state == UC_BTN_PRESSED)
      {
        uint32_t held = now - s_user_tick;
        if ((held >= UC_USER_LONG_MS) && (AppAcqIsRunning() != 0U))
        {
          AppAcqStop();
          LED_Set(0U);
          s_user_state = UC_BTN_HANDLED;
        }
        else if ((held >= UC_USER_SHORT_MS) && (AppAcqIsRunning() == 0U))
        {
          AppAcqStart(APP_ACQ_SINK_SD, 0U);   /* 0 = 无限时长 */
          s_user_state = UC_BTN_HANDLED;
        }
      }
    }
    else
    {
      s_user_state = UC_BTN_IDLE;
    }

    /* ── 电源按键 ── */
    uint8_t pwr_dn = PwrBtn_IsPressed();
    if (pwr_dn)
    {
      if (s_pwr_state == UC_BTN_IDLE)
      {
        s_pwr_state = UC_BTN_PRESSED;
        s_pwr_tick  = now;
      }
      else if (s_pwr_state == UC_BTN_PRESSED)
      {
        if ((now - s_pwr_tick) >= UC_PWR_LONG_MS)
        {
          if (AppAcqIsRunning() != 0U)
          {
            AppAcqStop();
            osDelay(1200U);   /* 等 logger 关闭文件 */
          }
          LED_Set(0U);
          PowerCtl_Set(0U);            /* 拉低 POWER_CTL → 断电 */
          for (;;) { osDelay(1000U); } /* 保险死循环 */
        }
      }
    }
    else
    {
      s_pwr_state = UC_BTN_IDLE;
    }

    /* ── LED 闪烁 ── */
    if (AppAcqIsRunning() != 0U)
    {
      if ((int32_t)(now - s_led_next) >= 0)
      {
        LED_Toggle();
        uint32_t iv = UC_LED_MIN_MS +
                      (uint32_t)((uint32_t)rand() %
                                 (UC_LED_MAX_MS - UC_LED_MIN_MS + 1U));
        s_led_next = now + iv;
      }
    }
    else
    {
      LED_Set(0U);
    }

    osDelay(UC_POLL_MS);
  }
}
```

- [ ] **Step 3: 在 app_freertos.c MX_FREERTOS_Init 中添加 UserCtrl task**

在 includes 区加：
```c
#include "user_ctrl.h"
#include "board_io.h"
```

在 `MX_FREERTOS_Init()` 中，传感器任务创建之后加：
```c
/* UserCtrl task: 按键轮询 + LED + 电源关机 */
static const osThreadAttr_t userCtrlTask_attributes = {
  .name       = "userCtrlTask",
  .priority   = (osPriority_t)osPriorityBelowNormal,
  .stack_size = 512 * 4
};
osThreadNew(StartUserCtrlTask, NULL, &userCtrlTask_attributes);
```

- [ ] **Step 4: 在 SensorProj.uvprojx 中添加 user_ctrl.c**

```xml
<File>
  <FileName>user_ctrl.c</FileName>
  <FileType>1</FileType>
  <FilePath>../Core/Src/user_ctrl.c</FilePath>
</File>
```

- [ ] **Step 5: 编译验证**

```
Keil: -rebuild
Expected: 0 Error(s), 0 Warning(s)
```

- [ ] **Step 6: 提交**

```bash
git add Core/Inc/user_ctrl.h Core/Src/user_ctrl.c \
        Core/Src/app_freertos.c MDK-ARM/SensorProj.uvprojx
git commit -m "feat: user_ctrl — button SM + LED blink + power-off"
```

---

## Task 7: 集成验证

- [ ] **Step 1: 全量重编译**

```
删除 MDK-ARM\SensorProj\ 目录，再 -rebuild
Expected: 0 Error(s), 0 Warning(s)
Program Size 应在 130KB 左右
```

- [ ] **Step 2: USB 枚举验证**

烧录 → 设备管理器 → 通用串行总线设备 → "Sensor WCID Bulk" ✓

- [ ] **Step 3: 传感器 SPI 验证**

串口日志：
```
[LSM6DSOX] init ok
[H3LIS100DL] init ok
[QMA6100P] init ok
```
三者均 OK 说明 HW-v2 CS/INT 引脚映射正确。

- [ ] **Step 4: SD 采集验证**

电池供电（POWER_CTL 生效），插入 SD 卡后上电：
1. 短按用户按键(PC15) ≥1s → LED 高频闪烁开始 → SD 创建文件夹/csv
2. 长按用户按键(PC15) ≥2s → LED 灭 → SD 文件关闭
3. 取出 SD 验证 csv 数据行数完整

- [ ] **Step 5: 电源关机验证（电池供电）**

长按电源按键(PC14) 3s → LED 灭 → 设备断电

- [ ] **Step 6: 最终提交**

```bash
git add -A
git commit -m "feat(hw-v2): full integration — GPIO remapping + SD + board_io + user_ctrl"
git push origin main
```

---

## 已知遗留问题（本计划不处理）

| 问题 | 说明 |
|---|---|
| USB BUS RESET 后 SETUP 包不到 | 已确认非时钟、非软件问题；怀疑新板 USB 走线信号质量；仍可通过旧板 WCID 进行数据上传 |
| PC13 新板接 LIS2MDL INT | 当前代码读 PC13 仅用于日志，不影响功能 |
| banner 字符串仍显示旧引脚 | 低优先级，可后续更新 |
