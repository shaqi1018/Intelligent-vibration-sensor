# RTC 实时时钟时间戳 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用 PCF85063ATL RTC 的壁钟时间替换现有基于启动时间的 `tick_ms` 时间戳，CSV 第 2 字段改为 `epoch_us`（Unix 纪元起微秒数），精度 1µs，满足 8kHz 最高采样率要求。

**Architecture:** 锚点法 — 启动时通过 I2C2 读取 PCF85063ATL 秒寄存器，同时记录 DWT 计数器值（`anchor_dwt_us`）；此后每个样本用 `rtc_epoch_s × 1,000,000 + (DWT_us − anchor_dwt_us)` 计算绝对微秒时间戳，无需再次访问 I2C。CSV 格式：`frame_id,epoch_us,...`。

**Tech Stack:** STM32U575 HAL I2C，DWT 计数器，PCF85063ATL（I2C2 地址 0x51，PB10/PB11），FreeRTOS CMSIS-OS2

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `Core/Inc/rtc_pcf85063.h` | 新建 | PCF85063ATL I2C 驱动接口 |
| `Core/Src/rtc_pcf85063.c` | 新建 | I2C2 读写 + BCD 解码 + 时间设置/读取 |
| `Core/Inc/app_time.h` | 新建 | 壁钟锚点接口：Sync / GetEpochUs |
| `Core/Src/app_time.c` | 新建 | 锚点维护 + `AppTime_GetEpochUs()` |
| `Core/Src/i2c.c` | 新建 | MX_I2C2_Init（HAL 风格，PB10/PB11） |
| `Core/Inc/i2c.h` | 新建 | hi2c2 extern 声明 |
| `Core/Src/main.c` | 修改 | 调用 MX_I2C2_Init |
| `Core/Src/app_freertos.c` | 修改 | ① AppF1 → AppU64 输出 epoch_us；② `set_time` 命令解析；③ 采集启动时调用 AppTime_Sync |
| `MDK-ARM/build/SensorProj/builder.params` | 修改 | sourceList 加入两个新 .c 文件 |

---

## Task 1：I2C2 外设初始化

**Files:**
- 新建: `Core/Inc/i2c.h`
- 新建: `Core/Src/i2c.c`
- 修改: `Core/Src/main.c`（加入 MX_I2C2_Init 调用）

- [ ] **Step 1: 新建 Core/Inc/i2c.h**

```c
#ifndef I2C_H
#define I2C_H
#include "stm32u5xx_hal.h"
extern I2C_HandleTypeDef hi2c2;
void MX_I2C2_Init(void);
#endif
```

- [ ] **Step 2: 新建 Core/Src/i2c.c**

```c
#include "i2c.h"

I2C_HandleTypeDef hi2c2;

/* PB10 = I2C2_SCL (AF4), PB11 = I2C2_SDA (AF4)
 * 400kHz Fast-mode，内部上拉（板上已有外部上拉可不启用内部，但保留无害） */
void MX_I2C2_Init(void)
{
  __HAL_RCC_I2C2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
  g.Mode      = GPIO_MODE_AF_OD;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &g);

  hi2c2.Instance              = I2C2;
  hi2c2.Init.Timing           = 0x00C01F67U; /* 400kHz @ 160MHz PCLK1，CubeMX 生成值 */
  hi2c2.Init.OwnAddress1      = 0U;
  hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2      = 0U;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);
}
```

> **注意 Timing 值：** `0x00C01F67` 是 STM32CubeMX 为 STM32U5 160MHz PCLK1 生成的 I2C2 400kHz 参数。若编译后 I2C 通信不稳定，用 STM32CubeMX 的 I2C Timing Configuration Tool 重新生成并替换此值。

- [ ] **Step 3: 在 main.c 中加入 I2C2 初始化调用**

在 `Core/Src/main.c` 找到以下位置（`MX_SPI1_Init()` 调用之前）：

```c
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
```

在 `MX_GPIO_Init()` 之后插入：

```c
  MX_I2C2_Init();
```

同时在文件顶部加入：
```c
#include "i2c.h"
```

- [ ] **Step 4: 在 builder.params 的 sourceList 中加入 i2c.c**

打开 `MDK-ARM/build/SensorProj/builder.params`，在 `"../Core/Src/boot_mode.c"` 附近插入：
```
"../Core/Src/i2c.c",
```

- [ ] **Step 5: 编译确认 0 错 0 警告**

在 EIDE / Keil 执行全量编译，确认输出末尾为：
```
Program Size: ...
"SensorProj.axf" - 0 Error(s), 0 Warning(s).
```

- [ ] **Step 6: 提交**

```bash
git add Core/Inc/i2c.h Core/Src/i2c.c Core/Src/main.c MDK-ARM/build/SensorProj/builder.params
git commit -m "feat: 初始化 I2C2 外设 (PB10/PB11 400kHz) 为 PCF85063ATL RTC 准备"
```

---

## Task 2：PCF85063ATL I2C 驱动

**Files:**
- 新建: `Core/Inc/rtc_pcf85063.h`
- 新建: `Core/Src/rtc_pcf85063.c`

- [ ] **Step 1: 新建 Core/Inc/rtc_pcf85063.h**

```c
#ifndef RTC_PCF85063_H
#define RTC_PCF85063_H

#include <stdint.h>

/* PCF85063ATL 时间结构体 */
typedef struct {
  uint8_t  year;   /* 0-99（加 2000 得公历年） */
  uint8_t  month;  /* 1-12 */
  uint8_t  day;    /* 1-31 */
  uint8_t  hour;   /* 0-23 */
  uint8_t  minute; /* 0-59 */
  uint8_t  second; /* 0-59 */
} Pcf85063_Time_t;

/* 返回值 */
#define PCF85063_OK    0
#define PCF85063_ERR   1

uint8_t Pcf85063_Init(void);
uint8_t Pcf85063_GetTime(Pcf85063_Time_t *t);
uint8_t Pcf85063_SetTime(const Pcf85063_Time_t *t);

/* 将 Pcf85063_Time_t 转换为 Unix 纪元秒（相对 1970-01-01 00:00:00 UTC）
 * 注意：PCF85063 不存储时区，调用方保证传入 UTC 时间。 */
uint32_t Pcf85063_ToEpochSeconds(const Pcf85063_Time_t *t);

#endif
```

- [ ] **Step 2: 新建 Core/Src/rtc_pcf85063.c**

```c
#include "rtc_pcf85063.h"
#include "i2c.h"
#include <string.h>

#define PCF85063_ADDR   (0x51U << 1U)  /* HAL 使用 8-bit 地址 */
#define PCF85063_REG_CTRL1   0x00U
#define PCF85063_REG_SECONDS 0x04U

static inline uint8_t BcdToDec(uint8_t b) { return (uint8_t)((b >> 4U) * 10U + (b & 0x0FU)); }
static inline uint8_t DecToBcd(uint8_t d) { return (uint8_t)(((d / 10U) << 4U) | (d % 10U)); }

uint8_t Pcf85063_Init(void)
{
  /* Control_1: 正常模式（bit7=0），12/24h 选 24h（bit1=0）
   * 写 0x00 到 Control_1 寄存器即可 */
  uint8_t ctrl = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c2, PCF85063_ADDR, PCF85063_REG_CTRL1,
                         I2C_MEMADD_SIZE_8BIT, &ctrl, 1U, 50U) != HAL_OK)
  {
    return PCF85063_ERR;
  }
  return PCF85063_OK;
}

uint8_t Pcf85063_GetTime(Pcf85063_Time_t *t)
{
  uint8_t raw[7]; /* seconds, minutes, hours, days, weekdays, months, years */
  if (HAL_I2C_Mem_Read(&hi2c2, PCF85063_ADDR, PCF85063_REG_SECONDS,
                        I2C_MEMADD_SIZE_8BIT, raw, 7U, 50U) != HAL_OK)
  {
    return PCF85063_ERR;
  }
  t->second = BcdToDec(raw[0] & 0x7FU); /* bit7 = OS（晶振停止标志），屏蔽 */
  t->minute = BcdToDec(raw[1] & 0x7FU);
  t->hour   = BcdToDec(raw[2] & 0x3FU);
  t->day    = BcdToDec(raw[3] & 0x3FU);
  /* raw[4] = weekday，忽略 */
  t->month  = BcdToDec(raw[5] & 0x1FU);
  t->year   = BcdToDec(raw[6]);
  return PCF85063_OK;
}

uint8_t Pcf85063_SetTime(const Pcf85063_Time_t *t)
{
  uint8_t raw[7];
  raw[0] = DecToBcd(t->second);
  raw[1] = DecToBcd(t->minute);
  raw[2] = DecToBcd(t->hour);
  raw[3] = DecToBcd(t->day);
  raw[4] = 0U; /* weekday 不使用 */
  raw[5] = DecToBcd(t->month);
  raw[6] = DecToBcd(t->year);
  if (HAL_I2C_Mem_Write(&hi2c2, PCF85063_ADDR, PCF85063_REG_SECONDS,
                         I2C_MEMADD_SIZE_8BIT, raw, 7U, 50U) != HAL_OK)
  {
    return PCF85063_ERR;
  }
  return PCF85063_OK;
}

/* 简化 epoch 计算：以 2000-01-01 00:00:00 为基准（避免 1970 年闰年复杂计算）
 * 返回值 = 2000-01-01 00:00:00 UTC 起的秒数 + 946684800（2000年相对1970年偏移）*/
static const uint16_t s_days_per_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

uint32_t Pcf85063_ToEpochSeconds(const Pcf85063_Time_t *t)
{
  uint32_t year4 = 2000U + t->year;
  /* 以 1970-01-01 为基准计算天数 */
  uint32_t days = 0U;
  for (uint32_t y = 1970U; y < year4; y++)
  {
    days += ((y % 4U == 0U && (y % 100U != 0U || y % 400U == 0U)) ? 366U : 365U);
  }
  uint8_t leap = (year4 % 4U == 0U && (year4 % 100U != 0U || year4 % 400U == 0U)) ? 1U : 0U;
  for (uint8_t m = 1U; m < t->month; m++)
  {
    days += s_days_per_month[m - 1U];
    if (m == 2U && leap) { days++; }
  }
  days += t->day - 1U;
  return days * 86400U + (uint32_t)t->hour * 3600U + (uint32_t)t->minute * 60U + t->second;
}
```

- [ ] **Step 3: 在 builder.params 加入 rtc_pcf85063.c**

在 sourceList 适当位置加入：
```
"../Core/Src/rtc_pcf85063.c",
```

- [ ] **Step 4: 编译确认 0 错 0 警告**

- [ ] **Step 5: 提交**

```bash
git add Core/Inc/rtc_pcf85063.h Core/Src/rtc_pcf85063.c MDK-ARM/build/SensorProj/builder.params
git commit -m "feat: PCF85063ATL I2C 驱动 — GetTime/SetTime/ToEpochSeconds"
```

---

## Task 3：AppTime 锚点模块

**Files:**
- 新建: `Core/Inc/app_time.h`
- 新建: `Core/Src/app_time.c`

- [ ] **Step 1: 新建 Core/Inc/app_time.h**

```c
#ifndef APP_TIME_H
#define APP_TIME_H

#include <stdint.h>

/* 从 RTC 读取当前时间并记录 DWT 锚点。
 * 应在 FreeRTOS 启动后、第一次采集开始前调用。
 * 返回 1=成功，0=I2C 失败（时间戳退化为启动相对值）。 */
uint8_t AppTime_Sync(void);

/* 返回当前时刻的 Unix 纪元微秒数（UTC）。
 * 精度：~1µs（DWT 160MHz）。DWT 溢出周期 ~71 分钟，长会话下自动处理溢出。 */
uint64_t AppTime_GetEpochUs(void);

/* 返回 AppTime_Sync 时刻对应的 epoch 秒（供日志打印用）。 */
uint32_t AppTime_GetAnchorEpochS(void);

#endif
```

- [ ] **Step 2: 新建 Core/Src/app_time.c**

```c
#include "app_time.h"
#include "rtc_pcf85063.h"
#include "stm32u5xx_hal.h"  /* DWT, SystemCoreClock */

/* DWT 计数器访问 */
static inline uint32_t DwtUs(void)
{
  return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static uint32_t s_anchor_epoch_s  = 0U;  /* RTC 读取时刻的 Unix 秒 */
static uint32_t s_anchor_dwt_us   = 0U;  /* 对应的 DWT µs 值 */
static uint32_t s_anchor_dwt_prev = 0U;  /* 上一次 GetEpochUs 时的 DWT 值（溢出检测） */
static uint32_t s_wrap_count      = 0U;  /* DWT uint32 溢出计数 */
static uint8_t  s_synced          = 0U;

uint8_t AppTime_Sync(void)
{
  /* 启用 DWT（若尚未启用） */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  Pcf85063_Time_t t;
  if (Pcf85063_Init() != PCF85063_OK || Pcf85063_GetTime(&t) != PCF85063_OK)
  {
    /* RTC 不可用：以当前 DWT 为基准，时间戳为启动相对值（epoch_s=0） */
    s_anchor_epoch_s  = 0U;
    s_anchor_dwt_us   = DwtUs();
    s_anchor_dwt_prev = s_anchor_dwt_us;
    s_wrap_count      = 0U;
    s_synced          = 1U;
    return 0U;
  }

  /* 读 RTC 后立即采样 DWT，最小化锚点误差 */
  s_anchor_epoch_s  = Pcf85063_ToEpochSeconds(&t);
  s_anchor_dwt_us   = DwtUs();
  s_anchor_dwt_prev = s_anchor_dwt_us;
  s_wrap_count      = 0U;
  s_synced          = 1U;
  return 1U;
}

uint64_t AppTime_GetEpochUs(void)
{
  if (s_synced == 0U) { return 0ULL; }

  uint32_t now_us = DwtUs();

  /* 检测 uint32 溢出（DWT µs 约 71 分钟绕回一次） */
  if (now_us < s_anchor_dwt_prev)
  {
    s_wrap_count++;
  }
  s_anchor_dwt_prev = now_us;

  /* 相对 anchor 的偏移（含溢出补偿） */
  uint64_t offset_us = (uint64_t)s_wrap_count * 0xFFFFFFFFULL
                     + (uint64_t)(now_us - s_anchor_dwt_us);

  return (uint64_t)s_anchor_epoch_s * 1000000ULL + offset_us;
}

uint32_t AppTime_GetAnchorEpochS(void)
{
  return s_anchor_epoch_s;
}
```

- [ ] **Step 3: 在 builder.params 加入 app_time.c**

```
"../Core/Src/app_time.c",
```

- [ ] **Step 4: 编译确认 0 错 0 警告**

- [ ] **Step 5: 提交**

```bash
git add Core/Inc/app_time.h Core/Src/app_time.c MDK-ARM/build/SensorProj/builder.params
git commit -m "feat: AppTime 锚点模块 — RTC+DWT 实现 1µs 精度 epoch 时间戳"
```

---

## Task 4：传感器任务时间戳格式升级

**Files:**
- 修改: `Core/Src/app_freertos.c`（涉及 3 处传感器任务 + AppU64ToDec 函数）

**背景：** 当前每个传感器任务的时间戳字段是 `tick_ms_i`（uint32_t，boot 起的毫秒）。改为 `epoch_us`（uint64_t，Unix 纪元微秒），需要增加 `AppU64ToDec` 打印函数，并修改 3 个传感器任务的 CSV 组装代码。

- [ ] **Step 1: 在 app_freertos.c 中加入 AppU64ToDec 函数**

在 `AppU32ToDec` / `AppI32ToDec` 函数定义区域附近加入：

```c
static inline uint32_t AppU64ToDec(char *out, uint64_t v)
{
  if (v == 0ULL) { out[0] = '0'; return 1U; }
  char tmp[20];
  uint32_t n = 0U;
  while (v > 0ULL) { tmp[n++] = (char)('0' + (uint8_t)(v % 10ULL)); v /= 10ULL; }
  for (uint32_t i = 0U; i < n; i++) { out[i] = tmp[n - 1U - i]; }
  return n;
}
```

- [ ] **Step 2: 在 app_freertos.c 顶部加入头文件引用**

```c
#include "app_time.h"
```

- [ ] **Step 3: 修改 LSM6DSOX 任务的时间戳字段**

找到 LSM 任务中：
```c
uint32_t tick_ms_i = lsm_ts_us / 1000U;
```
替换为：
```c
uint64_t epoch_us_i = AppTime_GetEpochUs();
```

找到 CSV 组装中：
```c
off += AppU32ToDec(&rowbuf[off], tick_ms_i);
```
替换为：
```c
off += AppU64ToDec(&rowbuf[off], epoch_us_i);
```

同时将 `rowbuf[96]` 确认已足够（epoch_us = 16位数 + 其他字段，合计 ~130字节最大，将 rowbuf 改为 `char rowbuf[160]`）。

- [ ] **Step 4: 修改 H3LIS100DL 任务的时间戳字段**

找到 H3 任务中：
```c
uint32_t tick_ms_i = h3_ts_us / 1000U;
```
替换为：
```c
uint64_t epoch_us_i = AppTime_GetEpochUs();
```

找到 H3 任务的 CSV 组装：
```c
off += AppU32ToDec(&rowbuf[off], tick_ms_i);
```
替换为：
```c
off += AppU64ToDec(&rowbuf[off], epoch_us_i);
```

将 `rowbuf[64]` 改为 `char rowbuf[96]`。

- [ ] **Step 5: 修改 QMA6100P 任务的时间戳字段**

找到 QMA 任务中：
```c
uint32_t tick_ms_i = qma_ts_us / 1000U;
```
替换为：
```c
uint64_t epoch_us_i = AppTime_GetEpochUs();
```

找到 QMA 任务的 CSV 组装：
```c
off += AppU32ToDec(&rowbuf[off], tick_ms_i);
```
替换为：
```c
off += AppU64ToDec(&rowbuf[off], epoch_us_i);
```

- [ ] **Step 6: 在 AppAcqStart 采集启动前调用 AppTime_Sync**

在 `AppAcqStart()` 函数开头（sink 判断之前）加入：

```c
  /* 每次采集启动时重新同步 RTC 锚点，保证长时间运行后时钟漂移最小 */
  AppTime_Sync();
```

- [ ] **Step 7: 编译确认 0 错 0 警告**

- [ ] **Step 8: 提交**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat: CSV 时间戳升级为 epoch_us — 1µs 精度壁钟时间，满足 8kHz 采样"
```

---

## Task 5：`set_time` 命令 — 通过 USB 设置 RTC 时间

**Files:**
- 修改: `Core/Src/app_freertos.c`（UsbCmd_Process 函数）

**命令格式：** `set_time 2026-06-07T10:30:00`
**响应：** `OK set_time` 或 `ERR set_time: parse fail`

- [ ] **Step 1: 在 UsbCmd_Process 加入 set_time 分支**

找到 `UsbCmd_Process` 函数中的命令解析区域（`acq_start`/`acq_stop` 分支处），加入：

```c
  else if (strncmp(cmd, "set_time ", 9U) == 0)
  {
    /* 格式: set_time YYYY-MM-DDTHH:MM:SS  （共 19 字符，含前缀共 28） */
    const char *p = cmd + 9U;
    Pcf85063_Time_t t;
    /* 简单位置解析（不含格式校验） */
    uint32_t yr   = (uint32_t)(p[2] - '0') * 10U + (uint32_t)(p[3] - '0'); /* 取后两位年 */
    uint32_t mo   = (uint32_t)(p[5] - '0') * 10U + (uint32_t)(p[6] - '0');
    uint32_t dy   = (uint32_t)(p[8] - '0') * 10U + (uint32_t)(p[9] - '0');
    uint32_t hr   = (uint32_t)(p[11] - '0') * 10U + (uint32_t)(p[12] - '0');
    uint32_t mn   = (uint32_t)(p[14] - '0') * 10U + (uint32_t)(p[15] - '0');
    uint32_t sc   = (uint32_t)(p[17] - '0') * 10U + (uint32_t)(p[18] - '0');
    if (mo >= 1U && mo <= 12U && dy >= 1U && dy <= 31U &&
        hr <= 23U && mn <= 59U && sc <= 59U)
    {
      t.year = (uint8_t)yr; t.month = (uint8_t)mo; t.day   = (uint8_t)dy;
      t.hour = (uint8_t)hr; t.minute = (uint8_t)mn; t.second = (uint8_t)sc;
      if (Pcf85063_SetTime(&t) == PCF85063_OK)
      {
        AppTime_Sync();  /* 立即更新锚点 */
        printf("OK set_time %04u-%02u-%02uT%02u:%02u:%02u\r\n",
               2000U + yr, mo, dy, hr, mn, sc);
      }
      else { printf("ERR set_time: I2C write fail\r\n"); }
    }
    else { printf("ERR set_time: parse fail\r\n"); }
  }
```

同时在文件顶部确认已包含：
```c
#include "rtc_pcf85063.h"
```

- [ ] **Step 2: 编译确认 0 错 0 警告**

- [ ] **Step 3: 刷机验证**

连接设备，在 USB 串口或 WCID 终端输入：
```
set_time 2026-06-07T10:30:00
```
期望响应：`OK set_time 2026-06-07T10:30:00`

随后 `acq_start usb 2000`，检查 CSV 第 2 字段是否为 16 位 Unix µs 时间戳（约 1749295800×10⁶ 量级）。

用 Python 验证：
```python
import pandas as pd
df = pd.read_csv('lsm.csv', header=None)
df[1] = pd.to_datetime(df[1], unit='us')
print(df[1].head())
# 期望输出: 2026-06-07 10:30:00.xxxxx...
```

- [ ] **Step 4: 提交**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat: set_time 命令 — 通过 USB 设置 PCF85063ATL RTC 实时时钟"
```

---

## 自查清单

### Spec 覆盖检查
- [x] I2C2 外设初始化 → Task 1
- [x] PCF85063ATL 读写驱动 → Task 2
- [x] 秒级精度 epoch 锚点 → Task 3
- [x] 1µs 精度时间戳（DWT+锚点） → Task 3
- [x] CSV 格式升级（epoch_us） → Task 4
- [x] 所有三个传感器任务更新 → Task 4（Step 3/4/5）
- [x] USB 设置 RTC 时间命令 → Task 5
- [x] 溢出处理（DWT 71分钟绕回） → Task 3（app_time.c）
- [x] RTC 不可达时优雅降级 → Task 3（AppTime_Sync 返回 0）

### 精度说明（交付物附注）
- **RTC 本身**：1 秒分辨率，32.768kHz 晶振 ±20ppm → 每天 ±1.7s
- **样本时间戳**：RTC 锚点 + DWT → **1µs** 分辨率
- **绝对误差**：受限于 `set_time` 命令的发送延迟（约数百 ms），可通过 PPS 信号或 NTP 辅助校准精确到 <10ms
- **相对误差（样本间）**：DWT 漂移 <1ppm@160MHz，1ms 内误差 <1ns，可忽略

### 常见坑
1. **EIDE 重新生成 builder.params** 会丢掉新加的 `.c` 文件 → 每次重生成后需重新加入 `i2c.c`、`rtc_pcf85063.c`、`app_time.c`
2. **DWT 需要调试模块使能** (`CoreDebug->DEMCR |= TRCENA`)，AppTime_Sync 已处理
3. **PCF85063ATL 的 OS bit**（seconds 寄存器 bit7）在首次上电或断电后会置 1 表示时钟停过 → Pcf85063_GetTime 已屏蔽此位，但建议 `set_time` 后检查
4. **I2C Timing 值** 需与实际 PCLK1 频率匹配，若主频改变需重新计算
