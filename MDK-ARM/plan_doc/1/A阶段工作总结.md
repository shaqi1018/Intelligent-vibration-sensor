# 阶段A完成总结：SDMMC1 与 USB OTG FS 底层外设骨架

## 1. 阶段目标

阶段A的目标是补齐当前工程中 `SDMMC1` 与 `USB OTG FS Device` 的底层外设骨架，使工程能够：

1. 正常编译通过；
2. 在上电后完成 SD 与 USB 外设初始化；
3. 通过串口输出明确的初始化状态；
4. 为后续 `FatFs`、传感器日志写卡、USB MSC 导出 SD 卡功能提供稳定底座。

---

## 2. 本阶段已完成的工作

### 2.1 打开 HAL 相关模块

已在 `Core/Inc/stm32u5xx_hal_conf.h` 中启用当前阶段所需 HAL 模块：

- `HAL_SD_MODULE_ENABLED`
- `HAL_PCD_MODULE_ENABLED`

同时保留当前工程运行所需的 GPIO、RCC、DMA、UART、SPI、TIM、ICACHE 等基础模块配置。

---

### 2.2 完成 SDMMC1 初始化与 MSP 骨架

已新增并打通 `SDMMC1` 相关初始化流程，主要包括：

- `Core/Src/sdmmc.c`
- `Core/Inc/sdmmc.h`

已完成内容：

- 定义 `SD_HandleTypeDef hsd1`
- 提供 `MX_SDMMC1_SD_Init()`
- 提供 `SDMMC1_IsCardDetected()`
- 配置 `SDMMC1` 初始为 `1-bit` 模式启动
- 初始化成功后切换到 `4-bit` 总线模式
- 配置 `SDMMC1` 外设时钟来源
- 完成 `HAL_SD_MspInit()` / `HAL_SD_MspDeInit()`
- 打通 `PC8~PC12 + PD2` 的 SDMMC1 复用引脚配置
- 保留 `PC13` 作为插卡检测输入

本阶段调试过程中，已经根据当前板级连接特点完成关键修正：

- SD 引脚 GPIO 改为 `GPIO_NOPULL`
- GPIO 速度改为 `GPIO_SPEED_FREQ_VERY_HIGH`
- 初始化阶段采用较保守的时钟分频设置
- 当前板级按 `USE_SD_TRANSCEIVER = 0` 的直连卡座模型处理

最终结果为：

- SD 卡检测正常
- `HAL_SD_Init()` 已成功通过
- `HAL_SD_ConfigWideBusOperation(..., SDMMC_BUS_WIDE_4B)` 已成功通过

---

### 2.3 完成 USB OTG FS PCD 初始化与 MSP 骨架

已新增并打通 `USB OTG FS` 设备侧底层初始化流程，主要包括：

- `Core/Src/usb_otg.c`
- `Core/Inc/usb_otg.h`

已完成内容：

- 定义 `PCD_HandleTypeDef hpcd_USB_OTG_FS`
- 提供 `MX_USB_OTG_FS_PCD_Init()`
- 配置 USB FS 为 Device/PCD 初始化参数
- 禁用当前阶段暂不使用的低功耗、LPM、BC、DMA、VBUS sensing 等选项
- 在 `HAL_PCD_MspInit()` 中完成：
  - `PA11/PA12` USB 复用配置
  - USB 外设时钟使能
  - `VddUSB` 供电使能
  - `OTG_FS_IRQn` 中断优先级与使能

当前阶段目标是“PCD 骨架可初始化”，并非“USB MSC 已经枚举”，这一目标已经达到。

---

### 2.4 完成中断入口打通

已在 `Core/Src/stm32u5xx_it.c` 中补齐并接通以下中断服务入口：

- `OTG_FS_IRQHandler()`
- `SDMMC1_IRQHandler()`

并已正确关联到：

- `HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS)`
- `HAL_SD_IRQHandler(&hsd1)`

这一步完成后，USB PCD 与 SDMMC1 已具备后续中断驱动能力。

---

### 2.5 完成 USB/SDMMC 所需 48MHz 时钟配置

已在外设 MSP 初始化中完成 48MHz 时钟路径配置：

- SDMMC 使用 `CLK48 / HSI48`
- USB FS 使用 `CLK48 / HSI48`

这满足阶段A对 `USB FS` 和 `SDMMC1` 外设时钟来源的要求。

---

### 2.6 完成 MDK 工程层接入与可编译验证

本阶段已经将新增的 SD/USB 相关源文件纳入当前 MDK 工程，并解决了此前 SD/USB HAL 符号未解析、工程文件成员不完整等问题。

当前结果为：

- 工程可以正常编译通过
- `main.c` 中已经接入：
  - `MX_SDMMC1_SD_Init()`
  - `MX_USB_OTG_FS_PCD_Init()`
- 上电启动流程可以顺序完成 GPIO、ICACHE、UART、SDMMC1、USB PCD、SPI 外设初始化

---

## 3. 阶段A验证结果

当前串口日志已经验证本阶段目标达成，典型输出如下：

```text
[初始化] GPIO/ICACHE/UART1 初始化完成
[初始化] SD DET(PC13)=0, inserted_level=0
[初始化] SDMMC1 初始化完成 (检测到SD卡)
[初始化] USB OTG FS PCD 初始化完成
[初始化] SPI1 初始化完成
[初始化] SPI2 初始化完成
```

其中最关键的结果是：

- SD 卡检测成功
- SDMMC1 初始化成功
- SD 总线已成功切换到 4-bit 模式
- USB OTG FS PCD 初始化成功
- 工程整体启动流程正常

---

## 4. 阶段A验收结论

对照原计划书中的阶段A目标：

### 已满足

1. 在 `stm32u5xx_hal_conf.h` 中开启 `HAL_SD_MODULE_ENABLED` 与 `HAL_PCD_MODULE_ENABLED`
2. 新增 `SDMMC1` 初始化与 MSP
3. 新增 `USB OTG FS` 初始化与 MSP
4. 补齐 `SDMMC1_IRQHandler`、`OTG_FS_IRQHandler`
5. 调整并落实 `USB FS` 与 `SDMMC1` 所需的 `48MHz` 时钟方案
6. 工程可编译通过
7. 上电后 SD 与 USB 外设可完成初始化
8. 串口可输出明确初始化状态

### 结论

**阶段A已经完成。**

---

## 5. 本阶段边界说明

虽然阶段A已经完成，但其完成范围仅限于“底层外设骨架打通”。

当前**尚未在阶段A内完成**的内容包括：

- SD 卡块读写验证
- `FatFs` 挂载与文件读写
- USB Device Core/MSC Class 接入
- 电脑识别为 U 盘
- 本地日志记录与 MSC 导出切换控制

这些内容应在后续阶段继续推进：

- 阶段B：SD 卡块设备与 `FatFs` 打通
- 阶段C：传感器日志写卡
- 阶段D：USB MSC 导出 SD 卡
- 阶段E：日志模式与 MSC 模式切换

---

## 6. 建议下一步

建议按原计划进入 **阶段B：SD 卡块设备与 FatFs 打通**，优先完成：

1. 引入 `FatFs` 必要源码；
2. 建立 `diskio / sd_diskio` 桥接层；
3. 完成 SD 卡挂载、建文件、写入、关闭验证；
4. 先实现最小可验证文件写卡链路，再进入日志任务与 USB MSC。
