# 阶段B完成总结：SD 卡块设备与 FatFs 打通

## 1. 阶段目标

阶段B的目标是在阶段A已经完成 `SDMMC1` 底层初始化的基础上，继续打通 `SD 卡块设备 -> FatFs -> 文件读写验证` 这一条完整链路，使工程能够：

1. 正确识别已插入的 SD 卡；
2. 通过 `diskio / sd_diskio` 将 `HAL_SD` 能力桥接给 `FatFs`；
3. 完成 `挂载 -> 建文件 -> 写入 -> 关闭 -> 回读` 的最小文件系统验证；
4. 在不引入 USB MSC 和日志任务的前提下，先把 SD 文件系统底座跑通。

---

## 2. 本阶段已完成的工作

### 2.1 引入最小 FatFs 核心源码

本阶段已将当前工程需要的最小 `FatFs` 核心文件纳入项目：

- `Middlewares/Third_Party/FatFs/src/ff.c`
- `Middlewares/Third_Party/FatFs/src/ff.h`
- `Middlewares/Third_Party/FatFs/src/diskio.c`
- `Middlewares/Third_Party/FatFs/src/diskio.h`
- `Middlewares/Third_Party/FatFs/src/integer.h`
- `Middlewares/Third_Party/FatFs/src/ffconf.h`

其中 `ffconf.h` 已按当前 STM32U575 工程的阶段B需求进行裁剪与配置，采用最小可用配置：

- 单卷 `1` 个逻辑盘
- 512 字节扇区
- 关闭 LFN
- 关闭 exFAT
- 不启用重入保护
- 启用基本读写能力

这样可以避免在阶段B过早引入不必要的复杂度。

---

### 2.2 新增 SD 块设备桥接层

本阶段已新增 SD 块设备桥接文件：

- `Core/Src/sd_diskio.c`
- `Core/Inc/sd_diskio.h`

已完成内容：

- 定义当前工程使用的物理盘号与扇区大小
- 提供 `SD_disk_status()`
- 提供 `SD_disk_initialize()`
- 提供 `SD_disk_read()`
- 提供 `SD_disk_write()`
- 提供 `SD_disk_ioctl()`

桥接策略为：

- 继续复用阶段A已经稳定的 `hsd1`
- 继续复用 `SDMMC1_IsCardDetected()`
- 通过 `HAL_SD_ReadBlocks()` / `HAL_SD_WriteBlocks()` 完成阻塞式块读写
- 使用 `HAL_SD_GetCardInfo()` 向 `FatFs` 提供扇区数量与扇区大小信息
- 当前阶段保持 polling/blocking 模式，不引入 DMA

这已经满足计划书中“先把块设备打通”的目标。

---

### 2.3 完成 FatFs 通用磁盘接口接入

本阶段已补齐 `FatFs` 所需的通用磁盘调度层：

- `Middlewares/Third_Party/FatFs/src/diskio.c`

该文件当前已直接转发到板级 `sd_diskio` 实现，完成：

- `disk_status()`
- `disk_initialize()`
- `disk_read()`
- `disk_write()`
- `disk_ioctl()`
- `get_fattime()`

这样 `ff.c` 即可通过标准 `FatFs` 接口访问底层 SD 卡块设备。

---

### 2.4 新增 FatFs 文件系统验证封装

本阶段已新增文件系统测试封装：

- `Core/Src/fatfs_sd.c`
- `Core/Inc/fatfs_sd.h`

已完成内容：

- 提供 `FatFs_SD_Mount()`
- 提供 `FatFs_SD_Unmount()`
- 提供 `FatFs_SD_RunPhaseBSmokeTest()`
- 封装 `FRESULT` 到字符串的简单映射，便于串口输出

当前的阶段B测试流程为：

1. 检查是否插卡；
2. 初始化底层 disk；
3. `f_mount("0:")` 挂载文件系统；
4. 创建并打开测试文件 `0:/PHASEB.TXT`；
5. 写入一行测试文本；
6. `f_sync()` 同步；
7. `f_close()` 关闭文件；
8. 再次打开文件；
9. `f_read()` 回读内容；
10. 比较回读内容是否一致；
11. 再做一次底层扇区读取校验；
12. `unmount` 卸载文件系统。

这已经实现了阶段B要求的完整最小链路。

---

### 2.5 将阶段B测试接入主启动流程

本阶段已在 `Core/Src/main.c` 中接入文件系统验证流程。

当前启动顺序变为：

1. GPIO / ICACHE / UART 初始化
2. SD 卡检测
3. `MX_SDMMC1_SD_Init()`
4. SD 初始化成功后调用 `FatFs_SD_RunPhaseBSmokeTest()`
5. 再继续 USB PCD 与 SPI 外设初始化

这样做的好处是：

- 阶段B验证保持在调度器启动前完成
- 不引入 FreeRTOS 线程间互斥复杂度
- 文件系统问题更容易定位
- 为后续阶段C日志任务预留清晰边界

---

### 2.6 完成 MDK 工程与 eide 工程接入

本阶段不仅更新了 `MDK-ARM/SensorProj.uvprojx`，也同步更新了 `MDK-ARM/.eide/eide.yml`，确保两套工程入口都能正确编译阶段B内容。

已纳入工程的新增文件包括：

- `Core/Src/sd_diskio.c`
- `Core/Inc/sd_diskio.h`
- `Core/Src/fatfs_sd.c`
- `Core/Inc/fatfs_sd.h`
- `Middlewares/Third_Party/FatFs/src/ff.c`
- `Middlewares/Third_Party/FatFs/src/ff.h`
- `Middlewares/Third_Party/FatFs/src/diskio.c`
- `Middlewares/Third_Party/FatFs/src/diskio.h`
- `Middlewares/Third_Party/FatFs/src/ffconf.h`
- `Middlewares/Third_Party/FatFs/src/integer.h`

同时已补齐 `FatFs` 头文件搜索路径，避免 `ff.h`、`diskio.h` 等在不同构建入口下找不到的问题。

---

## 3. 阶段B验证结果

### 3.1 工程编译验证

阶段B接入后的 MDK 构建日志显示：

```text
"SensorProj\SensorProj.axf" - 0 Error(s), 0 Warning(s).
```

说明：

- 新增的 `FatFs` 核心文件可以正常参与编译
- 新增的 `sd_diskio`、`fatfs_sd` 可以正常参与链接
- 当前阶段B工程状态稳定

---

### 3.2 板上串口实测验证

当前实测串口输出为：

```text
[初始化] GPIO/ICACHE/UART1 初始化完成
[初始化] SD DET(PC13)=0, inserted_level=0
[初始化] SDMMC1 初始化完成 (检测到SD卡)
[FatFs] 挂载成功
[FatFs] 阶段B测试成功: 挂载/写入/关闭/回读 已通过
[初始化] USB OTG FS PCD 初始化完成
[初始化] SPI1 初始化完成
[初始化] SPI2 初始化完成
```

从这组日志可以确认：

1. SD 卡插卡检测正常；
2. `SDMMC1` 初始化成功；
3. `FatFs` 挂载成功；
4. 测试文件已成功写入；
5. 文件已成功关闭并重新打开；
6. 回读校验已通过；
7. 主程序后续初始化流程未受影响。

---

## 4. 阶段B验收结论

对照原计划书中阶段B的目标：

### 已满足

1. 已引入 `FatFs` 必要源码
2. 已新增 `diskio` 和 `sd_diskio`
3. 已完成 `HAL_SD` 到 `FatFs diskio` 的桥接
4. 已新增卡检测、挂载、卸载接口
5. 已完成最小文件测试：
   - 插卡
   - 挂载
   - 创建测试文件
   - 写入一行文本
   - 同步
   - 关闭
   - 回读验证
6. 工程编译通过
7. 板上实测已经通过串口验证整条链路

### 结论

**阶段B已经完成。**

---

## 5. 本阶段边界说明

虽然阶段B已经完成，但当前完成范围仍然只限于：

- SD 卡块设备打通
- `FatFs` 文件系统打通
- 最小文件创建/写入/回读验证完成

当前**尚未在阶段B内完成**的内容包括：

- 三路传感器数据持续写卡
- 日志文件命名与轮转策略
- USB MSC 设备导出 SD 卡
- 本地日志与 MSC 模式切换互斥控制

这些内容应分别在后续阶段继续推进：

- 阶段C：传感器日志任务
- 阶段D：USB MSC 导出 SD 卡
- 阶段E：日志模式与 MSC 模式切换

---

## 6. 建议下一步

建议按原计划进入 **阶段C：传感器日志任务**，优先完成：

1. 定义三传感器共享采样快照结构；
2. 新增日志任务或存储管理层；
3. 周期生成 CSV；
4. 周期 `f_write + f_sync`；
5. 在当前已经打通的 `FatFs` 底座上持续落卡。
