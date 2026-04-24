# 阶段C完成总结：三传感器 CSV 日志任务落卡

## 1. 阶段目标

阶段C的目标是在阶段B已经完成 `SD 卡块设备 -> FatFs -> 文件读写验证` 的基础上，继续把三路传感器的实时数据持续写入 SD 卡，形成可离线分析的 CSV 日志文件，使工程能够：

1. 在 FreeRTOS 运行期间持续采集三颗传感器数据；
2. 通过共享快照模型汇总 `LSM6DSOX`、`H3LIS100DL`、`QMA6100P` 的最新采样结果；
3. 通过独立 `logger task` 周期生成 CSV 行并写入 SD 卡；
4. 建立日志目录、文件命名和周期同步策略；
5. 在不引入 USB MSC 切换、DMA、多级缓存等复杂机制的前提下，先把“多传感器持续写卡”主链路跑通。

---

## 2. 本阶段已完成的工作

### 2.1 新增共享传感器快照模型

本阶段已新增共享快照头文件：

- `Core/Inc/sensor_snapshot.h`

已完成内容：

- 定义统一采样周期 `APP_SENSOR_SAMPLE_PERIOD_MS = 500ms`
- 定义数据新鲜度阈值 `APP_SENSOR_STALE_TIMEOUT_MS = 1000ms`
- 为三颗传感器分别定义最新快照结构：
  - `AppLsm6dsoxSnapshot_t`
  - `AppH3lis100dlSnapshot_t`
  - `AppQma6100pSnapshot_t`
- 定义总快照结构：
  - `AppSensorSnapshot_t`

每个子快照都包含：

- `valid`：该传感器当前快照是否有效
- `last_update_ms`：最近一次成功更新的系统 tick
- `data`：对应驱动层返回的最近一次有效数据

这样阶段C就建立了三颗传感器到日志任务之间的统一数据交换模型。

---

### 2.2 在 RTOS 中新增快照互斥与发布机制

本阶段已在 `Core/Src/app_freertos.c` 中加入共享快照访问控制：

- 新增 `snapshot_mutex`
- 新增全局快照对象 `g_sensor_snapshot`
- 新增快照发布与复制函数：
  - `AppSnapshotPublishLsm6dsox()`
  - `AppSnapshotPublishH3lis100dl()`
  - `AppSnapshotPublishQma6100p()`
  - `AppSnapshotCopy()`

设计策略为：

- 三个传感器线程仍然保持原有采样结构，不重构驱动调用方式；
- 采样成功后只负责把结果发布到共享快照；
- `logger task` 只在写卡前复制一次完整快照；
- 原有 `spi2_mutex` 继续只负责 `H3LIS100DL` 与 `QMA6100P` 的共享 SPI2 总线仲裁；
- 新增 `snapshot_mutex` 专门负责多线程快照一致性。

这样避免了传感器线程直接参与文件系统操作，也避免了 SD 写延迟反向阻塞传感器采样。

---

### 2.3 将正常模式下的三传感器任务切换为“采样 + 发布”

本阶段已保持原有三路线程架构不变，但调整了正常运行模式下的行为：

- `StartLsm6dsoxTask()`
- `StartH3lis100dlTask()`
- `StartQma6100pTask()`

当前策略为：

- 单传感器测试模式下，仍然保留原有详细串口打印，便于单独调试；
- 正常三传感器模式下，不再每 500ms 持续串口刷屏；
- 正常模式改为：采样成功后更新共享快照，由日志线程统一写入 CSV。

这样可以减少串口输出对系统运行节奏的干扰，同时让阶段C的主要输出通道回归到 SD 卡日志文件本身。

---

### 2.4 扩展 FatFs 存储封装为日志会话接口

本阶段在阶段B已有 `fatfs_sd.c / fatfs_sd.h` 的基础上，继续扩展出阶段C日志接口：

- `FatFs_SD_LoggerStart()`
- `FatFs_SD_LoggerAppendRow()`
- `FatFs_SD_LoggerSync()`
- `FatFs_SD_LoggerStop()`

同时保留并兼容原有阶段B接口：

- `FatFs_SD_Mount()`
- `FatFs_SD_Unmount()`
- `FatFs_SD_RunPhaseBSmokeTest()`

扩展后的职责划分为：

- 阶段B函数仍负责启动前的最小文件系统验证；
- 阶段C函数负责 RTOS 运行期日志文件的创建、追加、同步和关闭。

这样没有再新建一套并行存储路径，而是复用了阶段B已经验证稳定的 FatFs 底座。

---

### 2.5 完成日志目录、文件命名和 CSV 表头定义

本阶段在 `Core/Src/fatfs_sd.c` 中实现了日志文件组织规则：

- 日志目录：`0:/LOG`
- 日志文件：顺序编号形式
  - `LOG0001.CSV`
  - `LOG0002.CSV`
  - ...

启动日志会话时的流程为：

1. 初始化底层 SD disk；
2. 挂载 `0:`；
3. 检查并创建 `LOG` 目录；
4. 扫描下一个未占用文件名；
5. 创建并打开新的 CSV 文件；
6. 写入固定表头；
7. `f_sync()` 确保表头落盘。

当前 CSV 表头包含：

- 行级字段：
  - `tick_ms`
  - `row_seq`
- `LSM6DSOX` 段：
  - `valid`
  - `age_ms`
  - 三轴加速度
  - 三轴陀螺仪
  - 温度
- `H3LIS100DL` 段：
  - `valid`
  - `age_ms`
  - 三轴原始值
  - 三轴加速度
- `QMA6100P` 段：
  - `valid`
  - `age_ms`
  - 三轴原始值
  - 三轴加速度

这样阶段C已经具备可直接导出分析的统一 CSV 结构。

---

### 2.6 完成 logger task 周期写卡与同步策略

本阶段已在 `Core/Src/app_freertos.c` 中新增独立日志线程：

- `StartLoggerTask()`

其运行策略为：

1. 启动后打印一次任务启动信息；
2. 调用 `FatFs_SD_LoggerStart()` 打开新的日志文件；
3. 每 `500ms` 复制一次三传感器共享快照；
4. 生成一行 CSV 并调用 `FatFs_SD_LoggerAppendRow()` 写入；
5. 每 `5` 行调用一次 `FatFs_SD_LoggerSync()`；
6. 如果启动、追加或同步失败，则打印一次简短错误并延时重试。

当前策略仍然保持：

- 阻塞式 `f_write()` / `f_sync()`
- 不引入 DMA
- 不引入双缓冲/环形缓冲
- 不引入日志模式与 MSC 模式切换

这满足了阶段C“先把持续落卡跑通”的范围要求。

---

### 2.7 完成工程文件接入与构建验证

本阶段对工程入口同步完成了阶段C所需接入：

- `MDK-ARM/SensorProj.uvprojx`
- `MDK-ARM/.eide/eide.yml`

其中：

- `sensor_snapshot.h` 已纳入工程视图；
- 阶段C新增接口通过现有 `Core/Inc` 搜索路径参与编译；
- 阶段B已接入的 `fatfs_sd.c/h` 在阶段C扩展后继续参与两套工程编译。

MDK 构建日志显示：

```text
"SensorProj\SensorProj.axf" - 0 Error(s), 0 Warning(s).
```

说明阶段C代码接入后，当前工程仍保持稳定可编译状态。

---

## 3. 阶段C验证结果

### 3.1 启动串口验证

当前板上串口输出已验证：

```text
[LSM6DSOX] 初始化成功 (加速度:+/-4g 104Hz, 陀螺仪:+/-2000dps 104Hz)
[QMA6100P DIAG] CHIP_ID try=1 ret=0 TX=[80 00] RX=[FF 90]
[QMA6100P] init ok (+/-4g 100Hz)
[H3LIS100DL DIAG] WHO_AM_I try=1 ret=0 TX=[8F 00] RX=[FF 32]
[H3LIS100DL] init ok (+/-100g 100Hz)
[Logger] task started, period=500 ms
[FatFs] 日志文件已打开: 0:/LOG/LOG0001.CSV
```

从这组日志可以确认：

1. 三颗传感器都已成功初始化；
2. `logger task` 已经启动；
3. 日志目录/文件创建成功；
4. 阶段C主链路已进入持续写卡状态；
5. 正常路径下串口静默属于设计预期，不代表系统停住。

---

### 3.2 构建验证

本阶段已完成以下验证：

1. MDK 工程构建通过；
2. `build_log_phase_c.txt` 记录为 `0 Error(s), 0 Warning(s)`；
3. 按 eide 的包含路径对 `main.c`、`app_freertos.c`、`fatfs_sd.c` 做了语法检查，均通过。

说明阶段C的头文件依赖、接口声明和主要源文件在两套工程入口下都已对齐。

---

## 4. 阶段C验收结论

对照原计划书中阶段C的目标：

### 已满足

1. 已新增 `logger task`
2. 已定义共享采样快照结构
3. 已实现周期生成 CSV
4. 已实现周期 `f_write + f_sync`
5. 已定义日志目录与文件命名规则
6. 三颗传感器采样结果已能汇总到统一日志线程
7. 日志文件可在 `0:/LOG/LOG0001.CSV` 形式下创建
8. 工程编译通过，板上启动链路验证通过

### 结论

**阶段C已经完成。**

---

## 5. 本阶段边界说明

虽然阶段C已经完成，但当前完成范围仍然只限于：

- 三传感器共享快照建立
- `logger task` 周期写卡
- CSV 文件组织、字段定义与周期同步
- 基于 FatFs 的持续日志落卡主链路

当前**尚未在阶段C内完成**的内容包括：

- USB MSC 把 SD 卡导出给电脑
- 本地写卡与 MSC 访问之间的互斥切换
- RTC 时间戳命名或实时时间写入
- DMA/双缓冲/高吞吐量日志优化
- 热插拔完整恢复策略的系统级打磨

这些内容应继续放在后续阶段推进：

- 阶段D：USB MSC 导出 SD 卡
- 阶段E：日志模式与 MSC 模式切换

---

## 6. 建议下一步

建议按原计划进入 **阶段D：USB MSC 导出 SD 卡**，优先完成：

1. 引入 USB Device Core 与 MSC Class；
2. 新增 `usbd_storage_if` 并桥接 SD 读写接口；
3. 让电脑端能把设备识别成 U 盘；
4. 在阶段D先验证只读或基础读写可见性；
5. 再在阶段E处理“日志模式”和“MSC 模式”之间的切换与互斥。
