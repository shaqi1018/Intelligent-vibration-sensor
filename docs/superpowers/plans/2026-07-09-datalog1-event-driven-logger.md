# 完整重构：DATALOG1 式事件驱动写线程（替换 logger 忙轮询）

日期：2026-07-09
参考：ST 官方 DATALOG1 V1.5.1 `sdcard_manager.c`（SDM_Thread 消息队列驱动 + 双半缓冲）

## 一、要解决的问题

现状 logger 主循环是**忙轮询**：`while(did_work){ 遍历7环写 } + osDelay(2)`。每 2ms 醒来扫一遍所有环，即使没数据也空转。问题：
- 平时空转占 CPU（虽低优先级，但满配下抢采集任务时间片）
- osDelay(2) 固定节奏与"环半满"无关 → 写时机不精确
- 上一轮我改 gate 把写块改小，掉帧变差（已修回 3/4 环，但架构仍是轮询）

DATALOG1 的写线程：**平时阻塞睡**，生产者在环半满时投消息唤醒它，醒来写一个固定大块，写完继续睡。省 CPU + 写时机精确 + 块大小恒定。

## 二、设计（适配我们：单 logger 遍历多环，非 DATALOG1 每子传感器独立文件）

**不照搬消息队列（那是为"消息带 sID/ssID 定位独立文件"设计的），改用更贴合现状的"数据就绪信号量"**——因为我们 logger 本就遍历所有环，只需要"有环该写了，醒来"这一个信号。

### 核心机制
1. **新增一个计数信号量 `s_logger_wake`**（或 binary + osEventFlags）：logger 阻塞在它上。
2. **生产者侧**（`RingBuf_Write` 内 / mic 的 `AppRing_WriteMic`）：写完后检查该环 avail 是否**跨过唤醒阈值**（如 ≥ 半环或 ≥ WRITE_BLOCK），是则 `osSemaphoreRelease(s_logger_wake)`。ISR 安全（osSemaphoreRelease 可在 ISR 调，mic 已用同款）。
3. **logger 主循环**：
   ```
   for(;;){
     osSemaphoreAcquire(s_logger_wake, timeout=50ms);  // 阻塞睡,50ms超时兜底(低速环/尾部)
     if(!session_active){ ...原有会话开关/停止逻辑... continue; }
     // 遍历7环,把达阈值的写掉(保持现有 LoggerDrainRing 3/4环 + 扇区对齐 + 大块)
     do{ did_work=0; for each ring: LoggerDrainRing(...); }while(did_work);
     // 周期性 sync / WAV checkpoint (原有逻辑,靠50ms超时也能定期跑)
   }
   ```
4. **保留 LoggerDrainRing 现有的 3/4 环 gate + 512 扇区对齐 + 超时封顶**（这些是对的，只是外层从"忙轮询"变"事件唤醒"）。

### 为什么这样最小风险又抓住精髓
- 数据结构（7 个 SPSC 环）不动 → 不推翻已验证的 ring/生产者/FatFs/IDMA/低电量/分段/WAV。
- 只换 logger 的**唤醒机制**：忙轮询 osDelay(2) → 阻塞信号量。
- 生产者加几行"跨阈值 release" → 精确唤醒。
- 50ms 超时兜底：保证低速环（env/mag 攒不满也会因超时被扫到→走 LoggerDrainRing 的 aged 分支）、周期 sync、WAV checkpoint、会话停止检测都不饿死。

## 三、逐处改动

### 1. `app_freertos.c` 全局
- 新增 `static osSemaphoreId_t s_logger_wake;`（计数信号量，max 足够大如 16，init 0）。
- `MX_FREERTOS_Init` 或 ring init 处创建：`s_logger_wake = osSemaphoreNew(16, 0, NULL);`

### 2. 生产者唤醒（RingBuf_Write 内，覆盖所有 7 环 + mic）
- `RingBuf_Write` 成功写入后，计算 avail，若 avail 首次跨过唤醒阈值（`>= min(WRITE_BLOCK, size*3/4)`，取整512）→ `if(s_logger_wake) osSemaphoreRelease(s_logger_wake);`
- 注意去重：不必每次写都 release（会淹没信号量）。可用"跨阈值边沿"或简单地"avail 超阈值就 release"（计数信号量+logger 醒来排空，多余 release 被 logger 一次清空，无害但略浪费）。**采用边沿触发**：记录上次是否已超阈值，只在 未超→超 的边沿 release。
- mic 走 `AppRing_WriteMic`→`RingBuf_Write`，天然覆盖（ISR 中 release，安全）。

### 3. logger 主循环（StartLoggerTask）
- 把 `while(did_work){...} + osDelay(2)` 的忙轮询外层，改为：循环顶部 `osSemaphoreAcquire(s_logger_wake, 50)`（50ms 超时）。
- session 开关/启动/停止逻辑**不变**（还在循环里，靠 50ms 超时定期检查）。
- drain 段**不变**（LoggerDrainRing 3/4环+对齐）。
- 周期 sync、WAV checkpoint **不变**（靠 50ms 超时定期触发，节奏从 2ms→50ms 无影响，本就是秒级周期）。
- 会话停止的 AppLoggerStopSdSession **不变**。

### 4. 唤醒阈值宏
- 复用 `APP_SD_WRITE_BLOCK`（32KB）或每环 3/4 作为唤醒阈值。低速环靠 50ms 超时兜底，不需要它们 release（它们攒不满阈值也不 release，靠超时被扫到）。

## 四、正确性 / 风险

- **信号量淹没**：边沿触发 release + 计数上限 16，logger 醒来一次排空所有环，多余信号被下次 acquire 立即消费，最坏空转几次，无害。
- **低速环不饿死**：env/mag 攒不满唤醒阈值→不 release→靠 50ms 超时被扫到→LoggerDrainRing 的 aged(超时)分支写掉。✓
- **会话停止/启动检测延迟**：最多 50ms（原 2ms）。停止是人工/自动，50ms 无感。✓
- **周期 sync/WAV checkpoint**：秒级周期，50ms 扫描足够。✓
- **mic ISR release**：osSemaphoreRelease 在 ISR 安全（项目已用 osSemaphoreRelease 从 EXTI ISR，mic 从 SAI ISR）。✓
- **不改吞吐上限**：诚实——这不改卡 PROGRAMMING 物理吞吐。预期收益=省 CPU（logger 不空转）+ 写时机精确。mic 掉帧（卡吞吐<产出的物理差）大概率仍在，需真实数据验证，可能后续仍需降 mic 率。

## 五、验证

1. Keil GUI 编译烧录。
2. 满配 CSV+IDMA **正常停止**长录 ≥15min。
3. 看 `[Ring]` drop 对比重构前（22-23 会话：mic 26663/s、qma 8768/s、h3 1535/s；gate 修回 3/4 环后应已回到基线附近）。
4. 看 CPU：`[Stack]` 各任务余量、是否 logger 空转减少（间接看 LSM/qma/h3 是否因 CPU 更充裕而改善）。
5. 若 mic 仍大量丢 → 坐实是卡吞吐物理差，下一步降 mic 率或双卡。

## 六、不做

- 不引入 CMSIS-v1 消息队列（用信号量更贴合单-logger-多-环现状）。
- 不改 7 环数据结构 / 生产者写入逻辑（只加唤醒 release）。
- 不动 FatFs/IDMA/HWFC/分段/WAV checkpoint/低电量/诊断日志。
- 不动 mic 采样率（本轮只重构，降载留作下一步据数据决定）。
