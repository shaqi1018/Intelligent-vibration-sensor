# 双缓冲块池 + 专职 SD 写线程 设计文档

日期：2026-07-14
状态：待实现
分支：path/sd

## 背景与动机

当前 SD 落盘链路为串行结构：

```
7 个环形缓冲区(ring) → 单个共享 s_sd_bounce(64KB) → f_write(阻塞 DMA)
```

logger 任务在一个 `while(did_work)` 循环里轮询 7 个环，凑够扇区对齐块就
`RingBuf_CopyToBounce` 到 `s_sd_bounce`，再 `f_write`。整个"凑块(memcpy)"与
"写卡(f_write)"在同一个任务里**串行**执行——写卡阻塞期间，CPU 虽让出给传感器
任务，但**下一块的 memcpy 无法与本次写卡重叠**。

参考工程 FP-SNS-DATALOG1 的 `sdcard_manager.c` 采用双缓冲(ping-pong) +
专职 SD 写线程，把"凑块"和"写卡"解耦成流水线。本设计移植该架构。

## 实测数据前提（决定收益边界，务必记录）

对 `新建文本文档.txt`（满配 96k 长录）的 2317 个 `[SDwr]` 样本统计
（`tools/analyze_sdwr.py`）：

| 块大小(cnt×512B) | 样本数 | dma 均值(ms) |
|---|---|---|
| cnt=1 (512B) | 553 | 121 |
| cnt=33..64 (~32KB) | 506 | 121 |
| cnt=97..127 (~63KB) | 316 | 144 |

**关键结论：写 512 字节 121ms，写 63KB 也只 144ms。** 每次 f_write 有约 120ms
的固定命令开销，与数据量几乎无关——这是当前这张（山寨扩容）卡的每命令延迟。

**由此，本设计的收益边界是明确的：**
- 双缓冲能省的是"下一块 memcpy"的时间 ≈ 160µs/块。
- 在 120ms 固定写开销面前，重叠收益 ≈ **0.05%**。
- **在当前卡上，本改造几乎不会降低掉帧率。** 掉帧根因是卡带宽缺口
  (有效 173KB/s)，非拷贝开销。
- **在每命令延迟远小于 120ms 的好卡上，写时间由 DMA 传输主导，此时
  拷贝/写重叠与写线程调度才能兑现收益。本改造为换卡后的收益预留架构。**

用户已知晓上述收益边界并明确要求实现此架构。本设计的目标因此定义为：
**架构正确、零数据回归、零损坏**，而非"降低掉帧率"。

（另：数据还显示 cnt=1 元数据碎写吃掉 23% 写时间，大簇格式化可拿约 +29% ——
这是独立的、真正能动掉帧的杠杆，见"附录：正交优化"，不属本设计范围。）

## 平台约束（纠正参考文章中不适用于本平台的两点）

- **MCU = STM32U575，Cortex-M33，无 D-Cache。** 参考文章中的
  `SCB_CleanDCache_by_Addr` 不适用。本平台的 DMA 一致性隐患是 IDMA 作为第二
  总线主机读到 M33 写缓冲中的旧字节，正确原语是 `__DMB()`——现有代码已在
  `LoggerDrainRing` 启动 IDMA 前调用，移植到写线程即可。
- **数据文件是定长 BIN 帧（14B/22B）**，不是可容忍填充的文本流。参考文章
  "超时刷盘不足部分补 0x00 到扇区对齐"的做法**会损坏 BIN 文件**（插 0x00 使
  解析器错位、CRC 失败）。本设计保留现有更安全的做法：向下取整到扇区写，
  余数留环里下轮再拼，仅会话停止的末块写非对齐尾。
- **FatFs `_USE_MKFS=0`**：固件内不能格式化。大簇格式化是 PC 端/独立动作。

## 决策（已与用户确认）

| 项 | 决策 | 理由 |
|---|---|---|
| 块池大小 N | **N=3 × 32KB = 96KB** | 净增 RAM +32KB（取代现有 64KB bounce）；写 1 块时 drain 可填 2 块 |
| SD 写线程优先级 | **AboveNormal**（= 当前 logger） | 维持"SD 写在传感器(High)之下"的现状关系，与 DATALOG1 一致，风险最小 |
| 验证方式 | **满配长录 + BIN 审计对比** | 确认帧丢失率对齐现状(LSM~7%)、CRC 零损坏、无回归 |

## 架构

```
   ┌─ 采集任务(High/AboveNormal) ── 帧 ──> [7 个 ring]
   │                                          │
   │                        drain 任务(AboveNormal) 轮询：挑"最满"的环
   │                                          │ memcpy 一个扇区对齐块(≤32KB)
   │   [FreeQ 空闲块队列] ──弹出空闲块指针──────┤ 到该块，然后立即 RingBuf_Consume
   │        ▲ 初始 N 块                        │ 填 {ptr,len,file_idx} 压入 WriteQ
   │        │ 归还                             ▼
   │        └──────────  SD 写线程(AboveNormal) 弹出 WriteQ
   │                          │ __DMB(); f_write(阻塞 DMA); 周期 f_sync
   │                          └── 归还块指针到 FreeQ
```

### 内存流转（全程只传 4 字节指针）

- 块池：`static uint8_t s_block_pool[N][APP_SD_WRITE_BLOCK] __attribute__((aligned(32)))`
  （N=3, 每块 32KB）。取代现有单个 `s_sd_bounce`。
- FreeQ / WriteQ：FreeRTOS queue，元素统一用**块索引**标识（避免指针/索引混用）：
  - FreeQ 元素 = `uint8_t block_idx`（0..N-1）。
  - WriteQ 元素 = `{ uint8_t block_idx; uint32_t len; uint8_t file_idx; }`。
  - 块地址由 `s_block_pool[block_idx]` 得到。
- 初始化：块索引 0..N-1 全部入 FreeQ。
- 数据只在"环 → 块"这一次必要 memcpy 时搬运，之后仅指针流转，无二次拷贝。

## 组件与职责

### 组件 A：块池 + 双队列（新增）
- `s_block_pool[N][32KB]`，32 字节对齐。
- `s_free_q`（容量 N，装空闲块），`s_write_q`（容量 N，装待写块）。
- 启动时 N 块入 `s_free_q`。

### 组件 B：drain 任务（改造现有 logger 循环）
- 保留现有唤醒机制（`s_logger_wake` 信号量 + 50ms 超时）。
- 每轮：对 7 个环，按现有 `LoggerDrainRing` 的 gate/aged 逻辑判断是否该写。
  - **调度增强**：优先服务 `avail/size` 比例最高的环（截止期近似），把溢出从
    随机分布挪向可控。（可选，先按现有 round-robin 顺序实现，调度增强作为
    第二步。）
- 该写时：从 `s_free_q` 取一个空闲块（非阻塞 `xQueueReceive` timeout=0）。
  - 取到：`RingBuf_CopyToBounce(rb, block, want)` → **立即 `RingBuf_Consume(rb, n)`**
    → 填 `{block, n, file_idx}` 压 `s_write_q`。
  - 取不到（池空 = 背压）：跳过本环本轮（数据留环，靠环兜底 drop，与现状一致）。
- **drain 任务绝不调用任何 FatFs 函数。**

### 组件 C：SD 写线程（新增）
- 循环：`xQueueReceive(s_write_q, &item, portMAX_DELAY)` 阻塞等块。
- 取到块：`__DMB();` → `FatFs_SD_LoggerWriteFileIndex(item.file_idx, s_block_pool[item.block_idx], item.len)`。
- 写完（无论成败）：把 `item.block_idx` 归还 `s_free_q`。
- 累计已落盘字节，达 `APP_SD_SYNC_INTERVAL_BYTES` 触发 `f_sync`（从 drain 迁来）。
- 写失败重试逻辑不变（`SD_disk_write` 内部已有 3 次重试 + DeInit）。3 次仍失败：
  记 `AppFlowStatsRecordWriteFailure()`，丢该块，继续（与现状等价）。

### 组件 D：会话生命周期同步（关键，防丢尾块）
- **会话停止 / f_close 前**：drain 先把所有环的末块（`min_flush=0` 全排空）压入
  WriteQ，然后**等待 WriteQ 排空 + 所有块归还 FreeQ**（即 `s_free_q` 计数恢复 N），
  再由写线程侧执行 `f_close` / WAV 头回填。
- 用一个"barrier"实现：drain 压完末块后阻塞等 `s_free_q` 满，或用一个完成信号量。
- WAV 头更新、`f_close`、目录 `f_sync` 全部在写线程侧串行执行（FatFs 单线程约束）。

## 不变量（必须由代码保证）

1. **FatFs 单线程**：`f_write/f_sync/f_close/f_lseek/WAV 头` 只在 SD 写线程调用。
   drain 与采集任务永不碰 FatFs。
2. **块所有权单一**：块在 FreeQ 时归池所有；被 drain 取出到归还写线程前，
   是数据唯一持有者。同一块指针任一时刻只在一个队列或一个线程手里。
3. **消费即释放**：memcpy 进块后立即 `RingBuf_Consume`——这正是防环溢出的动作，
   不得延后到写成功之后（否则环无法及时腾空，退化回串行）。
4. **扇区对齐**：块内 len 除会话末块外恒为 512 整数倍（沿用现有逻辑）。
5. **__DMB 屏障**：启动 IDMA 写前必须 `__DMB()`（源块对 IDMA 可见）。
6. **收尾屏障**：f_close 前 WriteQ 必须排空，否则丢尾块。

## 保留不动（明确不改）

- 机会式门控 `APP_SD_OPPORTUNISTIC=0`（已证伪，保持关）。
- IDMA + HWFC 模式（`APP_SD_USE_IDMA=1`）。
- `aged` 超时刷盘（`APP_SD_WRITE_MAX_AGE_TICKS=3000`）——已是"超时刷盘"，
  不改成零填充。
- 采集任务优先级关系（LSM/H3 = High，在 SD 写线程之上）。
- 帧格式（14B/22B BIN）、CRC、周期 sync 的 1MB 间隔（防掉电孤儿）。
- 现有诊断打点（`[SDwr]`/`[LoggerBlk]`/`[LoggerLoop]`）——用于改造前后对比。

## RAM 预算

| 项 | 前 | 后 | 增量 |
|---|---|---|---|
| s_sd_bounce | 64KB | 0（被块池取代） | -64KB |
| 块池 N=3 | 0 | 96KB | +96KB |
| 队列 + 结构体 | 0 | ~数百字节 | 忽略 |
| **净增** | | | **+32KB** |

7 环 564KB + 块池 96KB + 其他 ≈ 660KB+，U575 SRAM 786KB，余量充足。

## 风险与缓解

| 风险 | 缓解 |
|---|---|
| FatFs 被两线程并发调用 → 文件系统损坏 | 不变量 1：FatFs 只在写线程。代码审查 + BIN CRC 审计确认。 |
| 收尾丢尾块 | 不变量 6：f_close 前屏障等 WriteQ 排空。 |
| 块所有权错乱（double-free/泄漏） | 不变量 2：块索引/指针严格单持有。队列容量恰为 N，泄漏会表现为 FreeQ 永不满，收尾屏障会超时暴露。 |
| 池空背压退化 | 与现状等价（环满即 drop）。收益边界已知。 |
| 引入回归使现有 ~7% 变差 | 满配长录 + BIN 审计对比，任一通道显著变差即回退。 |

## 验证计划

1. 编译（Keil GUI，命令行 UV4 -build 在 GUI 开时不可靠——见项目记忆）。
2. 满配长录（7 通道 + 96k mic），≥10 分钟。
3. BIN 审计（现有工具）对比改造前后：
   - LSM ACC/GYR 帧丢失率 ≈ 7%（不显著变差）。
   - QMA/H3/MAG/AHT 帧丢失率不变差。
   - **CRC 零损坏**（这是本改造正确性的硬指标）。
   - 会话目录/文件完整，无孤儿、无空目录、尾块不丢。
4. 观察 `[SDwr]`/`[LoggerLoop]` 打点确认写线程正常出队、无 WriteQ 长期堆积或
   FreeQ 长期枯竭。

## 附录：正交优化（不属本设计，但数据强烈指向）

`tools/analyze_sdwr.py` 显示 cnt=1 单扇区写占 24% 写次数、23% 写时间(67s)，
其中 233 次在低扇区地址（FAT/目录/FSINFO）= 文件系统元数据碎写。消除后有效
吞吐 173→223KB/s（+29%）。手段：**SD 卡格式化为大簇（exFAT 或 FAT32 32/64KB
簇）**。零固件代码（除非要固件内格式化，则需开 `_USE_MKFS`）。这是当前卡上
唯一实测指向显著掉帧改善的杠杆，建议作为独立任务评估。
