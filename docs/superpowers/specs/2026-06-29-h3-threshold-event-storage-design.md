# H3 阈值触发事件存储设计

**日期**: 2026-06-29
**范围**: 记录模式新增"阈值触发存储"——以 H3LIS100DL 合矢量越限为触发,SD 卡按事件分段录制(带亚秒预触发)
**目标**: 平时不写盘,一旦 H3 检测到冲击,把"冲击前亚秒 + 冲击后 N 秒"作为一次独立会话录到 SD;USB 路径完全不受影响

---

## 形式与术语

本特性是"事件门控存储"(形式 B,带预触发),等价于示波器的触发采集:

- **触发源**: 仅 H3LIS100DL(±100g 抗冲击),~400Hz。
- **判据**: 合矢量 `acc_x² + acc_y² + acc_z² > 阈值²`(单位 g,上越;比较平方避免开方)。
- **预触发(pre-trigger)**: 触发瞬间各 ring 中现存的最近一段历史(亚秒,受 ring 大小约束,< 1s,不单独配置)。
- **后触发(post-trigger)**: 触发后继续录 `post_sec` 秒。
- **事件 = 一次普通会话**: 每个事件复用现有 `FatFs_SD_LoggerStart` 机制,生成普通会话目录(`CKBXnnnn`),内含照旧的传感器文件 + `CONFIG.JSN` 快照。**不发明 EVT 命名**;靠快照里的模式参数区分。
- **排他性**: DEVCFG.JSN 配了阈值触发模式后,其他录制触发源(按键等)对 SD 录制不起作用。**与 USB 无关**——USB(WCID 直发)绕过 ring,照旧连续流。

---

## 现状锚点(已读代码)

| 事实 | 位置 |
|---|---|
| H3 每样本得 `data.acc_mg[0..2]`(mg 整数),返回后即可判据 | `app_freertos.c:2743-2768` |
| 各传感器写各自 ring;logger 任务抽 ring 写 SD(轮询 `s_sd_bounce`,防 IDMA 损坏) | `app_freertos.c:3500-3550` |
| **`RingBuf_Write` 满了丢"新"数据、保留"旧"数据** | `app_freertos.c:1430` |
| 会话目录由 `FindNextSessionDir` 从 `CKBX0001` 逐个 `f_stat` 扫到空位(O(n)) | `fatfs_sd.c:127-153` |
| `LoggerStart` 建目录 + 开 6 文件首段 + 写头;支持 BIN/CSV + seg_size_mb 分段 | `fatfs_sd.c:289-376` |
| `DeviceCfg_WriteCurrentToSD` 往会话目录写 `CONFIG.JSN` 配置快照 | `device_config.c:602,823` |
| 触发模式骨架 `NONE/EXTERNAL/TIMER` + 状态机 `IDLE/ARMED/RUNNING/STOPPING`;**EXTERNAL/TIMER 及 ARMED 未接通**(无代码消费 ARMED) | `acq_config.h:42-46,86-91`; `acq_config.c:381` |
| ring 尺寸: LSM 256KB / QMA 32KB / H3 16KB / MAG 16KB / AHT 4KB / MIC 64KB | `sensor_snapshot.h:103-109` |

LSM BIN 帧 28 字节 @6664Hz ≈ 187KB/s → 256KB ring 预触发上限 ≈ **1.3s**(CSV 更大,~0.5s)。预触发被 LSM 卡在亚秒级,符合需求。

DATALOG1 参考工程为连续流式记录,无预触发事件模式可抄(其 Automode 是定时自动分段)。预触发环形缓冲为本工程自有设计。

---

## 配置(DEVCFG.JSN)

**复用既有 `trigger_mode` 字段,不新增平行模式字段。** 现状 `trigger_mode`(`NONE/EXTERNAL/TIMER`)是没接完的脚手架:仅 `AcqConfig_Start`(`acq_config.c:381`)用它决定 `RUNNING` vs `ARMED`,但全工程无任何代码消费 `ACQ_STATE_ARMED`,采集任务用独立的 `g_acq_ctrl.running`,且 DEVCFG.JSN 从不解析 `trigger_mode`。`EXTERNAL/TIMER` 从未真正工作,`ARMED` 状态闲置。

本设计**给 `trigger_mode` 加 `THRESHOLD` 值并真正接通**:`ARMED`(注释即"等待触发")语义与事件门控吻合,顺手复活闲置状态机。`EXTERNAL/TIMER` 保留不动(本就未用)。

沿用既有自定义 JSON 解析 + 缺字段自动补写 + 夹紧的套路(同 `seg_size_mb`/`bat_full_mv`):

```jsonc
"trigger_mode": "threshold", // "none"(默认=今天的连续录制) | "threshold"(事件门控)
"trig_level_g": 5,           // 合矢量阈值(g),夹紧 [1, 100]
"trig_post_sec": 3,          // 后触发录制秒数,夹紧 [1, 600]
"trig_holdoff_sec": 2        // 事件间死区秒数,夹紧 [0, 600]
```

`"none"` = 默认值 = 现行连续记录模式(`acq_start`/开机自采/按键启动后连续录到停或 `duration_ms`),新功能默认关闭、不配者行为不变。

默认值: `trig_level_g=5`、`trig_post_sec=3`、`trig_holdoff_sec=2`。
缺失字段自动补写进 DEVCFG.JSN(带 `_doc_*` 说明,不写上限注释,夹紧静默在代码内)。

`AcqConfig_t` 改动:
```c
/* AcqTriggerMode_t 既有 NONE/EXTERNAL/TIMER, 新增: */
ACQ_TRIGGER_THRESHOLD = 3,   /* H3 合矢量越限事件门控 */

/* 新增参数字段(仅 THRESHOLD 模式生效): */
uint16_t trig_level_g;       /* 合矢量阈值, g */
uint16_t trig_post_sec;      /* 后触发秒数 */
uint16_t trig_holdoff_sec;   /* 事件间死区秒数 */
```
触发判据用 `trig_level_mg² = (level_g*1000)²` 预算成 `uint64_t` 存运行时,H3 每样本只做整数乘加比较。触发源固定 H3,不设 `trig_src`(YAGNI,扩展时再加)。

---

## 行为流程(状态机)

阈值模式下采集启动后,进入 **ARMED(布防)** 而非直接录制:

```
IDLE --start(threshold)--> ARMED
  ARMED:  所有启用传感器照常采集 → logger 消费者侧裁剪各 ring,只留最近 ~size/2(见风险①缓解)
          logger 不开文件、不写 SD;H3 任务每样本算合矢量
  ARMED --H3 合矢量越限--> TRIGGERED
  TRIGGERED:  ① 新建会话目录 + 开 6 文件(复用 LoggerStart)
              ② 立即 flush 各 ring 现存内容(= 预触发段)
              ③ 启动 post 计时器
  RUNNING(录制中):  logger 照常抽 ring 写 SD(原行为)
          若再次越限 → 重置 post 计时(延长录制,抓完整事件)
  RUNNING --安静满 post_sec--> 收尾(flush 尾部 + 写 CONFIG.JSN + 关文件)
          --> HOLDOFF(死区 holdoff_sec,期间不响应触发)
  HOLDOFF --超时--> ARMED(回到布防,等下一个事件)
  任意态 --stop--> IDLE
```

预触发自然性: ARMED 期间 ring 维持"最近 pre_window"内容,触发时 ②的 flush 即把这段历史落盘,无需独立预触发缓冲。

---

## 风险与缓解(已与用户确认)

| 风险 | 性质 | 严重度 | 缓解 |
|---|---|---|---|
| **① 预触发取错段**: 现 `RingBuf_Write` 满了丢新保旧,预触发需丢旧保新 | 内容错位 | **高(必改)** | **消费者侧维持**(保护干净 SPSC,不碰 6664Hz 热路径):ARMED/HOLDOFF 期 logger 不写盘,而是主动消费每条 ring 的最旧字节、只留最近 ~size/2(pre_window,LSM ≈ 0.6s,< 1s)。`RingBuf_Write` 与生产者一字不改(仍仅生产者写 `wr_idx`、仅消费者写 `rd_idx`,SPSC 不破)。触发时现有 drain 把留存尾段刷出即预触发。留 size/2 的空余也保证 ARMED 期生产者永不触发"丢新"。 |
| **② 开文件延迟溢出丢帧**: 触发瞬间 mkdir+6×open+头+sync(数十~>100ms),其间 ring 续灌 | 丢样本 | 中 | LSM ring 1.3s 深度提供缓冲;复用 `AppPrintRuntimeDiag` ring overrun 统计监测;CONFIG.JSN 记录是否发生溢出 |
| **③ FindNextSessionDir O(n)**: 事件攒多后每次从 CKBX0001 重扫,放大②的窗口 | 丢样本(长期) | 中 | 增量分配:缓存上次会话序号,从 last+1 起扫,避免 O(n²) |
| ④ 与 seg_size_mb 分段交互 | — | 低 | post_sec=3s 时 LSM 仅 ~560KB ≪ 512MB 段限,事件内不触发分段;无需处理 |
| 字节损坏 | — | **无** | 写盘字节通路(`LoggerWriteFileIndex`/`s_sd_bounce` 轮询)一字不改,IDMA 防损坏成果不重开 |

**事件内数据质量 == 现连续模式**(同一条写盘路径)。本特性的风险全在"边界丢帧"与"预触发段正确性",不涉及已写字节的损坏。

---

## 单元划分(便于隔离实现与测试)

1. **触发检测器**(H3 采集支路内,`app_freertos.c`): 输入每样本 `acc_mg[3]`,输出"是否越限";纯整数,无副作用。
2. **事件状态机**(新模块或并入 logger 任务): ARMED/TRIGGERED/RUNNING/HOLDOFF 转换 + post/holdoff 计时;调用 LoggerStart/Stop。
3. **ARMED 态 ring 裁剪**(logger 消费者侧): ARMED/HOLDOFF 时 logger 消费各 ring 最旧字节、只留最近 ~size/2;`RingBuf_Write`/生产者不改。
4. **配置解析与快照**(`device_config.c`/`acq_config.c`): 新字段解析、夹紧、自动补写、CONFIG.JSN 输出。
5. **会话序号增量分配**(`fatfs_sd.c`): `FindNextSessionDir` 缓存 last index。

每个单元接口清晰、可独立验证;写盘字节层不动。

---

## 不做(YAGNI)

- 不做单轴/任一轴触发(已定合矢量)。
- 不做可配置预触发时长(用 ring 自然深度,< 1s)。
- 不做多触发源并存(触发源固定 H3,不设 `trig_src` 字段)。
- 不动老 `EXTERNAL`/`TIMER`(本就未接通),不在本特性内补完它们。
- 不做事件专用目录命名(用普通 CKBXnnnn + CONFIG.JSN 区分)。
- 不碰 USB 路径、不碰 IDMA、不碰已验证的写盘字节通路。

---

## 预期结果

- 平时(ARMED)不产生 SD 写,省卡省功耗;H3 越限后产生一个普通会话目录,含预触发亚秒 + 后触发 `post_sec` 秒的完整多传感器数据。
- CONFIG.JSN 快照标明本次为 threshold 模式及阈值/post/holdoff 参数。
- USB 流式输出与现状完全一致。
- 录下的数据质量与连续模式一致;边界丢帧由 ring 深度 + overrun 监测兜底。
