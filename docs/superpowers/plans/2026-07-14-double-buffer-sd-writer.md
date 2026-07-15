# 双缓冲块池 + 专职 SD 写线程 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 SD 落盘从"logger 任务串行 memcpy+f_write"改造为"drain 任务凑块 → 双队列 → 专职写线程 f_write"的流水线，honor FatFs 单线程不变量，零数据回归、零 CRC 损坏。

**Architecture:** 新增 N=3×32KB 块池取代单个 s_sd_bounce。drain 任务从环 memcpy 进空闲块并立即 Consume，压入 WriteQ（tagged message：DATA/SYNC/WAVCKPT/STOP）。专职 SD 写线程是 FatFs 唯一访问者，按 FIFO 顺序执行写/同步/检查点/收尾，写完把块归还 FreeQ。会话收尾用 STOP 消息 + 完成信号量作屏障，防丢尾块。

**Tech Stack:** STM32U575 (Cortex-M33, 无 D-Cache)、FreeRTOS (CMSIS-OS v2)、FatFs、SDMMC IDMA+HWFC、Keil MDK-ARM。

---

## 平台现实与验证方式（务必先读）

- **无主机单元测试框架。** 本项目是裸机固件，没有 pytest/ceedling 等。"红-绿 TDD"在此不适用。
  每个任务的验证 = **(a) Keil GUI 编译通过（由用户在 GUI 中点编译，命令行 UV4 -build 在 GUI
  开启时读旧日志不可靠——见项目记忆 [[project_keil_gui_lock_build]]）**，behavior 任务追加
  **(b) 硬件冒烟 + BIN 审计对比**。
- **每个任务结束后 commit**（记忆：commit 随时可，push 必须等用户明确指示 [[feedback_no_push]]）。
- **不新增 .c 文件**：ring/drain/queue 逻辑与大量 static 状态（g_ring_*、s_logger_wake）都在
  `app_freertos.c`，拆出去要暴露一堆 static 且要改 .uvprojx + .eide（记忆 [[project_eide_builder_params]]）。
  全部改动落在 `app_freertos.c` + `sensor_snapshot.h`，遵循项目现有单文件惯例。
- **回退锚点**：改造前的串行实现在 commit `ebdeb21`（spec）之前的 HEAD。任一通道 BIN 审计显著
  变差即整体回退。

## 文件结构

| 文件 | 职责 | 改动 |
|---|---|---|
| `Core/Inc/sensor_snapshot.h` | 常量定义 | 新增 `APP_SD_BLOCK_POOL_N`、`APP_SD_WRITEQ_LEN` |
| `Core/Src/app_freertos.c` | 全部逻辑 | 块池/双队列/写线程/drain 重构/收尾屏障/生命周期迁移 |

## 已知调用点（改造前基线，供任务引用）

- `s_sd_bounce[APP_RING_FLUSH_CHUNK]`：`app_freertos.c:1365`（IDMA 唯一读源，将被块池取代）
- `LoggerDrainRing()`：`app_freertos.c:1380`（核心待拆函数：CopyToBounce→__DMB→WriteFileIndex→Consume）
- drain 主循环：`app_freertos.c:2943-3000`
- 周期 f_sync：`app_freertos.c:3043-3057`
- WAV checkpoint：`app_freertos.c:3062-3076`
- 帧追加路径：`app_freertos.c:3078-3113`（`FatFs_SD_LoggerAppendFrame`）
- 会话收尾强排空：`AppLoggerStopSdSession()` `app_freertos.c:795-833`
- 信号量创建：`app_freertos.c:371-374`（`s_logger_wake`、`s_sdmmc_dma_sem`）
- 线程创建：`app_freertos.c:398-408`

## FatFs 单线程迁移总表（改造后唯一访问者 = SD 写线程）

| FatFs 调用 | 改造前位置 | 改造后 |
|---|---|---|
| `FatFs_SD_LoggerWriteFileIndex` | LoggerDrainRing:1442 | WriteQ DATA 消息 → 写线程 |
| `FatFs_SD_LoggerSync` | logger 3048 / 3115 | WriteQ SYNC 消息 → 写线程 |
| `FatFs_SD_WavCheckpoint` | logger 3067 | WriteQ WAVCKPT 消息 → 写线程 |
| `FatFs_SD_LoggerStop` | AppLoggerStopSdSession:823 | WriteQ STOP 消息 → 写线程（屏障后） |
| `FatFs_SD_LoggerAppendFrame` | logger 3080 / 3101 | **见 Task 2：先确认是否 SD-sink 下活跃路径** |
| `FatFs_SD_LoggerStart` / `WavCreate` / `RingBuf_Reset` | logger 2837-2866 | 保留在 logger（写线程 provably idle：WriteQ 空、无 DATA 在飞），文档化前提 |

---

## Task 0: 基线固定 + 改造前满配审计

建立回退锚点与"改造前"BIN 审计数字，作为最终对比基准。纯记录，不改代码。

**Files:** 无（仅 git + 硬件跑一次）

- [ ] **Step 1: 记录回退锚点**

```bash
cd /d/ChengKe/Sensor_1/Sensor_Proj_V1.0
git log --oneline -3     # 记下当前 HEAD 短哈希，写进本任务备注
git status               # 确认工作区干净（uvprojx 行尾噪声可忽略）
```

- [ ] **Step 2: （用户）Keil GUI 编译 + 满配长录 ≥10 分钟**

7 通道 + 96k mic，SD sink。存下串口日志 + 会话目录（BIN 文件）。

- [ ] **Step 3: 跑改造前 BIN 审计，记录每通道帧丢失率 + CRC**

用现有审计工具（tools/ 下），记录：LSM ACC/GYR、QMA、H3、MAG、AHT 丢失率 + CRC 全绿。
这组数字写进 plan 本任务下方作为"BASELINE"。预期 LSM~7%、CRC 零损坏。

- [ ] **Step 4: 无需 commit（无代码改动）**

本任务只产出基线数字，记录在对话/plan 备注即可。

---

## Task 1: 确认帧追加路径是否为 SD-sink 活跃路径（决定迁移范围）

logger 任务里存在两条 FatFs 写路径：BIN 环排空（`LoggerDrainRing`）与逐帧
（`FatFs_SD_LoggerAppendFrame`，:3078-3113）。改造前必须确认后者在 SD sink 满配下是否真被执行——
若活跃，它是第二个 FatFs 写者，也必须迁到写线程；若在 SD sink 下 provably 不进入（例如
`AppFrameBufferPop` 恒返回 0 或该分支被 sink 类型绕开），则只需在写线程约束下把它标注为"USB/其他 sink
专用，SD 会话期不触发"。

**Files:** 无（纯代码阅读 + 判定，产出结论写进 plan）

- [ ] **Step 1: 判定帧缓冲在 SD 会话期是否有数据**

阅读 `app_freertos.c:3078` 的 while 条件与 `AppFrameBufferPop`（:1138）。确认：
SD sink 下，采集任务是否仍 `AppFrameBufferPush`（:2119）？还是 push 仅在非 SD sink 时发生？

Run（辅助）：
```bash
cd /d/ChengKe/Sensor_1/Sensor_Proj_V1.0
grep -n "AppFrameBufferPush\|AppFrameBufferPop\|APP_ACQ_SINK" Core/Src/app_freertos.c | head -40
sed -n '2105,2120p' Core/Src/app_freertos.c   # push 是否受 sink 门控
```

- [ ] **Step 2: 记录结论（二选一），决定后续 Task 6 范围**

在 plan 本任务下写明其一：
- **结论 A（活跃）**：SD 会话期帧缓冲有数据 → Task 6 必须把 3078-3113 的 `LoggerAppendFrame` 也
  改成经 WriteQ（新增 APPENDFRAME 消息类型或复用 DATA）。
- **结论 B（不活跃）**：SD 会话期帧缓冲恒空/被绕开 → Task 6 仅需保证该 while 在写线程改造后不直接
  调 FatFs（保持现状即可，因为它根本不执行），并加注释说明。

> 备注：从当前阅读看，BIN 路径（环 drain）是 SD 落盘主力，逐帧路径疑为早期/其他 sink 残留。
> 但**必须由 Step 1 证实**，不可假设。

- [ ] **Step 3: 无 commit（无代码改动）**

---

## Task 2: 常量定义（块池 N 与队列长度）

**Files:**
- Modify: `Core/Inc/sensor_snapshot.h`（在 `APP_SD_WRITE_BLOCK` 定义附近，约 :153）

- [ ] **Step 1: 新增常量**

在 `sensor_snapshot.h` 中 `#define APP_SD_WRITE_BLOCK (32U * 1024U)` 之后插入：

```c
/* ★2026-07-14 双缓冲块池:取代单个 s_sd_bounce。N 个 32KB 连续对齐块,drain 凑块与
 * 写线程 f_write 流水线重叠。N=3:写线程写 1 块时 drain 可填另 2 块。净增 RAM = N*32KB - 64KB(旧bounce)。 */
#define APP_SD_BLOCK_POOL_N     3U
/* WriteQ 深度:除 N 个 DATA 块外,还要容纳 SYNC/WAVCKPT/STOP 控制消息,给足余量避免 drain 侧阻塞。 */
#define APP_SD_WRITEQ_LEN       (APP_SD_BLOCK_POOL_N + 4U)
```

- [ ] **Step 2: （用户）Keil GUI 编译**

Expected: 编译通过（仅新增宏，无引用，必然通过）。

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/sensor_snapshot.h
git commit -m "feat(path/sd): 双缓冲块池常量 N=3 + WriteQ 深度"
```

---

## Task 3: 块池 + 双队列 + tagged 消息数据结构（不接线）

只定义静态存储与初始化辅助函数，暂不接入 drain/写线程（下一任务才用）。保留旧 `s_sd_bounce`
暂不删，确保本任务编译独立通过。

**Files:**
- Modify: `Core/Src/app_freertos.c`（`s_sd_bounce` 定义处 :1365 附近新增；信号量声明区 :88-95 附近）

- [ ] **Step 1: 定义消息类型、块池、队列句柄**

在 `app_freertos.c` 中 `s_sd_bounce`（:1365）**之后**新增：

```c
/* ===== 双缓冲块池 + 双队列(2026-07-14) ===== */
/* WriteQ 消息类型:写线程是 FatFs 唯一访问者,所有落盘动作都经此队列串行化。 */
typedef enum {
  APP_SDMSG_DATA = 0,   /* 写一个数据块到 file_idx,写完归还 block_idx 到 FreeQ */
  APP_SDMSG_SYNC,       /* FatFs_SD_LoggerSync() */
  APP_SDMSG_WAVCKPT,    /* FatFs_SD_WavCheckpoint() */
  APP_SDMSG_STOP        /* 收尾:FatFs_SD_LoggerStop(),然后 release s_sd_writer_done */
} AppSdMsgType_t;

typedef struct {
  uint8_t  type;        /* AppSdMsgType_t */
  uint8_t  block_idx;   /* DATA:块索引 0..N-1;其他消息忽略 */
  uint8_t  file_idx;    /* DATA:目标文件索引;其他忽略 */
  uint32_t len;         /* DATA:块内有效字节(扇区对齐,末块可非对齐);其他忽略 */
} AppSdWriteMsg_t;

/* N 个 32KB 连续对齐块。取代旧单个 s_sd_bounce(下一步接线后删除旧的)。
 * 32 字节对齐:IDMA 对 SDMMC FIFO 做 32-bit 访问,源缓冲须字长对齐(H2 防护,同旧 bounce)。 */
static uint8_t s_block_pool[APP_SD_BLOCK_POOL_N][APP_SD_WRITE_BLOCK] __attribute__((aligned(32)));

static osMessageQueueId_t s_free_q;    /* 装空闲 block_idx(uint8_t),容量 N */
static osMessageQueueId_t s_write_q;   /* 装 AppSdWriteMsg_t,容量 APP_SD_WRITEQ_LEN */
static osSemaphoreId_t    s_sd_writer_done;  /* 收尾屏障:STOP 处理完后 release */
/* 写线程句柄 sdWriterTaskHandle 在 Task 4 随 sdWriterTask_attributes 一起定义(沿用项目 *TaskHandle 惯例) */
```

- [ ] **Step 2: 定义队列/信号量初始化辅助（供 Task 7 在 MX_FREERTOS_Init 调用）**

在同区域新增一个初始化函数（暂不被调用，本任务只保证可编译）：

```c
/* 创建块池队列 + 收尾信号量,并把 N 个块索引全部塞进 FreeQ。由 MX_FREERTOS_Init 调用。 */
static void AppSdBlockPoolInit(void)
{
  s_free_q  = osMessageQueueNew(APP_SD_BLOCK_POOL_N, sizeof(uint8_t), NULL);
  s_write_q = osMessageQueueNew(APP_SD_WRITEQ_LEN, sizeof(AppSdWriteMsg_t), NULL);
  s_sd_writer_done = osSemaphoreNew(1, 0, NULL);
  for (uint8_t i = 0U; i < APP_SD_BLOCK_POOL_N; i++)
  {
    (void)osMessageQueuePut(s_free_q, &i, 0U, 0U);
  }
}
```

- [ ] **Step 3: （用户）Keil GUI 编译**

Expected: 通过。可能出现 `s_block_pool`/`AppSdBlockPoolInit`/`s_sd_writer_*` 未使用的告警——
本任务可接受（下一任务接线后消除）。若把未使用当错误，临时加 `(void)`。

- [ ] **Step 4: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat(path/sd): 块池+双队列+tagged消息数据结构(未接线)"
```

---

## Task 4: 专职 SD 写线程函数（FatFs 唯一访问者）

新增写线程函数体。它是唯一调用 FatFs 写/同步/收尾的地方。本任务只写函数 + 线程属性，
线程创建在 Task 7，与 drain 接线在 Task 5/6，因此本任务后写线程尚不会跑。

**Files:**
- Modify: `Core/Src/app_freertos.c`（写线程函数放在 `LoggerDrainRing` 之后、`StartLoggerTask` 之前；
  线程属性放在 `loggerTask_attributes` :247 之后）

- [ ] **Step 1: 写线程属性**

在 `loggerTask_attributes`（:247-251）之后新增：

```c
osThreadId_t sdWriterTaskHandle;
const osThreadAttr_t sdWriterTask_attributes = {
  .name = "sdWriterTask",
  .priority = (osPriority_t)osPriorityAboveNormal,  /* = 当前 logger,维持"SD写在传感器(High)之下" */
  .stack_size = 1024 * 4  /* 4KB:f_write/f_sync/WavCheckpoint 调用链 + printf 诊断余量 */
};
```

- [ ] **Step 2: 写线程函数体**

新增（放在 `LoggerDrainRing` 结束后）：

```c
/* 专职 SD 写线程:WriteQ 的唯一消费者,FatFs 的唯一访问者。按 FIFO 串行执行落盘动作,
 * 保证 FatFs 单线程不变量。写数据块前 __DMB() 确保源块对 IDMA 可见(M33 写缓冲,H1 防护)。 */
static void StartSdWriterTask(void *argument)
{
  (void)argument;
  AppSdWriteMsg_t msg;
  for (;;)
  {
    if (osMessageQueueGet(s_write_q, &msg, NULL, osWaitForever) != osOK) continue;

    switch ((AppSdMsgType_t)msg.type)
    {
      case APP_SDMSG_DATA:
      {
        __DMB();   /* H1:源块已落 SRAM,再启动 IDMA(同旧 LoggerDrainRing:1441) */
        FRESULT r = FatFs_SD_LoggerWriteFileIndex(msg.file_idx,
                                                  s_block_pool[msg.block_idx], msg.len);
        if (r != FR_OK)
        {
          printf("[SdWriter] write fail idx=%u %s (%d)\r\n",
                 (unsigned int)msg.file_idx, FatFs_SD_ResultToString(r), (int)r);
          AppFlowStatsRecordWriteFailure();
        }
        /* 无论成败都归还块索引到 FreeQ(否则块泄漏 → FreeQ 枯竭 → drain 背压死锁)。 */
        (void)osMessageQueuePut(s_free_q, &msg.block_idx, 0U, 0U);
        break;
      }
      case APP_SDMSG_SYNC:
        (void)FatFs_SD_LoggerSync();
        break;
      case APP_SDMSG_WAVCKPT:
        (void)FatFs_SD_WavCheckpoint();
        break;
      case APP_SDMSG_STOP:
        FatFs_SD_LoggerStop();
        (void)osSemaphoreRelease(s_sd_writer_done);   /* 解除收尾屏障 */
        break;
      default:
        break;
    }
  }
}
```

- [ ] **Step 3: 前置声明**

在文件上部函数前置声明区（`static void ... LoggerDrainRing` 声明附近 :285）加：

```c
static void StartSdWriterTask(void *argument);
static void AppSdBlockPoolInit(void);
```

- [ ] **Step 4: （用户）Keil GUI 编译**

Expected: 通过。`StartSdWriterTask`/`sdWriterTask_attributes` 未使用告警可接受（Task 7 消除）。

- [ ] **Step 5: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat(path/sd): 专职SD写线程(WriteQ消费者,FatFs唯一访问者)"
```

---

## Task 5: 改造 LoggerDrainRing → 凑块入队（不再直接 f_write）

把 `LoggerDrainRing`（:1380-1450）的尾部从"CopyToBounce→__DMB→WriteFileIndex→Consume"改为
"取空闲块→CopyToBounce 进块→立即 Consume→压 WriteQ(DATA)"。gate/aged/扇区对齐决策逻辑
**完全保留**（:1396-1428 不动）。这是整个改造的核心。

**Files:**
- Modify: `Core/Src/app_freertos.c:1430-1450`（`LoggerDrainRing` 的写入尾段）

- [ ] **Step 1: 替换 want 夹取之后的写入尾段**

将现有（:1430-1450 附近）：

```c
  /* 夹到 bounce 容量... */
  if (want > APP_RING_FLUSH_CHUNK) want = APP_RING_FLUSH_CHUNK;
  n = RingBuf_CopyToBounce(rb, s_sd_bounce, want);
  if (n == 0U) return 0;

  __DMB();
  FRESULT r = FatFs_SD_LoggerWriteFileIndex(file_idx, s_sd_bounce, n);
  *out_res = r;

  if (r == FR_OK)
  {
    RingBuf_Consume(rb, n);
    *rows_since_sync += n;
    rb->last_write_tick = osKernelGetTickCount();
    return 1;
```

替换为（注意：块容量是 `APP_SD_WRITE_BLOCK` 32KB，不再是 64KB bounce）：

```c
  /* 夹到块容量(32KB,512 倍 → 保持扇区对齐);超出余量下一轮 drain 继续。 */
  if (want > APP_SD_WRITE_BLOCK) want = APP_SD_WRITE_BLOCK;

  /* 取空闲块(非阻塞)。取不到 = 池空背压 → 本环本轮不写,数据留环靠环兜底(与旧"环满drop"等价)。 */
  uint8_t blk;
  if (osMessageQueueGet(s_free_q, &blk, NULL, 0U) != osOK)
  {
    return 0;   /* 无空闲块:背压,下轮再来 */
  }

  n = RingBuf_CopyToBounce(rb, s_block_pool[blk], want);
  if (n == 0U)
  {
    (void)osMessageQueuePut(s_free_q, &blk, 0U, 0U);  /* 空环:归还块 */
    return 0;
  }

  /* ★消费即释放:memcpy 进块后立即 Consume 腾空环(防溢出的关键动作),不等写成功。
   * 块此刻成为数据唯一持有者,交给写线程落盘。 */
  RingBuf_Consume(rb, n);
  *rows_since_sync += n;
  rb->last_write_tick = osKernelGetTickCount();

  AppSdWriteMsg_t m = { .type = (uint8_t)APP_SDMSG_DATA, .block_idx = blk,
                        .file_idx = file_idx, .len = n };
  /* 压 WriteQ:队列已按 N+余量设深,正常不满。极端不满(理论不应发生,因块数=N≤队列容量)时
   * 阻塞式 put 保证不丢块(块已持数据)。 */
  (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
  *out_res = FR_OK;
  return 1;
```

> 说明：`out_res` 不再反映真实写结果（写在别的线程异步发生）。drain 侧的 `dr<0` 错误分支
> （:2986-2993）因此不再由 write error 触发——写错误改由写线程内 `AppFlowStatsRecordWriteFailure`
> 记录。这是 Task 6 要相应清理的点。

- [ ] **Step 2: 删除旧 s_sd_bounce 定义**

删除 `app_freertos.c:1365` 的 `static uint8_t s_sd_bounce[APP_RING_FLUSH_CHUNK] ...;`
（已被块池取代）。

⚠️ 注意 `s_sd_bounce` 另有引用点：`app_freertos.c:2793`（`APP_SD_BENCH` 分支）与
`FatFs_SD_RunWriteBenchmark`。检查：

Run:
```bash
cd /d/ChengKe/Sensor_1/Sensor_Proj_V1.0
grep -n "s_sd_bounce" Core/Src/app_freertos.c
```

- [ ] **Step 3: 处理 benchmark 对 s_sd_bounce 的引用**

`APP_SD_BENCH` 默认 0（:2790），该分支不编译进正常固件。但为保证 `#if APP_SD_BENCH` 打开时仍可编译，
把 :2793 的 `FatFs_SD_RunWriteBenchmark(s_sd_bounce, sizeof(s_sd_bounce));` 改为复用块池首块：

```c
  FatFs_SD_RunWriteBenchmark(s_block_pool[0], APP_SD_WRITE_BLOCK);
```

- [ ] **Step 4: （用户）Keil GUI 编译**

Expected: 通过，`s_sd_bounce` 无残留引用。若报未定义 → Step 2 的引用没清干净，回到 Step 2。

- [ ] **Step 5: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "refactor(path/sd): LoggerDrainRing凑块入队取代直接f_write,消费即释放"
```

---

## Task 6: 迁移周期 sync / WAV checkpoint 到 WriteQ

logger 任务里的周期 `f_sync`（:3043-3057）与 `WavCheckpoint`（:3062-3076）现在直接调 FatFs，
违反单线程不变量。改为压 SYNC/WAVCKPT 消息。这样它们相对 DATA 块保持 FIFO 顺序（正确性关键：
sync 必须在它要固化的那些数据块写完之后执行——队列 FIFO 天然保证）。

**Files:**
- Modify: `Core/Src/app_freertos.c:3043-3076`

- [ ] **Step 1: 周期 sync 改为入队**

将 :3043-3057 的 `(void)FatFs_SD_LoggerSync();` 那一段（含 probe 计时）替换为：

```c
    if ((sd_file_open != 0U) && (rows_since_sync >= APP_SD_SYNC_INTERVAL_BYTES))
    {
      AppSdWriteMsg_t m = { .type = (uint8_t)APP_SDMSG_SYNC, .block_idx = 0U,
                            .file_idx = 0U, .len = 0U };
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
      rows_since_sync = 0U;
    }
```

> probe 计时（`[LoggerBlk] SYNC took`）迁移到写线程内 SYNC 分支意义不大且需传时间戳，本任务
> 直接移除该 probe；写线程侧如需可另加。保持简单。

- [ ] **Step 2: WAV checkpoint 改为入队**

将 :3062-3076 替换为：

```c
    if ((sd_file_open != 0U) && ((int32_t)(osKernelGetTickCount() - wav_next_ckpt) >= 0))
    {
      AppSdWriteMsg_t m = { .type = (uint8_t)APP_SDMSG_WAVCKPT, .block_idx = 0U,
                            .file_idx = 0U, .len = 0U };
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
      wav_next_ckpt = osKernelGetTickCount() + 30000U;
    }
```

- [ ] **Step 3: 迁移帧追加路径（Task 1 结论 = A 活跃但窄）**

**Task 1 已判定**：`FatFs_SD_LoggerAppendFrame`（fatfs_sd.c:414）在 SD 会话期**会执行**，且它内部确实
调 `f_write`——但只写一件事：`frame->lsm6dsox.valid` 时，一行温度 CSV（`frame_id,tick_ms,temp_C`）到
`g_log_files[FATFS_SD_FILE_LSM_TMP]`，速率 = 每次 LSM FIFO drain 一行（慢，非逐样本）。acc/gyr 走 ring
不在此路径。这是**第三个 FatFs 写者，流量极小但必须迁移**（否则 logger 线程与写线程并发碰 FatFs）。

做法：新增 `APP_SDMSG_APPENDFRAME` 消息类型，携带 `AppSensorFrame_t`（帧较大 ~百字节，用独立消息
结构避免撑大 DATA 消息）。

**Step 3a**：在 Task 3 的类型定义处（已提交），扩展消息机制。改为在 `s_write_q` 用一个**联合消息**：

```c
/* WriteQ 元素:控制消息(DATA/SYNC/WAVCKPT/STOP)用 hdr;APPENDFRAME 额外携带一帧。
 * 用定长结构(含 frame)让队列元素尺寸固定,简单可靠。frame 仅 APPENDFRAME 有效。 */
typedef struct {
  uint8_t  type;        /* AppSdMsgType_t */
  uint8_t  block_idx;
  uint8_t  file_idx;
  uint32_t len;
  AppSensorFrame_t frame;   /* 仅 APP_SDMSG_APPENDFRAME 使用 */
} AppSdWriteMsg_t;
```

> 注意：这会让 `s_write_q` 每元素含一个 `AppSensorFrame_t`（查 `sensor_snapshot.h` 里其 sizeof；
> 若过大，改用"专门的 TMP_LOW 环 + DATA 块"方案——但温度是文本行、非定长 BIN 帧，走环需要额外
> 格式处理，故此处优先用联合消息，接受队列元素变大。`APP_SD_WRITEQ_LEN`=7 个元素，即使帧 ~200B
> 也仅 ~1.4KB 队列存储，可接受）。

**Step 3b**：新增消息类型枚举值 `APP_SDMSG_APPENDFRAME`（放在 STOP 之后）。

**Step 3c**：写线程 `StartSdWriterTask`（Task 4）的 switch 增加分支：

```c
      case APP_SDMSG_APPENDFRAME:
        (void)FatFs_SD_LoggerAppendFrame(&msg.frame);
        break;
```

**Step 3d**：logger 主循环 :3078-3113 的两处 `AppFrameBufferPop`→`FatFs_SD_LoggerAppendFrame` while
循环，改为 pop 后压 APPENDFRAME 消息（不再直接调 FatFs）：

```c
    while ((sd_file_open != 0U) && (AppAcqIsSdSessionActive() != 0U) && (AppFrameBufferPop(&frame) != 0U))
    {
      AppSdWriteMsg_t m;
      m.type = (uint8_t)APP_SDMSG_APPENDFRAME;
      m.block_idx = 0U; m.file_idx = 0U; m.len = 0U;
      m.frame = frame;
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
      AppFlowStatsRecordFrameWrite(&frame);
    }
```

删除原 while 体内的 `FatFs_SD_LoggerAppendFrame` 调用及其 `if (result != FR_OK)` 错误处理
（:3081-3092）——写错误现由写线程侧处理（`FatFs_SD_LoggerAppendFrame` 返回值可在写线程忽略或
计入 write-failure，与 DATA 一致）。`rows_since_sync++` 保留在 pop 处（用于 sync 间隔计数）。

> 因为此步依赖 Task 3 的 `AppSdWriteMsg_t` 增加 `frame` 字段、Task 4 的 switch 增加分支，
> **实现顺序上 Task 6 会回改 Task 3/4 已提交的结构**——这是可接受的增量演进（同一分支、连续提交）。
> 实现者应在本任务里一并完成 3a/3b/3c/3d 四处改动并保证编译一致。

- [ ] **Step 4: 清理 drain 循环里的 write-error 分支**

`LoggerDrainRing` 改造后不再返回 write error（返回 0 或 1）。检查 drain 主循环 :2986-2993 的
`if (dr < 0)` 分支——现在 `dr` 只会是 0/1。保留该分支无害（死代码），但加一行注释说明写错误已移到
写线程。若想干净，可保留结构不动（LoggerDrainRing 的 `-1` 返回路径在会话停止末块非对齐写仍可能用到，
需确认；保守起见**保留** `dr<0` 分支）。

- [ ] **Step 5: （用户）Keil GUI 编译**

Expected: 通过。

- [ ] **Step 6: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "refactor(path/sd): 周期sync/WAV检查点改经WriteQ串行化(FatFs单线程)"
```

---

## Task 7: 会话收尾屏障（防丢尾块）+ Stop 经写线程

`AppLoggerStopSdSession`（:795-833）现在直接调 `LoggerDrainRing`（已改为入队）+ `FatFs_SD_LoggerStop`
（违反单线程）。改为：强排空末块入队 → 等 WriteQ 全部块归还 FreeQ → 压 STOP 消息 → 等
`s_sd_writer_done`。这样所有尾块 provably 落盘后才 close。

**Files:**
- Modify: `Core/Src/app_freertos.c:795-833`（`AppLoggerStopSdSession`）

- [ ] **Step 1: 重写 AppLoggerStopSdSession 的收尾段**

将 :802-827 的 `if (*sd_file_open != 0U) { ... }` 块体替换为：

```c
  if (*sd_file_open != 0U)
  {
    /* 1) 强排空每个环的末块(min_flush=0 → 全排空,允许非对齐末块),全部压入 WriteQ。
     *    LoggerDrainRing 现在是"凑块入队",do/while 直到所有环再无数据可入队。 */
    {
      uint32_t dummy_rss = 0U;
      FRESULT fr;
      int any;
      do {
        any = 0;
        if (LoggerDrainRing(&g_ring_lsm_acc, 0U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_lsm_gyr, FATFS_SD_FILE_LSM_GYR, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_qma_acc, 3U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_h3_acc,  2U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_mic, FATFS_SD_FILE_MIC_WAV, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_aht_env, 4U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
        if (LoggerDrainRing(&g_ring_mag,     5U, 0U, &dummy_rss, &fr) > 0) { any = 1; }
      } while (any != 0);
    }

    /* 2) 屏障:等所有块归还 FreeQ(= N),确保上面入队的尾块都被写线程写完。
     *    最多等 ~N×单次写最坏(957ms)+余量;给 5s 超时兜底防写线程卡死时死锁。 */
    {
      uint32_t deadline = osKernelGetTickCount() + 5000U;
      while ((osMessageQueueGetCount(s_free_q) < APP_SD_BLOCK_POOL_N) &&
             ((int32_t)(deadline - osKernelGetTickCount()) > 0))
      {
        osDelay(2U);
      }
    }

    /* 3) DeviceCfg 快照仍走 FatFs — 必须在写线程侧或 provably 无并发时做。此处写线程此刻
     *    WriteQ 已空、无 DATA 在飞(屏障已过),logger 独占 FatFs,可安全直接写。 */
    (void)DeviceCfg_WriteCurrentToSD();

    /* 4) 收尾 close 交写线程执行(WAV 头回填/f_close),压 STOP 并等完成信号量。 */
    {
      AppSdWriteMsg_t m = { .type = (uint8_t)APP_SDMSG_STOP, .block_idx = 0U,
                            .file_idx = 0U, .len = 0U };
      (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
      (void)osSemaphoreAcquire(s_sd_writer_done, 5000U);  /* 等 FatFs_SD_LoggerStop 完成 */
    }

    SD_PrintWriteStats();
    AppPrintRuntimeDiag();
    *sd_file_open = 0U;
  }
```

> 注意：`DeviceCfg_WriteCurrentToSD`（原 :822）也调 FatFs。上面放在屏障之后、STOP 之前，此刻无
> 并发 FatFs 访问，安全。若审查认为更稳妥应也经写线程，可另加消息类型——但屏障后直接写已满足
> 单线程语义（同一时刻仅一个线程碰 FatFs）。

- [ ] **Step 2: 确认收尾路径的其他调用点一致**

logger 主循环 :3097-3128 的 `AppAcqDrainPendingStop()` 分支里有独立的 `FatFs_SD_LoggerSync()`
（:3115）与 `AppLoggerStopSdSession`（:3126）。把 :3115 的直接 sync 也改为入队 SYNC 消息（与 Task 6
同样式），保证该分支也不直接碰 FatFs：

```c
      /* final sync 经队列(会在 STOP 前被写线程执行,FIFO 保证顺序) */
      {
        AppSdWriteMsg_t m = { .type = (uint8_t)APP_SDMSG_SYNC, .block_idx = 0U,
                              .file_idx = 0U, .len = 0U };
        (void)osMessageQueuePut(s_write_q, &m, 0U, osWaitForever);
      }
```

删除其后 :3116-3123 的 `if (result != FR_OK)` final-sync 错误打印（result 不再来自此处）。
随后的 `AppLoggerStopSdSession` 调用会做屏障 + STOP，覆盖收尾。

- [ ] **Step 3: （用户）Keil GUI 编译**

Expected: 通过。

- [ ] **Step 4: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat(path/sd): 会话收尾屏障(等WriteQ排空)+Stop经写线程,防丢尾块"
```

---

## Task 8: 激活 — 初始化块池 + 创建写线程

前面所有任务已就绪但未接线。本任务在 `MX_FREERTOS_Init` 里调用 `AppSdBlockPoolInit` 并创建写线程。
接线后整条流水线首次真正运行。

**Files:**
- Modify: `Core/Src/app_freertos.c:374`（信号量区后）与 `:408`（线程创建区）

- [ ] **Step 1: 初始化块池队列（在 ring init 之前或之后均可，需在写线程创建之前）**

在 `RingBuf_Init(...&g_ring_mag...)`（:388）**之后**、`RTOS_QUEUES` 注释附近加：

```c
  /* 双缓冲块池 + 双队列 + 收尾信号量(必须在写线程/生产者跑之前建好) */
  AppSdBlockPoolInit();
```

- [ ] **Step 2: 创建 SD 写线程**

在 `loggerTaskHandle = osThreadNew(StartLoggerTask, ...)`（:408）**之后**加：

```c
  sdWriterTaskHandle  = osThreadNew(StartSdWriterTask,  NULL, &sdWriterTask_attributes);
```

并在下方 printf 诊断区（:412 附近同款）加一行确认：

```c
  printf("[RTOS] sdWriterTask created: %s\r\n", (sdWriterTaskHandle != NULL) ? "ok" : "FAILED");
```

- [ ] **Step 3: 复核创建顺序**

写线程 `StartSdWriterTask` 依赖 `s_write_q/s_free_q/s_sd_writer_done` 已建（Step 1 保证）。
logger 与采集任务入队依赖同上。`AppSdBlockPoolInit` 在所有 `osThreadNew` 之前 → 满足。

- [ ] **Step 4: （用户）Keil GUI 编译**

Expected: 通过，无未使用告警（所有符号现已接线）。

- [ ] **Step 5: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat(path/sd): 激活块池初始化+创建专职SD写线程,双缓冲流水线上线"
```

---

## Task 9: 硬件验证 — 满配长录 + BIN 审计对比（不变量校验）

改造完成，验证零回归、零损坏。掉帧率大概率与基线持平（收益边界已知 ~0.05%）；重点是**没有引入
FatFs 并发损坏、没有丢尾块、没有块泄漏死锁**。

**Files:** 无（硬件 + 审计）

- [ ] **Step 1: （用户）Keil GUI 编译 + 烧录 + 满配长录 ≥10 分钟**

同 Task 0 条件（7 通道 + 96k mic，SD sink）。存串口日志 + 会话目录。

- [ ] **Step 2: 检查启动日志**

串口应见 `[RTOS] sdWriterTask created: ok`。会话停止时不应出现 5s 屏障超时征兆（收尾迅速）。

- [ ] **Step 3: BIN 审计对比基线（Task 0 BASELINE）**

```bash
cd /d/ChengKe/Sensor_1/Sensor_Proj_V1.0
# 用现有 BIN 审计工具跑新会话目录,对每通道:
#   帧丢失率、CRC、frame_id 连续性、文件完整性
```

**通过判据（全部满足）：**
- CRC **零损坏**（硬指标——任何 CRC 错误 = FatFs 并发损坏，立即回退排查）。
- LSM ACC/GYR 丢失率 ≈ 基线 ±1%（不显著变差）。
- QMA/H3/MAG/AHT 不显著变差。
- 会话目录/文件完整，MIC.WAV 可播放，**尾块不丢**（文件末尾无异常截断）。
- frame_id 连续性与基线一致。

- [ ] **Step 4: 检查块泄漏 / 死锁征兆**

多次启停会话（≥3 次），确认每次都能正常 close（无 5s 屏障超时 log）。若某次收尾卡 5s → 块泄漏
（FreeQ 未回到 N），排查写线程归还路径。

- [ ] **Step 5: 判定**

- **全绿** → 改造成功，commit 一个空提交或 tag 标记里程碑（可选），保留分支等用户决定 push。
- **CRC 损坏 / 明显回归** → 回退到 Task 0 记录的锚点 HEAD，附诊断报告，回到对应任务排查。

- [ ] **Step 6: （成功后）记录里程碑 commit（可选）**

```bash
git commit --allow-empty -m "test(path/sd): 双缓冲流水线满配长录验证通过,CRC零损坏,无回归"
```

---

## 完成后的收尾说明

- **push 必须等用户明确指示**（记忆 [[feedback_no_push]]）。全程只 commit。
- 若验证证实掉帧未改善（预期如此），这不代表失败——spec 已明确收益边界在换好卡后兑现，本改造
  交付的是"架构正确 + 零回归"。是否进一步做正交优化（大簇格式化，spec 附录），由用户另行决定。
- 若 Task 1 落在结论 A（帧路径活跃），Task 6 Step 3 需要先补 spec 的 APPENDFRAME 消息设计再实现。
