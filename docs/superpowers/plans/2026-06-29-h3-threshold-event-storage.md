# H3 阈值触发事件存储 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 记录模式新增"阈值触发存储":H3LIS100DL 三轴合矢量越限时,把"冲击前亚秒 + 冲击后 N 秒"作为一次独立会话录到 SD;平时不写盘;USB 路径不受影响。

**Architecture:** 复用 logger 任务现有的"SD 会话开/关由 `sd_session_active` 0↔1 跳变驱动"机制——事件门控只在阈值模式下调制这个标志:ARMED/HOLDOFF 期 `sd_session_active=0`(不开文件,消费者侧裁剪各 ring 只留最近 ~size/2 作预触发);H3 越限 → `=1`(现有逻辑自动 `FindNextSessionDir` 建新 `CKBXnnnn` + 写 `CONFIG.JSN` + flush 留存的预触发段);录满 `post_sec` → `=0`(现有逻辑自动 flush 尾部 + 写快照 + 关文件)。`RingBuf_Write`、生产者、`s_sd_bounce` 轮询写盘字节通路一字不改(SPSC 不破,IDMA 防损坏成果不动)。模式开关复用既有 `trigger_mode` 字段、新增 `ACQ_TRIGGER_THRESHOLD` 值。

**Tech Stack:** STM32U575 + FreeRTOS + FatFs + SDMMC(轮询)+ Keil MDK(microLIB)。无主机单测框架——"测试"= Keil GUI 编译干净 + 设备串口日志/SD 卡产物在机验证。

> **构建约定(项目记忆):** 命令行 `UV4 -build` 在 Keil GUI 开着时读旧日志、不真正构建 → **每个编译步骤都请用户在 Keil GUI 里编译并回贴结果**。microLIB 不支持 `sscanf %lu`(本计划不用 sscanf,用既有 `json_parse_uint`)。commit 随时可做;**push 必须用户明确同意**。

**Spec:** `docs/superpowers/specs/2026-06-29-h3-threshold-event-storage-design.md`

---

## 文件清单

| 文件 | 职责 | 改动 |
|---|---|---|
| `Core/Inc/acq_config.h` | 配置结构/枚举 | 加 `ACQ_TRIGGER_THRESHOLD` 枚举值 + 3 个 `trig_*` 字段 |
| `Core/Src/acq_config.c` | 配置默认/校验 | 默认值;`AcqCfgValidate` 放行 THRESHOLD |
| `Core/Src/device_config.c` | DEVCFG.JSN 读写 | 解析+夹紧+自动补写+模板+两个写盘函数(含 CONFIG.JSN 快照) |
| `Core/Src/fatfs_sd.c` | 会话目录 | `FindNextSessionDir` 增量序号,避免 O(n) 重扫 |
| `Core/Src/app_freertos.c` | 采集/logger 任务 | 事件控制器(状态机)+ H3 检测调用 + logger gate 集成 |

无新增文件 → **不需要改 `.uvprojx`/`.eide`**。

---

## Task 1: acq_config.h/.c — 枚举值、配置字段、默认、校验放行

**Files:**
- Modify: `Core/Inc/acq_config.h`(枚举 `AcqTriggerMode_t`;结构 `AcqConfig_t`)
- Modify: `Core/Src/acq_config.c`(`AcqCfgLoadDefaults`、`AcqCfgValidate`)

- [ ] **Step 1: 给 `AcqTriggerMode_t` 增加 THRESHOLD 值**

`Core/Inc/acq_config.h` 当前(约 42-46 行):
```c
typedef enum {
  ACQ_TRIGGER_NONE     = 0, /* 立即开始 */
  ACQ_TRIGGER_EXTERNAL = 1, /* 等待外部 GPIO 边沿 */
  ACQ_TRIGGER_TIMER    = 2  /* 等待 trigger_delay_ms 后开始 */
} AcqTriggerMode_t;
```
改为:
```c
typedef enum {
  ACQ_TRIGGER_NONE      = 0, /* 立即开始（连续录制，原行为） */
  ACQ_TRIGGER_EXTERNAL  = 1, /* 等待外部 GPIO 边沿（未接通，保留） */
  ACQ_TRIGGER_TIMER     = 2, /* 等待 trigger_delay_ms 后开始（未接通，保留） */
  ACQ_TRIGGER_THRESHOLD = 3  /* H3 合矢量越限事件门控存储 */
} AcqTriggerMode_t;
```

- [ ] **Step 2: 给 `AcqConfig_t` 增加 3 个 trig 参数字段**

`Core/Inc/acq_config.h` 在 `seg_size_mb` 字段之后(约 71 行后)插入:
```c
  uint16_t trig_level_g;               /* 阈值触发：H3 合矢量阈值(g)，仅 THRESHOLD 模式生效，夹紧[1,100] */
  uint16_t trig_post_sec;              /* 阈值触发：后触发录制秒数，夹紧[1,600] */
  uint16_t trig_holdoff_sec;           /* 阈值触发：事件间死区秒数，夹紧[0,600] */
```

- [ ] **Step 3: 设置默认值**

`Core/Src/acq_config.c` 的 `AcqCfgLoadDefaults()` 中,在 `s_cfg.seg_size_mb = 512U;` 之后插入:
```c
  s_cfg.trig_level_g     = 5U;             /* 默认 5g 阈值；trigger_mode=NONE 时不生效 */
  s_cfg.trig_post_sec    = 3U;             /* 默认后触发 3 秒 */
  s_cfg.trig_holdoff_sec = 2U;             /* 默认事件间死区 2 秒 */
```
注意:`trigger_mode` 默认已是 `ACQ_TRIGGER_NONE`(同函数已有 `s_cfg.trigger_mode = ACQ_TRIGGER_NONE;`),即阈值功能默认关闭,不配置者行为不变。

- [ ] **Step 4: `AcqCfgValidate` 放行 THRESHOLD(否则 `AcqConfig_Set` 拒绝整份配置)**

`Core/Src/acq_config.c` 当前(约 142-147 行):
```c
  if ((cfg->trigger_mode != ACQ_TRIGGER_NONE) &&
      (cfg->trigger_mode != ACQ_TRIGGER_EXTERNAL) &&
      (cfg->trigger_mode != ACQ_TRIGGER_TIMER))
  {
    return -1;
  }
```
改为:
```c
  if ((cfg->trigger_mode != ACQ_TRIGGER_NONE) &&
      (cfg->trigger_mode != ACQ_TRIGGER_EXTERNAL) &&
      (cfg->trigger_mode != ACQ_TRIGGER_TIMER) &&
      (cfg->trigger_mode != ACQ_TRIGGER_THRESHOLD))
  {
    return -1;
  }
```

- [ ] **Step 5: Keil GUI 编译验证**

请在 Keil GUI 编译。预期:**0 error**(纯结构/枚举扩展)。把结果贴回。

- [ ] **Step 6: Commit**

```bash
git add Core/Inc/acq_config.h Core/Src/acq_config.c
git commit -m "feat(acq): 加 ACQ_TRIGGER_THRESHOLD 枚举与 trig_* 配置字段"
```

---

## Task 2: device_config.c — 解析/夹紧/自动补写/模板/两个写盘函数

镜像既有 `seg_size_mb` 的五处样板。位掩码扩展:bit0=battery、bit1=seg、**bit2=trigger**。

**Files:**
- Modify: `Core/Src/device_config.c`(`kCfgTemplate`、`DeviceCfg_ParseAndApply`、`DeviceCfg_LoadFromSD` 补写段、`DeviceCfg_WriteCurrentToSD`、`DeviceCfg_WriteConfigToDir`)

- [ ] **Step 1: 模板 `kCfgTemplate` 增加 trigger 块**

在 `seg_size_mb` 块之后(当前约 58-59 行,`_doc_seg_size_mb` 行与其后空行 `"\r\n"` 之间)插入新块。即把:
```c
"  \"_doc_seg_size_mb\": \"SD每个数据文件分段大小(MB)：单文件写满即新建续段(如 LSM_IMU.BIN→IMU0002.BIN→...)，各文件独立计数；0=不分段(单文件)。麦克风MIC.WAV不分段\",\r\n"
"\r\n"
"  \"_s2\": \"==================== LSM6DSOX 六轴IMU ====================\",\r\n"
```
改为:
```c
"  \"_doc_seg_size_mb\": \"SD每个数据文件分段大小(MB)：单文件写满即新建续段(如 LSM_IMU.BIN→IMU0002.BIN→...)，各文件独立计数；0=不分段(单文件)。麦克风MIC.WAV不分段\",\r\n"
"\r\n"
"  \"_s_trig\": \"==================== 录制触发 ====================\",\r\n"
"  \"trigger_mode\": \"none\",\r\n"
"  \"_doc_trigger_mode\": \"录制触发模式：none=连续录制(原行为) / threshold=H3冲击阈值事件存储(平时不录,合矢量越限才录一段;USB不受影响)\",\r\n"
"  \"_options_trigger_mode\": [\"none\", \"threshold\"],\r\n"
"  \"trig_level_g\": 5,\r\n"
"  \"_doc_trig_level_g\": \"阈值(g)：H3三轴合矢量超过此值即触发；仅threshold模式生效\",\r\n"
"  \"trig_post_sec\": 3,\r\n"
"  \"_doc_trig_post_sec\": \"后触发录制秒数：触发后继续录多少秒\",\r\n"
"  \"trig_holdoff_sec\": 2,\r\n"
"  \"_doc_trig_holdoff_sec\": \"事件间死区秒数：一个事件录完后静默多久才允许下次触发\",\r\n"
"\r\n"
"  \"_s2\": \"==================== LSM6DSOX 六轴IMU ====================\",\r\n"
```

- [ ] **Step 2: `DeviceCfg_ParseAndApply` 解析 trigger 字段 + 夹紧 + 标记位**

更新函数头注释(约 275-276 行):
```c
/* 返回位掩码：bit0=文件已有 battery 字段，bit1=已有 seg_size_mb，bit2=已有 trigger_mode。
 * 缺失的位由调用者补写进配置文件。 */
```
在 `seg_size_mb` 解析块之后(约 330 行,`/* ---- lsm6dsox 子对象 ---- */` 之前)插入:
```c
    /* ---- 录制触发：trigger_mode + trig_* 标量 ---- */
    int has_trig = 0;
    p = json_find_value(buf, "trigger_mode");
    if (p != NULL && json_parse_string(p, s, sizeof(s)))
    {
        has_trig = 1;
        if      (strcmp(s, "threshold") == 0) cfg.trigger_mode = ACQ_TRIGGER_THRESHOLD;
        else if (strcmp(s, "none")      == 0) cfg.trigger_mode = ACQ_TRIGGER_NONE;
    }
    p = json_find_value(buf, "trig_level_g");
    if (json_parse_uint(p, &v))
    {
        if (v < 1U)   v = 1U;
        if (v > 100U) v = 100U;
        cfg.trig_level_g = (uint16_t)v;
    }
    p = json_find_value(buf, "trig_post_sec");
    if (json_parse_uint(p, &v))
    {
        if (v < 1U)   v = 1U;
        if (v > 600U) v = 600U;
        cfg.trig_post_sec = (uint16_t)v;
    }
    p = json_find_value(buf, "trig_holdoff_sec");
    if (json_parse_uint(p, &v))
    {
        if (v > 600U) v = 600U;
        cfg.trig_holdoff_sec = (uint16_t)v;
    }
```
注意:`s` 是 `char[16]`,"threshold"(9 字符)放得下。

- [ ] **Step 3: 更新 `DeviceCfg_ParseAndApply` 返回值**

当前(约 449 行):
```c
    return (has_battery ? 1 : 0) | (has_seg ? 2 : 0);
```
改为:
```c
    return (has_battery ? 1 : 0) | (has_seg ? 2 : 0) | (has_trig ? 4 : 0);
```

- [ ] **Step 4: `DeviceCfg_LoadFromSD` 自动补写——纳入 trigger 缺失**

更新补写触发条件与注释(约 519-522 行):
```c
    int present = DeviceCfg_ParseAndApply(s_buf);  /* bit0=battery, bit1=seg, bit2=trigger */

    /* 旧配置文件可能缺少 trigger / seg / battery：在末尾的 } 前一次性补齐缺失项 */
    if (((present & 1) == 0) || ((present & 2) == 0) || ((present & 4) == 0))
```
把补写缓冲扩大(约 537 行)`static char s_patch[640];` → `static char s_patch[1024];`
在 seg 补写块**之前**(约 547 行 `if ((present & 2) == 0)` 之前)插入 trigger 补写块:
```c
            if ((present & 4) == 0)   /* 补 trigger 块 */
            {
                adv = snprintf(w, cap,
                    "%s\r\n"
                    "  \"trigger_mode\": \"%s\",\r\n"
                    "  \"_doc_trigger_mode\": \"录制触发模式：none=连续录制 / threshold=H3冲击阈值事件存储(平时不录,越限才录;USB不受影响)\",\r\n"
                    "  \"_options_trigger_mode\": [\"none\", \"threshold\"],\r\n"
                    "  \"trig_level_g\": %u,\r\n"
                    "  \"trig_post_sec\": %u,\r\n"
                    "  \"trig_holdoff_sec\": %u",
                    first ? "" : ",\r\n",
                    (cfg2.trigger_mode == ACQ_TRIGGER_THRESHOLD) ? "threshold" : "none",
                    (unsigned int)cfg2.trig_level_g,
                    (unsigned int)cfg2.trig_post_sec,
                    (unsigned int)cfg2.trig_holdoff_sec);
                if (adv > 0 && (size_t)adv < cap) { w += adv; cap -= (size_t)adv; first = 0; }
            }
```
更新补写成功日志(约 588 行):
```c
                        printf("[DevCfg] 已向配置文件补写缺失字段(trigger/seg/battery)\r\n");
```

- [ ] **Step 5: `DeviceCfg_WriteCurrentToSD` 输出 trigger 块(根 DEVCFG.JSN 回写)**

在格式串里 `seg_size_mb` 块之后(约 642-643 行,`_doc_seg_size_mb` 行与其后 `"\r\n"`/`_s2` 之间)插入:
```c
        "  \"trigger_mode\": \"%s\",\r\n"
        "  \"_doc_trigger_mode\": \"录制触发模式：none=连续录制 / threshold=H3冲击阈值事件存储(平时不录,越限才录;USB不受影响)\",\r\n"
        "  \"_options_trigger_mode\": [\"none\", \"threshold\"],\r\n"
        "  \"trig_level_g\": %u,\r\n"
        "  \"_doc_trig_level_g\": \"阈值(g)：H3三轴合矢量超过此值即触发；仅threshold模式生效\",\r\n"
        "  \"trig_post_sec\": %u,\r\n"
        "  \"_doc_trig_post_sec\": \"后触发录制秒数\",\r\n"
        "  \"trig_holdoff_sec\": %u,\r\n"
        "  \"_doc_trig_holdoff_sec\": \"事件间死区秒数\",\r\n"
        "\r\n"
```
在参数列表 `(unsigned long)cfg.seg_size_mb,` 之后(约 706 行)插入对应实参:
```c
        (cfg.trigger_mode == ACQ_TRIGGER_THRESHOLD) ? "threshold" : "none",
        (unsigned int)cfg.trig_level_g,
        (unsigned int)cfg.trig_post_sec,
        (unsigned int)cfg.trig_holdoff_sec,
```
(顺序须与格式串里 `%s/%u/%u/%u` 出现顺序一致:mode、level、post、holdoff。)

- [ ] **Step 6: `DeviceCfg_WriteConfigToDir` 会话快照 CONFIG.JSN 记录触发模式参数(用户要求的"区分"靠这个)**

在格式串开头 `"{\r\n"` 之后、`"  \"lsm6dsox\": {\r\n"` 之前(约 769-770 行)插入:
```c
        "  \"trigger_mode\": \"%s\",\r\n"
        "  \"trig_level_g\": %u,\r\n"
        "  \"trig_post_sec\": %u,\r\n"
        "  \"trig_holdoff_sec\": %u,\r\n"
```
在参数列表最前(约 801 行 `(unsigned int)cfg.lsm6dsox.enabled,` 之前)插入:
```c
        (cfg.trigger_mode == ACQ_TRIGGER_THRESHOLD) ? "threshold" : "none",
        (unsigned int)cfg.trig_level_g,
        (unsigned int)cfg.trig_post_sec,
        (unsigned int)cfg.trig_holdoff_sec,
```

- [ ] **Step 7: Keil GUI 编译验证**

请在 Keil GUI 编译。预期:**0 error**。重点核对 `WriteCurrentToSD`/`WriteConfigToDir` 两处 `snprintf` 的 `%` 占位符数量与实参数量一致(漏一个就会乱序/栈读越界)。结果贴回。

- [ ] **Step 8: Commit**

```bash
git add Core/Src/device_config.c
git commit -m "feat(devcfg): DEVCFG.JSN/CONFIG.JSN 增加 trigger_mode 与 trig_* 解析与回写"
```

---

## Task 3: fatfs_sd.c — FindNextSessionDir 增量序号(防 O(n) 重扫放大丢帧)

阈值模式每个事件建一个新会话目录;若每次都从 `CKBX0001` 逐个 `f_stat` 重扫,事件攒多后开目录耗时 O(n) → 放大开文件延迟窗口。缓存上次序号,从 last+1 起扫。

**Files:**
- Modify: `Core/Src/fatfs_sd.c`(`FatFs_SD_FindNextSessionDir`,约 127-153 行)

- [ ] **Step 1: 加文件作用域缓存并从缓存起扫**

当前:
```c
static FRESULT FatFs_SD_FindNextSessionDir(char *dir, size_t dir_size)
{
  unsigned int index;
  FILINFO info;

  for (index = 1U; index <= FATFS_SD_SESSION_MAX; index++)
  {
```
改为:
```c
/* 增量缓存上次分配到的会话序号：阈值事件模式会反复建目录，避免每次从 1 重扫(O(n))。
 * 只增不减；即使旧目录被删也不回收序号(始终向上)，9999 上限内可接受。 */
static unsigned int s_last_session_idx = 0U;

static FRESULT FatFs_SD_FindNextSessionDir(char *dir, size_t dir_size)
{
  unsigned int index;
  FILINFO info;

  for (index = s_last_session_idx + 1U; index <= FATFS_SD_SESSION_MAX; index++)
  {
```
并在原 `return FR_OK;  /* 目录不存在，可用 */` 之前记录缓存。当前:
```c
    FRESULT r = f_stat(dir, &info);
    if (r == FR_NO_FILE)
    {
      return FR_OK;  /* 目录不存在，可用 */
    }
```
改为:
```c
    FRESULT r = f_stat(dir, &info);
    if (r == FR_NO_FILE)
    {
      s_last_session_idx = index;   /* 记住本次分配，下次从 index+1 起扫 */
      return FR_OK;                 /* 目录不存在，可用 */
    }
```

- [ ] **Step 2: Keil GUI 编译验证**

请在 Keil GUI 编译。预期 0 error。结果贴回。

- [ ] **Step 3: Commit**

```bash
git add Core/Src/fatfs_sd.c
git commit -m "perf(fatfs): FindNextSessionDir 增量序号,避免事件模式 O(n) 重扫"
```

---

## Task 4: app_freertos.c — 事件门控控制器(状态机)

新增一段自包含的事件控制器:状态机 + H3 越限检测 + SD gate + ring 裁剪。放在 `StartLoggerTask` 之前(约 3360 行之前),此处已在 `RingBuf_Available`/`RingBuf_Consume`(约 1414/1479 行)之后,可直接调用它们。

**Files:**
- Modify: `Core/Src/app_freertos.c`(在 `void StartLoggerTask(void *argument)` 定义之前插入新代码块)

- [ ] **Step 1: 插入事件控制器代码块**

在 `StartLoggerTask` 函数定义那一行之前插入:
```c
/* ============================================================================
 *  H3 阈值触发事件门控控制器
 *  - H3 采集任务每样本调 AppEvtH3Sample()，越限则自增 g_evt_trig_seq(唯一写者)。
 *  - logger 任务每轮调 AppEvtSdGate() 推进状态机，返回"此刻是否应开 SD 会话"。
 *  - ARMED/HOLDOFF 期 logger 调 AppEvtTrimRings() 在消费者侧裁剪各 ring，只留最近
 *    ~size/2 作预触发(不碰 RingBuf_Write/生产者，SPSC 不破)。
 *  时间基准用 AppTime_GetEpochUs()(RTOS tick 慢~19%，定时不可用 tick)；logger 每轮
 *  仅调用一次，频率低，符合 AppTime 限流要求。
 * ========================================================================= */
typedef enum { EVT_OFF = 0, EVT_ARMED, EVT_REC, EVT_HOLDOFF } AppEvtState_t;

static volatile uint32_t g_evt_trig_seq = 0U;   /* H3 任务自增(唯一写者)，logger 只读 */
static uint32_t  g_evt_last_seen_seq = 0U;       /* logger 上次观察到的 seq */
static uint64_t  g_evt_level_mg2     = 0U;       /* 阈值平方(mg²)，BeginSession 预算 */
static uint64_t  g_evt_post_us       = 0U;       /* 后触发窗口(us) */
static uint64_t  g_evt_holdoff_us    = 0U;       /* 死区(us) */
static uint64_t  g_evt_deadline_us   = 0U;       /* REC:post 截止 / HOLDOFF:死区截止 */
static AppEvtState_t g_evt_state      = EVT_OFF;
static uint8_t   g_evt_mode          = 0U;       /* 1=本次采集为阈值模式 */

/* 采集启动时调用：按配置决定是否进入阈值模式并预算阈值/窗口。 */
static void AppEvtBeginSession(const AcqConfig_t *cfg)
{
  if (cfg != NULL && cfg->trigger_mode == ACQ_TRIGGER_THRESHOLD)
  {
    uint64_t mg = (uint64_t)cfg->trig_level_g * 1000U;   /* g → mg */
    g_evt_level_mg2  = mg * mg;
    g_evt_post_us    = (uint64_t)cfg->trig_post_sec    * 1000000ULL;
    g_evt_holdoff_us = (uint64_t)cfg->trig_holdoff_sec * 1000000ULL;
    g_evt_last_seen_seq = g_evt_trig_seq;   /* 丢弃布防前的历史触发 */
    g_evt_state = EVT_ARMED;
    g_evt_mode  = 1U;
    if (cfg->h3lis100dl.enabled == 0U)
    {
      printf("[Evt] 警告：阈值模式但 H3 未启用，将永不触发！请在 DEVCFG.JSN 启用 h3lis100dl\r\n");
    }
    printf("[Evt] 阈值模式布防: level=%ug post=%us holdoff=%us\r\n",
           (unsigned int)cfg->trig_level_g,
           (unsigned int)cfg->trig_post_sec,
           (unsigned int)cfg->trig_holdoff_sec);
  }
  else
  {
    g_evt_mode  = 0U;
    g_evt_state = EVT_OFF;
  }
}

/* 采集停止时调用。 */
static void AppEvtEndSession(void)
{
  g_evt_mode  = 0U;
  g_evt_state = EVT_OFF;
}

/* H3 采集任务每样本调用:合矢量越限即自增触发序号。整数运算,~400Hz,可忽略。 */
static void AppEvtH3Sample(float mx, float my, float mz)
{
  if (g_evt_mode == 0U) return;
  int64_t ix = (int64_t)mx, iy = (int64_t)my, iz = (int64_t)mz;
  uint64_t mag2 = (uint64_t)(ix * ix + iy * iy + iz * iz);
  if (mag2 > g_evt_level_mg2)
  {
    g_evt_trig_seq++;   /* 唯一写者(H3 任务),logger 只读,32 位读原子 */
  }
}

/* logger 任务每轮调用:推进状态机,返回 1=此刻应开 SD 会话(REC),0=否(ARMED/HOLDOFF)。
 * 非阈值模式不应调用本函数(返回 1 兜底)。 */
static uint8_t AppEvtSdGate(void)
{
  if (g_evt_mode == 0U) return 1U;

  uint32_t seq      = g_evt_trig_seq;
  uint8_t  new_trig = (uint8_t)(seq != g_evt_last_seen_seq);
  g_evt_last_seen_seq = seq;
  uint64_t now = AppTime_GetEpochUs();

  switch (g_evt_state)
  {
    case EVT_ARMED:
      if (new_trig)
      {
        g_evt_deadline_us = now + g_evt_post_us;
        g_evt_state = EVT_REC;
        printf("[Evt] 触发! 开始录制事件 (post=%lus)\r\n",
               (unsigned long)(g_evt_post_us / 1000000ULL));
        return 1U;
      }
      return 0U;

    case EVT_REC:
      if (new_trig) { g_evt_deadline_us = now + g_evt_post_us; }  /* 再次冲击 → 延长 */
      if (now >= g_evt_deadline_us)
      {
        g_evt_deadline_us = now + g_evt_holdoff_us;
        g_evt_state = EVT_HOLDOFF;
        printf("[Evt] 事件录制结束,进入死区 %lus\r\n",
               (unsigned long)(g_evt_holdoff_us / 1000000ULL));
        return 0U;
      }
      return 1U;

    case EVT_HOLDOFF:
      if (now >= g_evt_deadline_us) { g_evt_state = EVT_ARMED; }
      return 0U;   /* 死区内忽略触发(new_trig 已被吞掉) */

    default:
      return 0U;
  }
}

/* ARMED/HOLDOFF 期消费者侧裁剪:每条 ring 只留最近 ~size/2(预触发),丢弃更旧字节。
 * 仅 logger(消费者)动 rd_idx,生产者不变,SPSC 不破。按字节裁剪可能在头部留半帧,
 * 事件预触发首条记录可能不完整(BIN 由 CRC 跳过/CSV 跳首行),属可接受的边界瑕疵。 */
static void AppEvtTrimRing(AppRingBuffer_t *rb)
{
  uint32_t avail = RingBuf_Available(rb);
  uint32_t keep  = rb->size / 2U;
  if (avail > keep) { RingBuf_Consume(rb, avail - keep); }
}

static void AppEvtTrimRings(void)
{
  AppEvtTrimRing(&g_ring_lsm_imu);
  AppEvtTrimRing(&g_ring_qma_acc);
  AppEvtTrimRing(&g_ring_h3_acc);
  AppEvtTrimRing(&g_ring_mic);
  AppEvtTrimRing(&g_ring_aht_env);
  AppEvtTrimRing(&g_ring_mag);
}
```

- [ ] **Step 2: Keil GUI 编译验证**

请在 Keil GUI 编译。预期 0 error;可能有"函数未被调用"类告警(Task 5/6 才接线),可忽略。确认 `AppTime_GetEpochUs`/`RingBuf_Available`/`RingBuf_Consume`/`g_ring_*` 均能解析(它们都在本文件、本块之前定义)。结果贴回。

- [ ] **Step 3: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat(evt): H3 阈值事件门控控制器(状态机+检测+ring裁剪)"
```

---

## Task 5: app_freertos.c — 在 H3 采集支路接入越限检测

**Files:**
- Modify: `Core/Src/app_freertos.c`(H3 读样本处,约 2746-2755 行)

- [ ] **Step 1: H3 读到样本后立即喂给检测器**

当前(约 2746-2754 行):
```c
    if (ret == 0)
    {
      /* Monotonic timestamp: seed with DWT on first sample, then +interval per sample. */
      static uint32_t h3_ts_us = 0U;
      static uint8_t h3_ts_init = 0U;
      if (h3_ts_init == 0U) { h3_ts_us = AppDwtUs(); h3_ts_init = 1U; }
      h3_ts_us += s_h3_odr_interval_us;

      uint32_t fid = ++g_h3_frame_id_counter;
```
改为(在时间戳维护之后、构帧之前插入一行调用):
```c
    if (ret == 0)
    {
      /* 阈值事件门控:每样本判合矢量越限(仅阈值模式生效,非阈值模式立即返回)。 */
      AppEvtH3Sample(data.acc_mg[0], data.acc_mg[1], data.acc_mg[2]);

      /* Monotonic timestamp: seed with DWT on first sample, then +interval per sample. */
      static uint32_t h3_ts_us = 0U;
      static uint8_t h3_ts_init = 0U;
      if (h3_ts_init == 0U) { h3_ts_us = AppDwtUs(); h3_ts_init = 1U; }
      h3_ts_us += s_h3_odr_interval_us;

      uint32_t fid = ++g_h3_frame_id_counter;
```
(`data` 是该作用域内的 `H3LIS100DL_Data_t`,`acc_mg[3]` 为浮点 mg,已在 CSV/BIN 路径使用。)

- [ ] **Step 2: Keil GUI 编译验证**

请在 Keil GUI 编译。预期 0 error。结果贴回。

- [ ] **Step 3: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat(evt): H3 采集支路接入合矢量越限检测"
```

---

## Task 6: app_freertos.c — logger 任务集成门控(核心接线)

把事件门控接进 `StartLoggerTask`:调制 `sd_session_active`、维持预触发裁剪、阈值模式跳过会话开启时的 `RingBuf_Reset`。

**Files:**
- Modify: `Core/Src/app_freertos.c`(`StartLoggerTask` 循环顶部约 3395-3402 行;会话开启 reset 块约 3452-3464 行)

- [ ] **Step 1: 循环顶部——生命周期 + gate 调制 `sd_session_active`**

当前(约 3393-3402 行):
```c
  for (;;)
  {
    AppAcqControl_t acq;
    uint8_t sd_session_active;
    uint8_t acq_running;

    AppAcqCheckAutoStop();
    AppAcqGetCopy(&acq);
    acq_running = acq.running;
    sd_session_active = (uint8_t)((acq_running != 0U) && (acq.sink == APP_ACQ_SINK_SD));
```
改为:
```c
  static uint8_t s_prev_sd_acq = 0U;   /* 上一轮 SD 采集是否活跃,用于检测起止沿 */
  for (;;)
  {
    AppAcqControl_t acq;
    uint8_t sd_session_active;
    uint8_t acq_running;

    AppAcqCheckAutoStop();
    AppAcqGetCopy(&acq);
    acq_running = acq.running;

    uint8_t sd_acq_active = (uint8_t)((acq_running != 0U) && (acq.sink == APP_ACQ_SINK_SD));

    /* 事件门控生命周期:SD 采集起/止沿驱动 BeginSession/EndSession */
    if (sd_acq_active != 0U && s_prev_sd_acq == 0U)
    {
      AcqConfig_t ecfg; AcqConfig_GetCopy(&ecfg);
      AppEvtBeginSession(&ecfg);
    }
    else if (sd_acq_active == 0U && s_prev_sd_acq != 0U)
    {
      AppEvtEndSession();
    }
    s_prev_sd_acq = sd_acq_active;

    if (g_evt_mode != 0U && sd_acq_active != 0U)
    {
      /* 阈值模式:gate 决定此刻是否开 SD 会话;未录时维持预触发裁剪 */
      uint8_t gate = AppEvtSdGate();
      sd_session_active = gate;
      if (gate == 0U) { AppEvtTrimRings(); }
    }
    else
    {
      /* 非阈值模式:维持原行为 */
      sd_session_active = sd_acq_active;
    }
```
说明:`sd_session_active` 之后被 3404 行起的现有逻辑使用——0→1 自动建新 `CKBXnnnn` + 写 `CONFIG.JSN`;1→0 自动 flush 尾部 + `WriteCurrentToSD` + `LoggerStop`。每个事件 = gate 的一次 1 脉冲 = 一个会话目录。

- [ ] **Step 2: 会话开启时阈值模式跳过 RingBuf_Reset 与帧号清零(保住预触发与帧号连续)**

当前(约 3452-3464 行):
```c
        /* Discard data buffered between sessions so each file starts clean
         * and frame_id/tick_ms in the new session align with real samples. */
        RingBuf_Reset(&g_ring_lsm_imu);
        RingBuf_Reset(&g_ring_qma_acc);
        RingBuf_Reset(&g_ring_h3_acc);
        RingBuf_Reset(&g_ring_mic);
        RingBuf_Reset(&g_ring_aht_env);
        RingBuf_Reset(&g_ring_mag);
        g_lsm_frame_id_counter = 0U;
        g_qma_frame_id_counter = 0U;
        g_h3_frame_id_counter  = 0U;
        g_aht_frame_id_counter = 0U;
        g_mag_frame_id_counter = 0U;
```
改为(整体包到 `if (g_evt_mode == 0U)` 里;阈值模式保留 ring 中的预触发段、帧号跨事件连续):
```c
        /* Discard data buffered between sessions so each file starts clean
         * and frame_id/tick_ms in the new session align with real samples.
         * 阈值模式例外:ring 里是要落盘的预触发段,绝不能清;帧号跨事件保持连续。 */
        if (g_evt_mode == 0U)
        {
          RingBuf_Reset(&g_ring_lsm_imu);
          RingBuf_Reset(&g_ring_qma_acc);
          RingBuf_Reset(&g_ring_h3_acc);
          RingBuf_Reset(&g_ring_mic);
          RingBuf_Reset(&g_ring_aht_env);
          RingBuf_Reset(&g_ring_mag);
          g_lsm_frame_id_counter = 0U;
          g_qma_frame_id_counter = 0U;
          g_h3_frame_id_counter  = 0U;
          g_aht_frame_id_counter = 0U;
          g_mag_frame_id_counter = 0U;
        }
```
(其余 `g_h3_irq_count`/`SD_ResetWriteStats`/`s_session_start_us`/`AppAcqResetSessionTimer` 等保持不变,每个事件正常重置统计/计时。)

- [ ] **Step 3: Keil GUI 编译验证**

请在 Keil GUI 编译。预期 0 error。核对:`g_evt_mode`、`AppEvtSdGate`、`AppEvtTrimRings`、`AppEvtBeginSession/EndSession` 均在 Task 4 中定义且在本文件、`StartLoggerTask` 之前。结果贴回。

- [ ] **Step 4: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat(evt): logger 任务接入阈值事件门控(调制会话+预触发裁剪)"
```

---

## Task 7: 在机验证(无单测,靠串口日志 + SD 卡产物)

**前置:** 用读卡器编辑 SD 卡根目录 `DEVCFG.JSN`,设 `"trigger_mode": "threshold"`、`"trig_level_g": 3`(便于手敲触发)、`"trig_post_sec": 3`、`"trig_holdoff_sec": 2`,确保 `h3lis100dl.enabled=1`、`sink="SD"`、`boot_acquire=1`。插卡上电。

- [ ] **Step 1: 验证布防日志**

预期串口出现:`[Evt] 阈值模式布防: level=3g post=3s holdoff=2s`。静置(不晃)时**不应**出现 `[Logger] starting SD session`,SD 上**不新增** CKBX 目录。
若静置仍反复建目录 → 阈值太低/噪声,调高 `trig_level_g`。

- [ ] **Step 2: 验证触发→事件目录**

敲击/甩动设备一次。预期:`[Evt] 触发! 开始录制事件` → 约 3s 后 `[Evt] 事件录制结束,进入死区 2s`。SD 上新增一个 `CKBXnnnn` 目录,内含传感器文件 + `CONFIG.JSN`。

- [ ] **Step 3: 验证 CONFIG.JSN 标明模式**

读卡查看该事件目录的 `CONFIG.JSN`,开头应有:
```
"trigger_mode": "threshold",
"trig_level_g": 3,
"trig_post_sec": 3,
"trig_holdoff_sec": 2,
```

- [ ] **Step 4: 验证预触发段存在**

打开事件的 H3 数据文件,确认起始数据时间戳/帧号**早于**触发时刻(即包含冲击前的亚秒历史)。BIN 用 Task 之外的解析工具或十六进制查看;CSV 直接看首行 datetime 是否早于敲击瞬间。允许首条记录不完整(边界裁剪)。

- [ ] **Step 5: 验证连续事件与死区**

连续敲两次(间隔 >post+holdoff)→ 应得两个独立 CKBX 目录。死区内(结束后 <2s)再敲 → 不应新建目录。

- [ ] **Step 6: 验证 USB 不受影响 + 回归连续模式**

(a) USB 模式或 BOTH 下,WCID 流式输出与改动前一致(USB 不被门控)。
(b) 把 `trigger_mode` 改回 `"none"`,确认恢复连续录制(开机/acq_start 后立即持续建会话写盘),与本特性引入前行为一致。
(c) 看 `AppPrintRuntimeDiag` 的 ring overrun 统计:事件边界处 LSM 不应出现明显 drop(开文件延迟未溢出)。

- [ ] **Step 7: 记录验证结论**

把 Step 1-6 的串口日志与 SD 产物结论贴回。如有异常(误触发/漏录/预触发缺失/边界丢帧),记录现象供调参或修正。

---

## 自检(写计划者对照 spec)

- **Spec 覆盖:** 配置字段(Task 2)、状态机 ARMED/REC/HOLDOFF(Task 4)、H3 合矢量检测(Task 4+5)、预触发消费者侧裁剪(Task 4+6)、每事件 = 普通会话目录(Task 6 复用现有机制)、CONFIG.JSN 记录模式(Task 2 Step 6)、FindNextSessionDir 增量(Task 3)、复用 trigger_mode+THRESHOLD(Task 1)、USB 不受影响(Task 6 仅调制 SD 路径)、风险①②③缓解均有对应步骤。✓
- **类型一致:** `trigger_mode` 用既有 `AcqTriggerMode_t`;`g_evt_*` 类型在 Task 4 定义并在 Task 5/6 一致使用;`AppEvtH3Sample(float,float,float)` 与 Task 5 调用实参(`data.acc_mg[]` 浮点)一致;`AppEvtSdGate`/`AppEvtTrimRings`/`AppEvtBeginSession`/`AppEvtEndSession` 签名前后一致。✓
- **占位符:** 无 TBD/TODO;每步均含真实代码与精确行号锚点。✓
- **已知边界瑕疵(spec 已认可):** 预触发首条记录可能半帧;开文件延迟极端慢卡可能丢几帧(overrun 监测兜底)。
