# 场景二配置整治 + 开机即采 执行计划

日期：2026-06-25
分支：feat/aht20-lis2mdl-sensors

## 背景（已核实）

入口 `AppAcqStart(sink, duration_ms)` 只认入参；带 trigger/ARMED 状态机的
`AcqConfig_Start()` 从未被调用（死代码）。导致 DEVCFG.JSN 的 **7 个顶层字段全是
"能配、能回显、不生效"**：`sample_rate_hz`、`sink`、`storage_mode`、`trigger_mode`、
`trigger_delay_ms`、`duration_ms`、`sd_ring_max_bytes`。

实测铁证：配 `sample_rate_hz=1000`，但抓出 LSM 6704 / QMA 1568 / H3 398 / MAG 55——
没有一个是 1000（每颗传感器各自用子对象里的 `odr_hz`，这部分是真生效的）。

## 目标（本轮范围）

1. **配置文件里每个字段都真正生效**——删假字段、接通有用字段。
2. **开机即采**：`boot_acquire` 写进配置，开机读到=1 就自动开始采集，`duration_ms`
   控制采多久自动停。
3. **配置文件重构**：分区 + 每字段 `_doc` 说明 + `_options` 可选值列表。

### 本轮不做（留后续）
- 循环存储 RING 的真正实现（`storage_mode` / `sd_ring_max_bytes`）。
- 阈值触发存储 / 外部 GPIO 触发。
- LED 多状态。
→ 这些字段本轮**从配置文件移除**，等真正实现时再带回（避免继续摆假字段）。

## 字段处置表

| 字段 | 处置 |
|---|---|
| `sample_rate_hz`（顶层） | **删除**：每传感器已有独立 odr，留着只会误导 |
| `sink` | **接通**：开机即采用它决定出口（SD/USB） |
| `duration_ms` | **接通**：开机即采用它做到点自动停（auto-stop 机制已现成） |
| `boot_acquire`（新增） | **新增**：0/1，开机自动采集开关 |
| `storage_mode` | **移除**（RING 未实现，留给后续） |
| `sd_ring_max_bytes` | **移除**（同上） |
| `trigger_mode` | **移除**（触发未实现，留给后续） |
| `trigger_delay_ms` | **移除**（同上） |
| per-sensor 子对象 | **保持**（已真生效），补全 `_doc`/`_options` |

## 目标 DEVCFG.JSN 结构（重构后）

```jsonc
{
  "_file": "Sensor Box 设备配置；开机读取并应用。下划线开头的键是注释，解析器忽略。",

  "_s1": "==================== 开机行为 ====================",
  "boot_acquire": 0,
  "_doc_boot_acquire": "开机是否自动开始采集：1=自动采，0=需按键/上位机手动启动",
  "_options_boot_acquire": [0, 1],

  "sink": "SD",
  "_doc_sink": "数据出口：SD=存卡(无人值守) / USB=传上位机。开机即采推荐 SD",
  "_options_sink": ["SD", "USB"],

  "duration_ms": 0,
  "_doc_duration_ms": "单次采集时长(ms)，到点自动停；0=一直采到关机或手动停",

  "_s2": "==================== LSM6DSOX 六轴IMU (SPI1) ====================",
  "lsm6dsox": {
    "enabled": 1, "_doc_enabled": "1=参与采集",
    "range_g": 4,    "_options_range_g": [2,4,8,16],
    "range_dps": 2000, "_options_range_dps": [250,500,1000,2000],
    "odr_hz": 1666,  "_options_odr_hz": [12,26,52,104,208,416,833,1666,3332,6664]
  },

  "_s3": "==================== H3LIS100DL 高量程 (SPI2) ====================",
  "h3lis100dl": {
    "enabled": 1,
    "range_g": 100, "_options_range_g": [100],
    "odr_hz": 400,  "_options_odr_hz": [50,100,400]
  },

  "_s4": "==================== QMA6100P 加速度 (SPI2) ====================",
  "qma6100p": {
    "enabled": 1,
    "range_g": 4,  "_options_range_g": [2,4,8,16,32],
    "odr_hz": 100, "_options_odr_hz": [100,200,400,800,1600]
  },

  "_s5": "==================== ES8311 麦克风 (I2C2/SAI) ====================",
  "es8311": {
    "enabled": 1,
    "sample_rate_hz": 16000, "_options_sample_rate_hz": [8000,16000,48000,96000],
    "bits": 16,
    "gain_db": 33, "_doc_gain_db": "0..42 dB"
  },

  "_s6": "==================== AHT20 温湿度 (I2C1) ====================",
  "aht20": { "enabled": 1, "odr_hz": 1, "_doc_odr_hz": "固定 1Hz" },

  "_s7": "==================== LIS2MDL 磁力计 (I2C1) ====================",
  "lis2mdl": { "enabled": 1, "odr_hz": 100, "_options_odr_hz": [10,20,50,100] }
}
```

> 解析器 `json_find_value` 只在 `"` 后做精确键名匹配，`_` 前缀键不会被误匹配，
> 分区注释/`_doc`/`_options` 全部安全忽略。已验证。

## 代码改动任务

### T1. AcqConfig 增加 boot_acquire 字段
- `Core/Inc/acq_config.h`：`AcqConfig_t` 加 `uint8_t boot_acquire;`
- `Core/Src/acq_config.c`：`AcqConfig_Init` 默认 `boot_acquire=0`；
  `AcqConfig_Set` 拷贝该字段（无需额外校验）。
- （可选清理）struct 里 storage_mode/trigger_* 字段可保留不动（无害），仅不在 JSON 暴露。

### T2. 解析器整治（device_config.c `DeviceCfg_ParseAndApply`）
- 删除：顶层 `sample_rate_hz`、`storage_mode`、`trigger_mode`、`trigger_delay_ms`、
  `sd_ring_max_bytes` 的解析块。
- 保留：`sink`、`duration_ms` 解析。
- 新增：`boot_acquire` 解析（json_parse_uint）。

### T3. 模板与回写整治（device_config.c）
- `kCfgTemplate[]`：替换为上面的新结构（分区 + _doc + _options）。
- 回写/快照函数（`DeviceCfg_*` 里 sprintf 整份 JSON 的那两处，约 503–618 / 664–711）：
  同步成新结构，**不再 emit 已删字段**。
- 注意 microLIB：sprintf `%lu` 可用（输出侧没问题，坑只在 sscanf 输入侧）。

### T4. 开机即采接线（核心，app_freertos.c）
- 不能在 `MX_FREERTOS_Init`（line 401 加载配置处）直接调 `AppAcqStart`——此时
  RTOS mutex/任务未就绪、SD 未必挂载。
- 方案：`DeviceCfg_LoadFromSD()` 后把 `boot_acquire` 意图存为模块标志
  `s_boot_acq_pending`（连同 sink/duration）。
- 在**采集/日志任务首次运行**（SD 已挂载就绪）时检查该标志，满足则调用一次
  `AppAcqStart(sink, duration_ms)`，随后清标志。
- **门控**：
  - 仅在正常采集固件模式执行；`g_boot_mode == BOOT_MODE_USB_MSC` 时**绝不**自动采集
    （那是 U 盘模式）。
  - sink=SD 时若无 SD 卡，`AppAcqStart` 优雅 no-op（已有卡状态判断）。
  - sink 仅 SD/USB（`AppAcqStart` 不接受 BOTH）；BOTH 本轮不支持，文档注明。

### T5. 字段映射
- 配置 `sink`(字符串) → `sink_mask`(已有) → 开机即采时转成 `AppAcqStart` 的
  `APP_ACQ_SINK_SD/USB`。

## 风险 & 注意
- **开机即采时序**是唯一有风险的点：必须等 SD 就绪后再启动，且避开 MSC 模式。靠"任务内
  首次触发 + 门控"规避，不在早期 init 里硬启。
- 删字段后旧 SD 卡上的老 DEVCFG.JSN 仍含旧字段——解析器对未知键本就忽略，兼容；新写回
  时会变成新结构。
- 不碰 SD 写盘 DMA/损坏相关逻辑，零回归风险面。

## 验证
1. `boot_acquire=0`：开机不采，行为同现在（回归）。
2. `boot_acquire=1, sink=SD, duration_ms=0`：上电（电池）即自动建 session 写卡，长按停。
3. `boot_acquire=1, sink=SD, duration_ms=10000`：上电采 10s 自动停（看 `[Acq] auto-stop` 打印）。
4. MSC 模式开机：不触发采集，U 盘正常。
5. 改 `sink=USB` + 上位机：开机即流。
6. 删字段后 `status` / 回写的 DEVCFG.JSN 结构正确、无残留死字段。
7. 改 per-sensor odr/range：仍生效（回归）。
```
