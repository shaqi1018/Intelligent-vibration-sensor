# 上位机适配说明 — 新增麦克风 USB 上传（WCID 流，第 4 个数据端点）

> 固件新增第 4 个数据端点把麦克风 96kHz/16-bit 原始 PCM 实时流到上位机。
> 这改动了 WCID 流的端点布局（host-visible），上位机**必须同步修改**，否则命令通道和数据解析会失效。

## 一、端点布局（新 vs 旧）

| 端点地址 | 旧（3 数据通道） | **新（4 数据通道）** | 上位机要改？ |
|---|---|---|---|
| `0x81` IN | LSM6DSOX | LSM6DSOX | 否（完全不变） |
| `0x82` IN | H3LIS100DL | H3LIS100DL | 否（完全不变） |
| `0x83` IN | QMA6100P（带 tag 字节） | QMA6100P（**改为纯净，无 tag**）⚠️ | 是：去掉剥 tag 的逻辑 |
| `0x84` IN | 命令响应 | **MIC 原始 PCM**（新增）⚠️ | 是：新开此端点收音频 |
| `0x85` IN | — | **命令响应**（从 0x84 移来）⚠️ | 是：改从这里读响应 |
| `0x01` OUT | 命令下发 | 命令下发 | 否（不变） |

**上位机要改的三处：**
1. **命令响应**改从 **`0x85`** 读（原 0x84）。不改 → 收不到命令回复。
2. **新增 `0x84`** 读麦克风 raw PCM。
3. **QMA（`0x83` 不变）去掉 tag 处理**：旧版 QMA 是"最后通道"会被插一个 tag 字节（值 0x02），上位机此前需剥掉每半缓冲首字节；新版 QMA 不再是最后通道，**已变纯净，不要再剥**，否则会吃掉 1 字节 CSV。

> LSM(0x81)/H3(0x82) **完全不变**。这是"顺着往后排 + 关掉多余 tag"的方案：每个数据通道独占一个端点（4 通道=4 端点），tag 字节本就多余，固件已将其在 1:1 情况下关闭，所以**现在所有数据通道都是纯净流**。

## 二、麦克风数据格式（端点 0x84）

- **纯 16-bit 小端 PCM，单声道，无任何帧头/tag 字节**。
- 采样率 = 当前配置（默认 **96000 Hz**，可由 `s mic sr` 改，见四）。
- 传输粒度：每次 bulk 传输 **2048 字节 = 1024 个采样**。连续拼接即可，无需对齐处理。
- 直接落盘为 WAV（用配置采样率写头）即可；内容与 SD 模式的 `MIC.WAV` 一致（同一路 SAI DMA）。

## 三、其它数据通道格式（均纯净）

- **LSM `0x81`**：CSV 文本，无 tag（不变）。
- **H3 `0x82`**：CSV 文本，无 tag（不变）。
- **QMA `0x83`**：CSV 文本，**无 tag**（新版变化点，见上）。

每个通道是各自独立的 bulk 流，按行/按样本解析即可，互不交叠。

## 四、命令通道（不变 + 新增 mic 命令）

- 命令下发仍走 **OUT `0x01`**；响应改从 **IN `0x85`** 读。
- 现有命令不变：`ping / help / status / acq_start / acq_stop / set_time / boot_msc`、
  `s <lsm|h3|qma> <odr|range|en> <val>`。
- **新增麦克风配置命令**（与 `s qma odr 800` 同风格）：

  | 命令 | 作用 | 取值 |
  |---|---|---|
  | `s mic gain <db>` | 麦克风 PGA 增益 | 0–42 |
  | `s mic sr <hz>` | 采样率 | 8000 / 16000 / 48000 / 96000 |
  | `s mic en <0\|1>` | 启用/禁用麦克风 | 0 或 1 |

  回复示例：`OK mic en=1 sr=96000 gain=24`。非法值回 `ERR: sr=8000/16000/48000/96000, gain<=42`。
  `status` 输出新增一行：`mic  en=1  sr=96000Hz  gain=24dB`。
  **生效时机**：mic 配置在**下次 `acq_start`** 时随会话 `Mic_Start` 生效（先 `s mic ...` 再 `acq_start`）。

## 五、带宽提示（满速能否跑满 = 本次要实测的点）

USB 全速（Full-Speed）Bulk 总线可用 ~1 MB/s。满载估算：
- MIC 96k×2B ≈ **192 KB/s**
- LSM CSV @6664 ≈ **400 KB/s**（文本，最大头）
- QMA @1600 ≈ 77 KB/s、H3 @400 ≈ 19 KB/s
- 合计 ≈ **~690 KB/s**（约总线 54%）

理论可行但偏紧。上位机务必**及时排空 0x84**（不要让 mic 端点 NAK 堆积），否则固件侧双缓冲会溢出、音频出现断点。验证时看固件日志 `[Ring] mic drop=` 是否为 0、上位机收到的 PCM 是否连续。若跑不满，回退：mic 降采样到 48k/16k（`s mic sr`），或把 LSM 改二进制传输（另议）。

## 七、未来扩展：加磁力计 + 温度传感器（端点不够 → tag 多路复用）

**现在不实现，仅作架构规划。** 当前（4 数据通道 + 响应 = 5 个 IN 流）正好占满 EP1–EP5，
是 1:1 纯净布局。再加 **MAG + TEMP** 后变成 **6 路数据 + 1 路响应 = 7 个 IN 流**，
但 OTG_FS 只有 **5 个可用数据 IN 端点**（EP1–EP5，EP0 是控制），**差 2 个 → 必须复用**。

复用用本流类**自带的 tag 多路复用**（`N_CHANNELS_MAX > N_IN_ENDPOINTS` 时自动启用，
当前因 1:1 被门控关闭）：多个逻辑通道挤在**最后一个端点**上，每段数据前缀 1 个 tag 字节
（值 = 通道索引）区分。**命令响应也作为其中一个 tag 通道**折叠进去，靠 tag 与数据区分，
无歧义——因此无需独立响应端点，也无需让 acq_stop 不回响应。

规划布局（`N_IN_ENDPOINTS=5`，`N_CHANNELS_MAX=7`）：

| 端点 | 通道 | 说明 |
|---|---|---|
| `0x81` | LSM | 独占·纯净（高带宽，~400KB/s） |
| `0x82` | MIC | 独占·纯净（PCM **必须无 tag**） |
| `0x83` | H3 | 独占·纯净 |
| `0x84` | QMA | 独占·纯净 |
| `0x85` | **MAG + TEMP + 命令响应** | **tag 多路复用**（低速/异步，合计仅几十 KB/s） |

要点：
- **MIC、LSM 必须独占干净端点**（MIC 二进制不能插 tag；LSM 带宽大）。
- 低速的 MAG/TEMP + 响应共用 `0x85`，每段带 tag 字节。
- 届时上位机需对 `0x85` 做 **tag 解复用**：读首字节判断属于 MAG / TEMP / 响应，再剥掉首字节取负载。

届时固件改动（预估）：`N_IN_ENDPOINTS=5`、`N_CHANNELS_MAX=7`、新增 MAG/TEMP 通道、
把响应路由改成共享端点的 tag 通道（取代独立的 `DATA_IN_EP_RESP`），FIFO 再分配。
本文件届时会同步更新端点表。

## 六、固件侧改动文件（参考）
- `Middlewares/.../SensorStreaming_WCID/Inc/usbd_wcid_streaming.h`：通道/端点宏（N=4，EP4=0x84 MIC，RESP=0x85）。
- `Middlewares/.../SensorStreaming_WCID/Src/usbd_wcid_streaming.c`：tag 插入门控为 `N_CHANNELS_MAX > N_IN_ENDPOINTS`（1:1 时不插 tag → 全通道纯净）。
- `Core/Src/usbd_conf.c`：FIFO 重分配（新增 EP4 mic 64 words，RX 压到 32）。
- `Core/Inc/usbd_wcid_app.h` / `Core/Src/usbd_wcid_app.c`：MIC 通道（ch3，纯净，半缓冲 2048）。
- `Core/Src/app_freertos.c`：`StartMicTask` USB 模式排空 `g_ring_mic`→EP4；`s mic` 命令；status/help。
