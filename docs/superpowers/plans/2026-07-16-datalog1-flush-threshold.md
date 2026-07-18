# 效仿 DATALOG1:去短超时兜底 + 按字节率的 flush 阈值(降写命令次数)

## 目标
满配掉帧真因 = 每条 SD 写固定 ~125ms,一秒仅 ~8 条写;低速通道被 3 秒短超时踢去做小写,
占用宝贵写窗口、把高频通道大写挤到后面排队 → 高频环溢出掉帧。
效仿 DATALOG1:**去掉短超时兜底,flush 阈值按各通道字节率缩放**,让低速通道"写得少但不做小写",
把写窗口让给高频通道。目标:满配 CSV 掉帧从 13-20% 压到个位数。

## 验收
- 满配采一组,tools/analyze_sdwr.py 看 cnt=1 占比大幅下降(当前 67.5%)、有效吞吐上升(当前148KB/s)
- tools/csv_audit_robust.py 各通道掉帧率下降
- BIN 不退化:BIN 满配对照掉帧 ≤ 现值(2.34/1.97/0.81/3.08%)

## DATALOG1 机制(已取回源码,docs 存证)
- 每通道 flush 阈值(半缓冲)= 字节率加权分 RAM,÷2,量化到 1024,按 2 秒封顶
- **运行时无时间超时**:只"半满触发写" + "停止时 flush 残余"
- 掉电安全靠"停止 flush + 低电量优雅关机",不是运行中频繁小写/ f_sync

## 与现工程的映射
我们已有"环 + LoggerDrainRing 的 gate 阈值",不需重写成双半缓冲。要改的是 gate/超时哲学:
- 现在 (app_freertos.c LoggerDrainRing ~1459-1491):
  - gate = min(3/4环, APP_SD_WRITE_BLOCK=32KB),取整扇区
  - aged = (now - last_write_tick) >= APP_SD_WRITE_MAX_AGE_TICKS(≈3秒短超时)
  - `if ((aligned < gate) && !aged) return 0;` ← 短超时让低速通道小写
- 改为 DATALOG1 式:
  - **去掉短超时**(aged 逻辑),或改成"极长保底超时"(仅防极慢通道整会话不落盘)
  - gate 保持按环大小(环大小本就按产出率配过),攒够才写

## 改动清单(方案:阈值按字节率缩放,最忠实 DATALOG1)

核心:每个环的 flush 阈值不再是"统一 3/4环 + 3秒短超时",而是 **flush_gate = 字节率 × T_FLUSH**,
取整扇区,夹到 [1 扇区, min(3/4环, WRITE_BLOCK)]。低速通道阈值天然小(自然定期写、写得少),
高频通道阈值大(攒大块)。去掉短超时兜底——阈值本身已内含"每 T_FLUSH 秒写一次"的节奏。

### 1. AppRingBuffer_t 加字段(sensor_snapshot.h)
- 加 `uint32_t flush_gate;`(本环的字节触发阈值,会话启动时按字节率算)
- `last_write_tick` 保留(改为极长保底超时用,防字节率估算偏差导致完全不写)

### 2. 会话启动时给每个环算 flush_gate(app_freertos.c 会话启动 ~3002 附近)
新增 `AppComputeFlushGate(rate_bytes_per_s)`:
```
gate = rate_bytes_per_s × T_FLUSH_S          // T_FLUSH_S ≈ 1 秒(DATALOG1 用 2 秒封顶,我们环较小取1)
gate -= gate % APP_SD_SECTOR                  // 取整扇区
cap = min(3/4 环, APP_SD_WRITE_BLOCK); cap 取整扇区
if (gate > cap) gate = cap
if (gate < APP_SD_SECTOR) gate = APP_SD_SECTOR  // 至少一扇区
```
各通道字节率 = ODR × bytes/记录:
- CSV: ACC~22 / GYR~18 / QMA~20 / H3~18 / MAG~20 / ENV~15 B/行
- BIN: 14 / 14 / 14 / 14 / 22 / 24 B/帧
- ODR 从 cfg 读(lsm/qma/h3/mag/env),MIC 用 sample_rate×2
- bytes/记录按 output_format 选 CSV 行长 or BIN 帧长 → **BIN/CSV 各自算各自的阈值,自然解耦**
会话启动(WriteConfigToDir 附近)对 7 个环各调一次 AppComputeFlushGate,存入 rb->flush_gate。

### 3. LoggerDrainRing 用 rb->flush_gate 替代原 gate + 去短超时(app_freertos.c ~1473-1490)
- 删除原 `gate = 3/4环 封顶32KB` 计算,改用 `gate = rb->flush_gate`
- 短超时 `APP_SD_WRITE_MAX_AGE_TICKS`(3s)→ 极长保底 `APP_SD_WRITE_MAX_AGE_TICKS_LONG`(~30s):
  仅防字节率估算偏差/极慢通道完全不落盘的兜底,正常靠 flush_gate 定期触发
- `if ((aligned < gate) && !aged) return 0;` 逻辑不变,只是 gate 来源变了

### 4. BIN/CSV 解耦(天然实现)
flush_gate 在会话启动按 output_format 选 bytes/记录来算 → BIN 走 BIN 阈值、CSV 走 CSV 阈值,
两者独立。BIN 不退化论证:BIN 产出低、帧密,算出的 gate 让高频环攒到合理块;去短超时只减小写。
**必须实测 BIN 对照掉帧 ≤ 现值(2.34/1.97/0.81/3.08%)才通过**。

### 5. f_sync 频率(本方案不动,下一步)
DATALOG1 全程不 sync,我们每 1MB sync。本方案先只改 flush 阈值,f_sync 独立下一步验证。

## 不做/冻结
- 不重写环为双半缓冲(现环结构等价,无需大改)
- 不动数据格式(BIN/CSV 行格式不变)
- 不动 f_sync(下一步)
- 不动 RAM 分配(超时改完 T_worst_stall 下降后再评估是否需要)

## 风险
- 极慢通道 30 秒不落盘,硬掉电丢最多 30 秒该通道数据(ENV/MAG 量极小,可接受)
- BIN 可能受影响(理论不会,需实测兜底)
- last_write_tick 语义变化:确认没有别处依赖它做诊断

## 验证
1. Keil GUI 编译(命令行不可靠)
2. 满配采 CSV:analyze_sdwr.py 看 cnt=1 降幅 + csv_audit_robust.py 看掉帧
3. 满配采 BIN 对照:bin_audit_multi.py 确认 BIN 未退化
4. 对比 baseline:CSV 13-20% / BIN 2-3%
