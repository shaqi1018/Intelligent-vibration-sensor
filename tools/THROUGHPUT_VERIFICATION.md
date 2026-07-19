# 产出速率验证报告

## 任务目标
验证理论计算的产出速率是否准确:
- CSV: 636 KB/s (用户提及)
- BIN: 425 KB/s (用户提及)

## 理论计算结果

基于固件配置和文档分析，我重新计算了理论产出速率:

### 传感器配置 (满配)
- **ACC_LOW (LSM加速度)**: 6664 Hz
- **GYR_LOW (LSM角速度)**: 6664 Hz  
- **ACC_MID (QMA)**: 1600 Hz
- **ACC_HIGH (H3)**: 400 Hz
- **MAG (磁力计)**: 100 Hz
- **ENV (温湿度)**: 1 Hz
- **MIC (麦克风)**: 96000 Hz (或48000 Hz)

### CSV格式分析

#### CSV行格式
从 `docs/superpowers/plans/2026-07-16-csv-slim-decouple.md` 文档:
- **含datetime**: 43 bytes/行 (ACC_LOW实测)
  - frame_id: ~10位数字
  - datetime: 12B (YYMMDDHHMMSS)
  - 逗号×4 + \r\n: 6B
  - x,y,z数据: ~18-21B

- **去datetime**: 30 bytes/行
  - 节省: 13B (30.2%)

#### CSV产出速率计算

**含datetime + 96kHz mic:**
```
LSM加速度:  6664 × 43 = 279.8 KB/s
LSM角速度:  6664 × 43 = 279.8 KB/s
QMA:        1600 × 43 =  67.2 KB/s
H3:          400 × 43 =  16.8 KB/s
MAG:         100 × 43 =   4.2 KB/s
ENV:           1 × 43 =   0.0 KB/s
MIC:       96000 × 2  = 187.5 KB/s
────────────────────────────────
总计:                  835.4 KB/s
```

**去datetime + 96kHz mic:**
```
传感器数据:            452.0 KB/s (30B/行)
MIC:                   187.5 KB/s
────────────────────────────────
总计:                  639.5 KB/s  ← 接近636 KB/s!
```

**去datetime + 48kHz mic:**
```
传感器数据:            452.0 KB/s
MIC:                    93.8 KB/s
────────────────────────────────
总计:                  545.8 KB/s
```

### BIN格式分析

#### BIN帧格式
从 `tools/audit_session.py`:
- ACC_LOW/GYR_LOW/ACC_MID/ACC_HIGH: 14 bytes/帧
- MAG: 22 bytes/帧
- ENV: 24 bytes/帧

格式: frame_id(4B) + data(6-14B) + CRC32(4B)

#### BIN产出速率计算

**BIN + 96kHz mic:**
```
LSM加速度:  6664 × 14 =  91.1 KB/s
LSM角速度:  6664 × 14 =  91.1 KB/s
QMA:        1600 × 14 =  21.9 KB/s
H3:          400 × 14 =   5.5 KB/s
MAG:         100 × 22 =   2.1 KB/s
ENV:           1 × 24 =   0.0 KB/s
MIC:       96000 × 2  = 187.5 KB/s
────────────────────────────────
总计:                  399.2 KB/s  ← 接近425 KB/s!
```

**BIN + 48kHz mic:**
```
传感器数据:            211.7 KB/s
MIC:                    93.8 KB/s
────────────────────────────────
总计:                  305.5 KB/s
```

## 理论值对比

| 格式配置 | 理论计算 | 用户提及 | 差异 |
|---------|---------|---------|------|
| CSV去datetime+96kHz | 639.5 KB/s | 636 KB/s | +3.5 KB/s (+0.6%) |
| BIN+96kHz | 399.2 KB/s | 425 KB/s | -25.8 KB/s (-6.1%) |

### 差异分析

1. **CSV计算接近准确** (639.5 vs 636 KB/s):
   - 差异仅0.6%，可能来自:
     - CSV行长度的轻微变化 (数据位数不固定)
     - \r\n vs \n的差异

2. **BIN计算偏低** (399.2 vs 425 KB/s):
   - 差异-6.1%，可能原因:
     - 用户提及的425 KB/s可能包含48kHz mic? 
       - 但305.5 KB/s (BIN+48k) 仍远低于425
     - 或者BIN帧实际大小比audit_session.py中的定义更大?
     - 需要实测验证

## 与SD卡写入上限对比

从 `docs/superpowers/specs/2026-07-08-dropframe-opportunistic-write.md`:
- **SD卡持续写入上限**: ~648 KB/s (实测)

### 掉帧预测

| 配置 | 理论产出 | vs 648 KB/s | 预测结果 |
|------|---------|------------|----------|
| CSV+datetime+96kHz | 835.4 KB/s | +187.4 KB/s | ⚠️ 严重掉帧 (~29%) |
| CSV+datetime+48kHz | 741.6 KB/s | +93.6 KB/s | ⚠️ 掉帧 (~14%) |
| **CSV-datetime+96kHz** | **639.5 KB/s** | **-8.5 KB/s** | ✅ 理论安全 |
| CSV-datetime+48kHz | 545.8 KB/s | -102.2 KB/s | ✅ 安全余量 16% |
| **BIN+96kHz** | **399.2 KB/s** | **-248.8 KB/s** | ✅ 安全余量 38% |
| BIN+48kHz | 305.5 KB/s | -342.5 KB/s | ✅ 安全余量 53% |

## 需要实测验证的点

1. **CSV实际行长度**:
   - 读取实际CSV文件前100行
   - 计算平均行长
   - 确认是否真的30B (去datetime) 或 43B (含datetime)

2. **BIN实际帧大小**:
   - 读取实际BIN文件
   - 验证各通道帧大小是否为14/22/24字节
   - 可能存在对齐填充?

3. **实际文件大小/时长比值**:
   - 从MIC.WAV获取准确时长 (SAI硬件时钟锚点)
   - 计算所有数据文件总大小 / 时长
   - 得到实测产出速率

4. **MIC采样率确认**:
   - 96kHz vs 48kHz
   - 从WAV头读取确认

## 工具准备

已创建两个工具脚本:

1. **calculate_throughput.py**: 理论计算工具
   - 基于配置参数计算理论产出
   - 对比CSV/BIN/不同mic采样率

2. **verify_throughput.py**: 实测验证工具
   - 分析实际会话目录
   - 读取文件大小和MIC.WAV时长
   - 采样CSV行长/BIN帧大小
   - 计算实测产出速率

## 使用方法

```bash
# 理论计算
python tools/calculate_throughput.py

# 实测验证 (需要提供会话路径)
python tools/verify_throughput.py E:/CTBX_2026-07-17-22-26/
python tools/verify_throughput.py E:/CTBX_2026-07-18-17-58/

# 对比多个会话
python tools/verify_throughput.py \
    /path/to/csv_session/ \
    /path/to/bin_session/
```

## 结论

理论计算显示:
- **CSV (去datetime) + 96kHz mic**: 639.5 KB/s ≈ 636 KB/s ✓
- **BIN + 96kHz mic**: 399.2 KB/s vs 425 KB/s (差异需实测确认)

要完成验证，需要:
1. 提供实际会话目录路径
2. 运行 verify_throughput.py 分析实际数据
3. 对比理论 vs 实测，找出差异根因
