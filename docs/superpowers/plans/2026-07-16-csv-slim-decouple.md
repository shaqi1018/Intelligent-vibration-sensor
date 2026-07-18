# CSV 瘦身降掉帧(去 datetime + 行原子 guard)—— 与 BIN 完全解耦

## 目标与验收
满配长录下,CSV 掉帧从 ~15% 降到个位数/近零,且消除字节截断畸形行。
**硬约束:BIN 分支一个字节都不改**(每通道 `if(BIN){...} else{CSV}` 已天然解耦,只动 else)。

验收(采一组满配 CSV,用 tools/csv_audit_robust.py 复核):
- ACC_LOW/MID/HIGH 真丢帧率显著下降(目标个位数)
- 各通道字段畸形率 → 近 0(行原子 guard 生效)
- CONFIG.JSN 含时间锚 + 时长,PC 端可 `t = base_us + fid×interval_us` 还原

## 背景事实(已核实)
- CSV 掉帧根因 = 产出(~765KB/s)> 落盘上限(~648KB/s)。降产出是唯一不碰 BIN 的解法。
- 去每行 datetime(12B+1 逗号=13B):ACC_LOW 行 43→30B(-30%)、ACC_MID -32%、ACC_HIGH -32%、MAG -40%、ENV -40%。总产出降 ~15-30%。
- 畸形行根因 = 环溢出按字节截断。ACC_LOW/GYR_LOW 已有 A2 guard(0.048%),无 guard 的 MID/HIGH/MAG/ENV 达 0.12-0.23%。
- 时间锚现状:只有目录名(分钟级)。`s_session_start_us`(app_freertos.c:186,2002)只用于诊断,未落盘。CONFIG.JSN(device_config.c:779)无时间字段。

## 改动清单

### A. CONFIG.JSN 增加时间锚 + 时长(device_config.c)
1. `DeviceCfg_WriteConfigToDir`(:779) 新增 JSON 字段(会话启动时写,时长先占位 0):
   ```
   "timebase": {
     "start_epoch_us": <s_session_start_us>,   // 秒级以上精度的起始锚
     "duration_s": 0,                            // 会话结束补写
     "interval_us": { "accel_low": <..>, "accel_mid": <..>, "accel_high": <..>,
                      "mag": <..>, "env": <..> } // 各通道采样间隔
   }
   ```
   - 需要把 `s_session_start_us` 和各通道 interval 传进来 → 改函数签名或加一个 `DeviceCfg_SetTimebase(start_us, intervals...)` setter,由 logger 会话启动时(app_freertos.c:2963 前)调用。
   - interval 来源:LSM `s_lsm_odr_interval_us`、QMA 625us、H3 2500us、MAG 10000us、ENV 1000000us(从 cfg.odr_hz 算)。
2. 会话结束补写 duration:在 `AppLoggerStopSdSession`(:834)完成后,算 `(AppTime_GetEpochUs()-s_session_start_us)/1e6`,重写 CONFIG.JSN 的 duration_s。
   - 最简做法:关闭时整体重写一次 CONFIG.JSN(已有 WriteCurrentToSD 走写线程路径,line 1558)。给 timebase 加一个模块内 static 缓存(start_us + duration),WriteConfig 时带出。

### B. 去掉高频通道 CSV 行的 datetime(app_freertos.c,仅 else 分支)
逐通道删掉 `memcpy(dt..,12); ',';` 两步,行变 `frame_id,x,y,z`:
- ACC_LOW else(:2153-2154)删 datetime。**表头也要同步改**(见 D)。
- ACC_MID else(:2595 段)删 dt_batch_qma。
- ACC_HIGH else(:2391 段)删 AppFmtDateTime12。
- MAG else(:2790 段)删 dt。
- ENV(:2678 段)删 dt。**已定:ENV 也去 datetime**(全通道统一靠 timebase 还原)。
- GYR_LOW 已无 datetime,不动。
- 相应删掉不再使用的 dt_batch/dt_batch_qma/epoch_s_i 计算(注意 BIN 分支或诊断是否还用到 epoch_s,若共用则保留)。

### C. 给无 guard 通道补"行原子"保护(app_freertos.c,仅 else 分支)
MID(:2595)/HIGH(:2391)/MAG(:2790)/ENV(:2678)入环前加:
```c
if (RingBuf_FreeSpace(&ring) >= off) {
    RingBuf_Write(&ring, rowbuf, off);
}   // 否则整行丢弃,绝不写半行 → 消除字节截断畸形
```
(ACC_LOW/GYR_LOW 已有,不动。)

### D. CSV 表头同步(去掉 datetime 列)
表头是集中数组 `kLogHeaders[]`(fatfs_sd.c:68-76,续段会重写表头故滚段第二段也一致)。
索引顺序:0=ACC_LOW,1=TMP_LOW,2=ACC_HIGH,3=ACC_MID,4=ENV,5=MAG,6=GYR_LOW。
改动(去 datetime 的通道):
- idx0 ACC_LOW: `frame_id,acc_x_mg,acc_y_mg,acc_z_mg\r\n`
- idx2 ACC_HIGH: `frame_id,acc_x_mg,acc_y_mg,acc_z_mg\r\n`
- idx3 ACC_MID: `frame_id,acc_x_mg,acc_y_mg,acc_z_mg\r\n`
- idx4 ENV: `frame_id,temp_C,humidity_pct\r\n`
- idx5 MAG: `frame_id,mag_x_mG,mag_y_mG,mag_z_mG\r\n`
- idx1 TMP_LOW 不动(本就 frame_id,tick_ms,temp_C);idx6 GYR_LOW 不动(本就无 datetime)。
**表头与数据行列数必须严格一致**,否则 PC 解析错位。D 步与 B 步逐通道对齐。

## 不做 / 冻结
- BIN 所有分支:零改动。
- 下游字节搬运层(RingBuf_Write/CopyToBounce/DrainRing/滚段):零改动。
- 256MB 滚段行内切割:本次不修(边界效应每段~1 行,可忽略;PC 解析器跳过残行即可)。

## 风险
- 删 datetime 时若误删共用变量(epoch_s 被 BIN/诊断复用)→ 编译错。删前 grep 每个变量的全部引用。
- 表头/数据列数不一致 → PC 解析错位。D 步必须和 B 步严格对应。
- CONFIG.JSN 二次写(补 duration)走写线程 or 直接写:需确保不与写线程竞争 FatFs(复用现有 WriteCurrentToSD 的写线程路径最安全)。

## 验证
1. 用户 Keil GUI 编译(命令行 UV4 不可靠)。
2. 满配采一组 CSV,tools/csv_audit_robust.py 复核掉帧率+畸形率。
3. 对比本方案前后(本次 E:\CTBX_2026-07-16-00-46 作为 baseline:ACC_LOW 13% / MID 18% / HIGH 17% / MAG 15%)。
