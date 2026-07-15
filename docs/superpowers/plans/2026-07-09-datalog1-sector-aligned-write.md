# 借鉴 DATALOG1：扇区对齐固定块写 + 缓冲吸收，降满配掉帧

日期：2026-07-09
参考：ST 官方 DATALOG1 V1.5.1（STM32L4R9 SensorTile.box，`sdcard_manager.c`）

## 一、调研结论：我们和 DATALOG1 的真实差距

深挖 DATALOG1 后确认，我们的架构骨架**已经和它一致**：
- 采集独立任务（High/AboveNormal）+ logger 独立任务，SPSC ring 解耦 ✓
- 采集优先级 > logger（卡忙时采集继续）✓
- IDMA 写 + 信号量等待（不 CPU 忙等）✓
- `_FS_TINY=0` ✓
- 环形缓冲（等卡时生产者继续填，等价乒乓双缓冲）✓

**唯一实质差距（DATALOG1 有、我们没有）**：
> DATALOG1 每次 `f_write` 的字节数**恒等于半缓冲大小，且严格是 1024 的整数倍**
> （`sdcard_manager.c:703` 向上取整到 SDM_MIN_BUFFER_SIZE=1024 的倍数）。

我们的 `RingBuf_CopyToBounce` 写 `min(avail, 64KB)` = **22 帧倍数（非 512 倍）的任意字节数**。

## 二、为什么这是病根（两个已实测的坏现象都指向它）

`_FS_TINY=0` 时每个 FIL 有 512B 扇区窗口。写**非扇区对齐**字节数时：
1. FatFs 把尾部不满一扇区的数据存进 FIL 窗口，触发 **read-modify-write**（回读扇区→改→写回）→ 额外扇区读 + 写放大 → 拉低吞吐（对上实测 648KB/s 反常低）。
2. IDMA 多块传输在 512 扇区边界重复写跨界 2 字节（`tools/bin_dupcheck.py` 实测：1208/1208 坏点在 512 边界、插入字节=边界前 2 字节复制）。

DATALOG1 的 1024 对齐固定块写**从源头避免**了这两者：每次写正好整数个扇区，f_write 直接走 disk_write 多块路径，零 FIL 窗口 RMW。

## 三、方案：改消费者写块策略为「扇区对齐固定大块」

**不推翻** SPSC ring / logger 任务 / IDMA / 低电量保护 / 分段（这些已验证，且等价于 DATALOG1 的任务解耦）。
**只改** `LoggerDrainRing` + `RingBuf_CopyToBounce` 的写块决策，套用 DATALOG1 的固定块思想。

### 核心改动

1. **定义扇区对齐写单位**（`sensor_snapshot.h`）：
   ```c
   #define APP_SD_SECTOR          512U
   #define APP_SD_WRITE_BLOCK     (32U * 1024U)   /* 每次写固定 32KB = 64 扇区，整簇友好 */
   ```
   （FLUSH_CHUNK 64KB 保留为 bounce 上限；写块设 32KB = 卡簇大小，兼顾大块摊薄与延迟。）

2. **`LoggerDrainRing` 门控改为「够一个对齐块才写」**：
   - 当 `avail >= APP_SD_WRITE_BLOCK`：写**恰好 APP_SD_WRITE_BLOCK** 字节（512 对齐）。
   - 否则不写（继续攒），除非 `min_flush==0`（会话停止的尾部 flush）。
   - 尾部 flush（stop）时写剩余全部 avail（可能非对齐，但这是最后一次写，且只影响文件末尾——CSV 靠 \n 分隔无害；BIN 末尾用重同步解析器可恢复，或单独接受末块非对齐）。

3. **`RingBuf_CopyToBounce` 支持"取整到 512"**：
   - 正常写：`want = APP_SD_WRITE_BLOCK`（已 512 对齐）。
   - 尾部 flush：`want = avail`（不取整，最后一块）。
   - 移除上一轮错误的 22 帧对齐（`frame_size` 逻辑），改为 512 对齐。

4. **移除/保留 frame_size**：上一轮加的 `frame_size` 字段和帧对齐逻辑**移除**（诊断错误，扇区对齐才对）。`AppRingBuffer_t` 恢复原状或复用字段存对齐粒度。

### 关键正确性论证

- **avail 恒是 512 对齐块的整数倍吗？** 不需要。avail 是任意值（帧倍数），但我们只在 `avail >= 32KB` 时写**固定 32KB**（512 对齐），剩余留环里。环不断累积，每够 32KB 写一次。写字节数恒为 32KB，永远扇区对齐。
- **CSV 会不会被切坏？** CSV 行靠 `\r\n` 分隔，扇区对齐写在行中间切开无害（下游按行解析，行跨块无影响，这是文本流特性）。**这正是 CSV 比 BIN 健壮的地方**——所以用户选 CSV 是对的。
- **BIN 会不会跨块？** 会，但 BIN 帧有 CRC + 重同步解析器（`tools/verify_bin_crc.py` 已证数据完整）。若要 BIN 也整帧对齐，需 32KB 是帧大小倍数——32768/22 非整除。**故 BIN 仍需重同步解析器；CSV 无此问题**。当前用户用 CSV，此点不阻塞。
- **尾部延迟**：32KB @ 满配单通道产出，最坏几百 ms 攒一块。低速通道（env 1Hz）靠 `min_flush==0` 的会话停止 flush 保证不丢；运行中低速通道可能攒很久——需加**时间封顶**（见增强项）。

### 增强项（DATALOG1 有，按需加）

5. **写块时间封顶**（DATALOG1 `SDM_MAX_WRITE_TIME=2`）：低速 ring（env/mag）攒不满 32KB 会很久不落盘。加：每 ring 记录上次写时间，超过 N 秒即使不满块也 flush（尾部非对齐，可接受）。**否则 env/mag 数据延迟极大**。这个必须加。

6. **按 ODR 加权分配 ring 大小**（DATALOG1 `SDM_CalculateSdWriteBufferSize`）：可选优化，我们现在手工固定大小基本够用，暂不动。

## 四、实施步骤

1. `sensor_snapshot.h`：加 `APP_SD_SECTOR` / `APP_SD_WRITE_BLOCK` 宏；移除上轮 `frame_size` 相关（或改注释）。
2. `app_freertos.c`：
   - `AppRingBuffer_t` 移除 frame_size 字段（回退上轮改动）。
   - `RingBuf_CopyToBounce`：改为按传入的目标字节数拷贝（512 对齐由调用方保证），移除帧取整。
   - `LoggerDrainRing`：门控改「avail>=WRITE_BLOCK 写固定块；min_flush==0 写余量」；加每 ring 写时间封顶。
   - `RingBuf_Init`：移除 frame_size 初始化。
   - 会话启动处：移除上轮按 output_format 设 frame_size 的代码块。
3. 编译（Keil GUI）、满配 CSV + IDMA 长录 ≥30min。

## 五、验证判据

- **掉帧**：mic/qma/h3 drop 对比修改前同配置。预期显著下降（扇区对齐消除 RMW → 吞吐升）。
- **数据完整性（CSV）**：读卡抽查行完整性、行数 vs 期望、frame_id 连续性。
- **吞吐**：`writes` 次数、落盘率（若能测）。预期 648KB/s 上限被突破。
- 若掉帧仍高：再上「增强项 6 按 ODR 分配缓冲」或考虑双 SDMMC。

## 六、不做什么（避免过度重构）

- 不换 SPSC → 消息队列（我们 ring 已解耦，CMSIS-v2 无需照抄 v1 API）。
- 不推翻 logger 任务结构。
- 不动低电量保护 / 分段 / WAV checkpoint / HWFC。
- 不重新引入 f_expand（DATALOG1 也没用，且我们已验证它放大掉电损坏）。
