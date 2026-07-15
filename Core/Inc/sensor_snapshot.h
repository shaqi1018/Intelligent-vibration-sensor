#ifndef __SENSOR_SNAPSHOT_H__
#define __SENSOR_SNAPSHOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lsm6dsox.h"
#include "h3lis100dl.h"
#include "qma6100p.h"

#define APP_SENSOR_SAMPLE_PERIOD_MS     1U
#define APP_SENSOR_STALE_TIMEOUT_MS     (APP_SENSOR_SAMPLE_PERIOD_MS * 2U)
#define APP_SENSOR_COHERENT_WINDOW_MS   APP_SENSOR_SAMPLE_PERIOD_MS
/* ★2026-07-12 512→32:g_frame_buffer(每帧104B)现仅承载低速温度(TMP_LOW,~26Hz
 * 由 LSM FIFO 批 push),logger 每轮全部 Pop 走。512深=缓冲~20s严重过配,占 512×104≈52KB
 * RAM。砍到32(缓冲~1.2s,足够)省~48KB → 匀给 QMA/H3 环扩容抗卡长录写变慢的掉帧。
 * (USB 流式路径才需要大深度,当前 path/sd 分支不走 USB 推流。) */
#define APP_SENSOR_FRAME_BUFFER_DEPTH   32U

#define APP_SENSOR_MASK_LSM6DSOX        (1UL << 0)
#define APP_SENSOR_MASK_H3LIS100DL      (1UL << 1)
#define APP_SENSOR_MASK_QMA6100P        (1UL << 2)
#define APP_SENSOR_MASK_ALL             (APP_SENSOR_MASK_LSM6DSOX | APP_SENSOR_MASK_H3LIS100DL | APP_SENSOR_MASK_QMA6100P)

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  uint32_t last_update_ms;
  LSM6DSOX_AllData_t data;
} AppLsm6dsoxSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  uint32_t last_update_ms;
  H3LIS100DL_Data_t data;
} AppH3lis100dlSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  uint32_t last_update_ms;
  QMA6100P_Data_t data;
} AppQma6100pSnapshot_t;

typedef struct {
  AppLsm6dsoxSnapshot_t lsm6dsox;
  AppH3lis100dlSnapshot_t h3lis100dl;
  AppQma6100pSnapshot_t qma6100p;
} AppSensorSnapshot_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  LSM6DSOX_AllData_t data;
} AppFrameLsm6dsox_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  H3LIS100DL_Data_t data;
} AppFrameH3lis100dl_t;

typedef struct {
  uint8_t valid;
  uint32_t sample_seq;
  QMA6100P_Data_t data;
} AppFrameQma6100p_t;

typedef struct {
  uint32_t frame_id;
  uint32_t tick_ms;
  uint32_t enabled_mask;
  uint32_t present_mask;
  AppFrameLsm6dsox_t lsm6dsox;
  AppFrameH3lis100dl_t h3lis100dl;
  AppFrameQma6100p_t qma6100p;
} AppSensorFrame_t;

/* === Lock-free SPSC ring buffer ============================================
 * Single producer (sensor task) calls RingBuf_Write to append CSV bytes.
 * Single consumer (logger) calls RingBuf_PeekContiguous + RingBuf_Consume to
 * flush via one large f_write per file. wr_idx/rd_idx are volatile single
 * 32-bit writes so no mutex is needed. The buffer is full when
 * (wr_idx + 1) % size == rd_idx (one byte sentinel reserved).
 *
 * The data array is provided externally (statically allocated in .bss).
 */
typedef struct {
  uint8_t *data;
  uint32_t size;
  volatile uint32_t wr_idx;
  volatile uint32_t rd_idx;
  volatile uint32_t dropped;          /* bytes dropped due to full buffer */
  volatile uint32_t high_watermark;   /* peak fill level for diagnostics */
  uint32_t last_write_tick;           /* 上次写该环的 tick(写时间封顶用,借鉴 DATALOG1
                                       * SDM_MAX_WRITE_TIME):低速环攒不满固定块时,超时也 flush,
                                       * 避免 env/mag 落盘延迟过大。0=尚未写过。 */
} AppRingBuffer_t;

/* 6664Hz×60B/row≈400KB/s。攒批写(LoggerDrainRing min_flush)把每次写攒到 ~16KB
 * 以摊薄固定开销，但代价是 ring 峰值更高：128KB(0.32s)在全传感器满载下会被攒批+
 * SD 抖动偶尔撑满(CKBX0297 hwm 满、drop 0.3%)。LSM 任务为 High 优先级(抢占 logger
 * 排空硬件 FIFO，几乎消除 FIFO 层丢失)，代价是 logger 吞吐略降、ring 峰值更高:
 * 192KB 仍被填满(CKBX0299 hwm 满、drop 0.8%)。256KB(0.64s)给抢占后的 drain 延迟
 * 足够余量，配合攒批 + LSM High 实现 ~100% 真实捕获且 ring 不溢出。 */
/* LSM 拆成 ACC/GYR 两个独立文件 → 两个独立文本环。原 256KB(合并行)按行长比例
 * 重切:acc 行含 datetime(~43B)、gyr 行仅 frame_id(~29B),总量维持 256KB 不增 RAM。
 * ★2026-07-13 GYR 曾试 96→160KB 对称扩容想救 LSM 掉帧,BIN 长录(CTBX-23-45)审计【证伪】:
 * 两环都扩到 160KB(ring drop=0/hwm 贴顶)后 fid 缺口仍 10.3%,没降。原因:LSM 的 gap 主体是
 * 800ms~2517ms 的秒级写阻塞(周期性),160KB=1.0s 缓冲比 96KB=0.6s 只多 0.4s,填不平 2s 级 gap。
 * LSM 掉帧是【SD 写周期性秒级阻塞】的物理问题,非缓冲不足,RAM 可行范围(+64KB=+0.4s)都无效。
 * 已回退 96KB。真正出路:打点定位那个 2.5s 阻塞源 / 降数据量 / 换高速卡。见记忆
 * project_fullload_dropframe_options / project_dropframe_bin_verdict。 */
/* ★2026-07-15 160→96KB:ACC 与 GYR 同产出(各 93KB/s@6664Hz),原 160KB(1.72s)对 GYR 96KB
 * (1.03s)不对称且无依据。压到 96KB=与 GYR 一致 1.03s,且 ≥86KB 仍能攒满 64KB 块。省 64KB
 * 匀给块池(32→64KB×2)。⚠️缩环有"掉帧升"前科(见下),需满配长录验证 ACC 未变差。 */
#define APP_RING_LSM_ACC_SIZE   (96U * 1024U)
#define APP_RING_LSM_GYR_SIZE   (96U * 1024U)
/* ★2026-07-12 QMA 32→64KB、H3 16→32KB:CSV长录实测(CTBX_22-28)卡写延迟后期从
 * <60ms升到>100ms,小环缓冲太薄扛不过慢写周期→QMA(原0.55s缓冲)掉63%、H3掉大半;LSM
 * (160KB/1.5s+)零丢。翻倍缓冲(QMA→1.1s/H3翻倍)扛过卡长录变慢。空间来自
 * APP_SENSOR_FRAME_BUFFER_DEPTH 512→32省的~48KB(见 sensor_snapshot.h 那条注释)。 */
/* ★2026-07-15 QMA 64→32KB(22KB/s→1.45s):产出低、原2.9s过配,压到1.45s仍富余匀RAM。实测15-25
 * 会话QMA 6.11%在历史区间内,未因缩环变差,保持32KB。
 * ★H3 32→16→24KB:16KB(2.9s)实测掉帧4.99%偏区间高端+暴露太紧,回调到24KB(4.4s)留余量。 */
#define APP_RING_QMA_ACC_SIZE   (32U * 1024U)
#define APP_RING_H3_ACC_SIZE    (24U * 1024U)
/* 麦克风 PCM 环。48kHz×2B=96KB/s(96kHz 配置则 192KB/s)。满配长录时 logger 在分段
 * 滚动(关旧段/开新段)/ card PROGRAMMING 卡顿期间来不及排空 → 溢出丢音频(实测 20-53
 * 满配 52min 丢 12.6%,约 6.6min)。丢帧是突发性的(SD 平均带宽够),故扩大缓冲有效:
 * 64KB(48k≈0.66s) → 192KB(48k≈2s/96k≈1s),吸收卡顿。RAM:U575 768KB,+128KB 后仍余
 * ~80KB。若仍丢,再考虑 logger 优先排空 mic 或降 SD 负载。 */
#define APP_RING_MIC_SIZE       (192U * 1024U)
#define APP_RING_AHT_ENV_SIZE   (4U * 1024U)    /* 温湿度 1Hz，~32B/行 → 4KB≈2min */
/* ★2026-07-15 16→8→16KB:8KB(3.6s)实测掉帧升到8.97%(远超历史~2-5%),明显缩过头——慢通道扛不过
 * 慢写周期,3.6s缓冲不够吸收满配下的写阻塞尖峰。回调回16KB(7.3s),恢复历史水平。 */
#define APP_RING_MAG_SIZE       (16U * 1024U)
/* Largest contiguous chunk handed to f_write per call. Bigger = fewer FAT
 * cluster traversals AND fewer card-PROGRAMMING waits — the real full-load
 * bottleneck (掉帧根因 = 每次 SD 写死等卡 PROGRAMMING 串行堆积,见
 * docs/superpowers/specs/2026-07-08-idma-corruption-rootcause-research.md §7).
 * 16→64KB:多块命令 32→128 扇区/条,PROGRAMMING 等待次数 ~÷4,给串行 drain 更多
 * 吞吐余量吸收突发 GC 停顿。s_sd_bounce 随之 16→64KB(+48KB;RW 用量 689/768KB,余
 * ~79KB,加后余 ~31KB,够)。注意:FatFs 会在簇边界裁剪单次 disk_write(ff.c:3673),
 * 若卡簇 <64KB 则实际按簇分次,收益打折但仍减写次数。纯轮询,零损坏风险。
 * FLUSH_CHUNK 现仅作 s_sd_bounce 容量上限;实际每次写量由 APP_SD_WRITE_BLOCK 定。 */
#define APP_RING_FLUSH_CHUNK    (64U * 1024U)

/* ★2026-07-09 借鉴 ST 官方 DATALOG1(sdcard_manager.c:703 每次 f_write 恒为 1024 整数倍):
 * 每次 SD 写用【扇区对齐的固定大块】,而非"攒到 gate 写任意字节数"。
 * 原因:_FS_TINY=0 时每个 FIL 有 512B 扇区窗口,写非扇区对齐字节数→尾部残余触发
 * read-modify-write(回读扇区→改→写回)+ IDMA 在 512 边界复写跨界 2 字节(tools/bin_dupcheck.py
 * 实测坏点 100% 落 512 边界)。→ 拉低吞吐(实测 648KB/s 反常低)+ BIN 帧错位。
 * 固定写整数个扇区 → 零 FIL 窗口 RMW,f_write 直接走 disk_write 多块路径。
 * WRITE_BLOCK=32KB=64 扇区=常见簇大小,兼顾大块摊薄 PROGRAMMING 与延迟余量。 */
#define APP_SD_SECTOR           512U
/* ★2026-07-15 32→64KB:好卡+64KB簇后实测每次写恒~123ms(与写大小无关,512B/32KB同),说明
 * 123ms是传输建立/总线固定开销,非卡慢。既然固定开销摊不掉,就一次写满一个64KB簇→同样123ms
 * 搬双倍数据,有效吞吐~翻倍(141→~250KB/s),追近满配产出、减轻环溢出→降掉帧。且64KB整块=正好
 * 一个64KB簇,消除半簇RMW。gate/wake_gate=min(3/4环,WRITE_BLOCK)随此宏自动变64KB,仅≥86KB的环
 * (LSM_ACC/GYR/MIC)能攒满64KB块吃红利;小环(QMA/H3/MAG)仍按3/4环出小块,行为不变。 */
#define APP_SD_WRITE_BLOCK      (64U * 1024U)   /* 每次写固定 64KB(128 扇区,512 对齐,=64KB簇) */
/* ★2026-07-14/15 双缓冲块池:N 个 APP_SD_WRITE_BLOCK(现 64KB)连续对齐块,乒乓——写线程写 A 块
 * 时 drain 填 B 块。N=2 × 64KB = 128KB(现值)。RAM 来自环重新平衡(慢通道过配的缓冲匀出 120KB,
 * 净 -56KB,不撞 768KB 单区上限)。N 不宜再大:重叠收益实测~0.05%,多一块不值得占 RAM。 */
#define APP_SD_BLOCK_POOL_N     2U
/* WriteQ 深度:除 N 个 DATA 块外,还要容纳 SYNC/WAVCKPT/STOP 控制消息,给足余量避免 drain 侧阻塞。 */
#define APP_SD_WRITEQ_LEN       (APP_SD_BLOCK_POOL_N + 4U)
/* 低速环(env/mag)攒不满 WRITE_BLOCK 会很久不落盘,借鉴 DATALOG1 SDM_MAX_WRITE_TIME=2s:
 * 超过此 tick 数即使不满块也 flush(尾块非对齐可接受,CSV 靠 \n 分隔无害)。tick 慢~19%,
 * 3000 tick ≈ 实际 3.6s,足够低速环及时落盘又不频繁小写。 */
#define APP_SD_WRITE_MAX_AGE_TICKS  3000U
/* ★2026-07-15 温度(TMP_LOW)记录限流间隔。逐帧路径原跟 6664Hz LSM 链每帧写一行温度→FatFs 每
 * 512B 刷一个扇区→cnt=1 单扇区碎写占 52% SD写预算。温度恒变极慢,~1Hz 足够。tick 慢~19%,
 * 1000 tick ≈ 实际 1.2s,温度分辨率绰绰有余。 */
#define APP_TEMP_LOG_INTERVAL_TICKS  1000U

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_SNAPSHOT_H__ */
