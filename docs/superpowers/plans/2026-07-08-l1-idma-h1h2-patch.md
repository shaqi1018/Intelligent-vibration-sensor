# L1 补丁计划:H1(__DMB) + H2(对齐) 试根治 IDMA 损坏

日期：2026-07-08　对应调研 specs/2026-07-08-idma-corruption-rootcause-research.md §6 L1

## 目标
在**保持默认纯轮询不变**的前提下，加两处零风险防护，使 `SD_SetDmaMode(1U)` 开 IDMA 时
不再字节损坏。用户做 A/B：开 IDMA + 满配96k长录≥30min，数坏行对照历史 CKBX0079 的 0.40%。

## 关键认识（缩小改动面）
**`s_sd_bounce` 是 IDMA 唯一的源。** 所有 SD 写都走 `LoggerDrainRing`：
`memcpy(s_sd_bounce, ring) → f_write(s_sd_bounce) → HAL 启动 IDMA 从 s_sd_bounce 读`。
IDMA **从不直接读 ring buffer**。所以 H1 屏障和 H2 对齐都**只需作用于 bounce**，一处搞定。

## 改动清单（全部在我们自己的文件，不碰 HAL 驱动）

### 改动1 — H1 内存屏障（app_freertos.c，LoggerDrainRing 内）
`memcpy(s_sd_bounce, p, n);` 之后、`FatFs_SD_LoggerWriteFileIndex(...)` 之前插入：
```c
memcpy(s_sd_bounce, p, n);
__DMB();  /* H1: 确保 memcpy 的写缓冲落进 SRAM 后再启动 IDMA(见调研 §8.5)。
           * 轮询模式下 __DMB 无副作用(只保证写完成,不减速)。 */
```
- `__DMB()` 来自 CMSIS core_cm33.h（经 main.h→stm32u5xx_hal.h 已包含，与 frame_ring.c:92 同源）。
- 一处覆盖全部 7 个通道（都经此 memcpy）。

### 改动2 — H2 缓冲对齐（app_freertos.c:1322）
```c
static uint8_t s_sd_bounce[APP_RING_FLUSH_CHUNK];
```
→
```c
/* H2: IDMA 对 SDMMC FIFO 做 32-bit 访问，源缓冲须字长对齐否则丢/错字节(见调研 §8.5)。
 * 给 32 字节对齐(超额保险，兼容任何潜在 burst 对齐要求)。s_sd_bounce 是 IDMA 唯一源。 */
static uint8_t s_sd_bounce[APP_RING_FLUSH_CHUNK] __attribute__((aligned(32)));
```
- 只改 bounce（IDMA 唯一源）。ring buffer 不动（IDMA 不读它们）。
- ARMCC/armclang 都支持 `__attribute__((aligned(N)))`（工程用 microLIB+armclang）。

### 改动3 — A/B 开关（app_freertos.c:2627，可选但推荐）
当前硬编码 `SD_SetDmaMode(0U)`。为方便 A/B，改成一个编译期宏，默认仍 0（轮询）：
```c
#ifndef APP_SD_USE_IDMA
#define APP_SD_USE_IDMA 0U   /* 0=轮询(默认,零损坏已验证);1=IDMA(L1试点,需H1+H2) */
#endif
  SD_SetDmaMode(APP_SD_USE_IDMA);
  printf("[Logger] SDMMC %s mode\r\n", APP_SD_USE_IDMA ? "IDMA(L1 test)" : "POLLING");
```
- 用户 A/B 时只需把宏改 1 重编译，测完改回 0。默认行为**完全不变**。
- 不删原有历史注释（保留根因记录）。

## 为什么零风险（默认路径不变）
- 改动1 `__DMB()`：轮询模式下只是"确保写完成"，不改变任何数据流、不减速。
- 改动2 对齐：只影响 bounce 的地址，功能等价。
- 改动3：宏默认 0，`SD_SetDmaMode(0U)` 行为与现在**完全一致**。
→ 不开 IDMA 时，本补丁对现有已验证的轮询路径**零行为改变**。

## 验证
1. Keil GUI 编译（用户操作；GUI 开着时命令行 UV4 不真跑，见记忆）。
2. **A 组(对照)**：宏=0，满配96k长录，确认仍 0 坏行（回归保护）。
3. **B 组(试点)**：宏=1(开IDMA+H1+H2)，满配96k长录≥30min，读卡审计数坏行/NUL。
   - 判据：坏行=0 → H1+H2 成立，IDMA 可安全用（再单独验吞吐/掉帧，那是③的事）。
   - 坏行>0（对照 0.40%）→ 强烈指向 H3 硅级，转 L2(换bank) 或放弃 IDMA。
4. 用记忆的审计方法论：MIC.WAV 秒数当锚，逐通道 CONFIG.JSN 读标称。

## 不做什么
- 不改 HAL 驱动 `stm32u5xx_hal_sd.c`（CubeMX 重生成会丢；且我们代码侧加屏障已足够）。
- 不动 ring buffer 尺寸/结构（那是③掉帧的事，本补丁只管损坏）。
- 不删 f_expand 回退注释、不动掉电保护/A2 对齐（已 commit 落袋）。

## 提交
改动小、聚焦。编译通过后可 commit（feat/test，标注 L1 试点、默认轮询不变）。push 仍等用户明确同意。
