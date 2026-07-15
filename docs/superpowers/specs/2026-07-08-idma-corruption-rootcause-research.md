# 方案1 深挖:SDMMC IDMA 字节损坏根因再调研（纯调研，不动码）

日期：2026-07-08
作者：调研（结合历史实测记忆 + ST 官方资料 + 当前代码复核）
状态：**调研结论 = 旧"bank 级冒险不可解"结论证据不足；头号新嫌疑 = 启动 IDMA 前缺内存屏障（软件可解）。待硬件 A/B 证实。**

---

## 0. 一句话结论

当年判 IDMA 损坏为"SRAM bank 级总线冒险、不可解、只能纯轮询"，但本次复核发现**关键反证**：
项目在 CPU↔CPU 的 SPSC 环（`frame_ring.c`）特意用 `__DSB()` 保证"数据先于指针可见"，
**却在 CPU→IDMA 这一跳（memcpy 到 bounce 后启动 IDMA）没有任何屏障**。这正是 ST 官方
bug 帖点名的同一类漏洞。若成立，方案1 的性质从"修不了的硅 bug"变成"启动 DMA 前补一条
`__DMB()`"的软件修复。**但这是假设，不是结论**——必须硬件 A/B 证实（判据见 §6）。

同时另一条**独立结论不变且更重要**：即便 IDMA 损坏被根治，**它也治不了满配长录掉帧**
（掉帧是卡 PROGRAMMING-bound，与用 CPU 还是 IDMA 当总线主机无关）。降掉帧要走出路2
（加大 FLUSH_CHUNK + 加环余量），见 §7。两件事要分开做、分开验。

## 1. 待验证的问题（本次调研的四个靶子）

1. IDMA 损坏的真实根因是不是"bank 级总线冒险不可解"？还是软件可解（缺屏障 / 安全属性 / 配置）？
2. IDMA 源地址能否限定到独立 SRAM bank？（"换 bank"方案的物理前提）
3. U575 官方 errata 里有没有 SDMMC IDMA 相关硅 bug 条目？
4. 换 IDMA 到底能不能提吞吐、解掉帧？（这是用户的核心诉求）

## 2. 硬事实核实（当前代码 + 硬件配置，全部本次复核过）

| 事实 | 出处 | 意义 |
|---|---|---|
| TrustZone **关闭** `configENABLE_TRUSTZONE=0`，用 `ARM_CM33_NTZ` 端口，GTZC 未启用 | `FreeRTOSConfig.h:70`、`stm32u5xx_hal_conf.h:52` | **排除** TrustZone/GTZC 安全属性拦 DMA 的假设 |
| SPSC 帧环写完数据、更新 head 前特意 `__DSB()` | `frame_ring.c:92,109` | 项目**已知** M33 需屏障保证"数据先于指针" |
| HAL DMA 写：`CMDTRANS_ENABLE → IDMABASER=pData → IDMACTRL=ENABLE`，**启动前无屏障** | `stm32u5xx_hal_sd.c:1285-1287`（读同结构 1385） | ★ 缺屏障的确切位置 |
| logger drain：`memcpy(s_sd_bounce,...) → f_write → HAL 启动 IDMA` | `app_freertos.c:1350-1351` | memcpy 写与 IDMA 读之间横跨 CPU 写缓冲，零屏障 |
| IDMA = `SINGLE_BUFF`，源地址 `IDMABASER` 可任意指定 | `stm32u5xx_hal_sd.c:1286` | "限定 IDMA 源到独立 bank"技术可行 |
| M33 **无 D-Cache**（只有 ICACHE，`main.c:118`） | `icache.c` | 排除 D-Cache 一致性（那是 M7/H7 的病） |
| SD 时钟 16MHz、4-bit、HWFC_EN；轮询写单次整块、PROGRAMMING 期间 `vTaskDelay(1)` 让出 | `sdmmc.c:61,80`、`sd_diskio.c:255-270` | 当前轮询已不关中断、已让出 CPU |

## 3. 机制分析：为什么"缺屏障"能解释"连私有 bounce 都坏"

历史死结（记忆 [[project_sdmmc_idma_corruption]] CKBX0292）：**私有 bounce（CPU 不并发写）
照样损坏** → 当年据此判"不是地址竞争（bounce 本可解），是 bank/总线矩阵级冲突"。

**但这个推理有漏洞。** 一个功能正常的多层总线矩阵，master A 读 region B 时 master C 写
region D，只会**仲裁变慢**，绝不会**损坏**彼此数据——损坏必有额外原因。候选：

- **A. 缺内存屏障（头号嫌疑，软件可解）**：`memcpy(bounce)` 是普通 store，Cortex-M33 有
  写缓冲（write buffer）+ 允许弱序。memcpy 返回时数据可能**还在写缓冲里没落进 SRAM**。
  紧接着 HAL 无屏障就 `IDMABASER=bounce; IDMACTRL=ENABLE` 启动 IDMA。IDMA 作为独立总线
  主机从 SRAM 读时，**读到的是尚未落盘的旧字节**。CRC 只校验 SD 总线传输、dma_err=0，
  故静默、查不出——与实测症状（逗号被吞/字节错/CRC 正常/dma_err=0）吻合。ST 官方 bug 帖
  「Missing compiler and CPU memory barriers」确认此类漏洞真实存在，解法 = 启动 DMA 前
  `__DMB()`。**bounce 挡住了"地址竞争"，但挡不住"写缓冲未落盘"——所以 bounce 照样坏，
  被误判成 bank 级不可解。**
- **B. 真硅 errata（不可解或需特定 workaround）**：需查 ES0499 确认（§5）。
- **C. 其它 HAL 配置（如 DBLOCKSIZE、IDMABNDT）**：ST 社区有 DBLOCKSIZE 配错致"每 512B
  夹 14-15 坏字节"的先例，但那是 HAL 标准路径不太可能错；仍列为待排除。

**假设 A 的边界（必须诚实标注）**：缺屏障主要污染**传输开头**尚未落盘的字节。当年症状是
"整段散布 ~0.4%"。散布式损坏，单靠开头一条屏障**未必全解**——可能还需在每个 memcpy 后、
或每次 IDMA 复用 bounce 前加屏障，甚至可能不是全部病因。**A 是高价值、低成本（改几行）的
头号试点，不是已证实的完整答案。**

## 4. IDMA 换 bank 的物理可行性

- U575 RAM 768KB = SRAM1(192K)+SRAM2(64K)+SRAM3(512K) 三块物理 bank，地址连续拼接
  （0x2000_0000 起）。当前链接脚本 `SensorProj.sct:12` 把**所有 RW+ZI 塞进单一
  `RW_IRAM1 0x20000000 0x000C0000`**，三 bank 混用、无分区。
- `IDMABASER` 可指任意地址 → 技术上可把 bounce 单独放某 bank、CPU 工作区放另一 bank，
  使 IDMA 读与 CPU 写落在不同 bank，减少总线层争用。
- **但**：若真因是"缺屏障"（假设 A），换 bank **无效**（屏障问题与 bank 无关）；若真因是
  仲裁级争用，换 bank 可能减轻但不保证消除。换 bank = 改链接脚本 + `__attribute__((section))`，
  中等改动、需 A/B，**且优先级应低于先试屏障**（屏障零成本、先证伪 A）。

## 5. 官方 errata（待人工确认，无硬件也能查文档）

- 文档：**ES0499**「STM32U575xx/585xx device errata」。芯片 = STM32U575RIT。
  **2026-07-08 取全文失败**：ST 站反爬 + errata 直链需真实 DM 编号(拼不出)、PDF 直取超时、
  Scribd 镜像 406、documentation 页 JS 动态加载无静态链接。web 搜索只得目录级摘要，
  **未逐条读到 SDMMC/IDMA 条目**。→ **需用户用 ST 登录账号下载 ES0499 PDF** 后我再 grep，
  或用户在 STM32CubeMX/CubeIDE 里查该器件 errata 视图。H3 是最弱假设，不阻塞 L1(H1+H2)。
  行动项(待用户协助)：ES0499 里 grep "SDMMC"/"IDMA"，确认有无"IDMA 与 CPU 并发访问 SRAM
  数据损坏"类硅 bug 及 workaround。
- 若 errata 明列此 bug 且 workaround = 加屏障/限定访问 → 直接印证假设 A/换 bank 方向。
- 若 errata 无此条 → 更支持"是我们缺屏障"而非硅 bug。

## 6. 验证阶梯（按"成本↑、风险↑"排序；每级都有明确证伪判据）

前置：所有 A/B 都用**满配 + 96kHz 长录（≥30min）**，因为短测险胜过、掩盖问题（记忆教训）。
判据数据源用 **MIC.WAV 秒数当锚**（SAI 硬件时钟），逐通道从 CONFIG.JSN 读标称，
**不硬编码全速 ODR、不拿 LSM 当基准**（记忆 [[project_audit_methodology]]）。

- **L0（先做，零风险，只读文档）**：下载 ES0499，确认有无 SDMMC IDMA errata。→ 决定 A vs B。
- **L1（头号试点，改几行，可秒回退）= H1+H2 一次同加**：保持结构不变，
  ① `LoggerDrainRing` memcpy 后、启动 IDMA 前加 `__DMB()`；
  ② `s_sd_bounce` 及各 ring buffer 加 `__attribute__((aligned(4)))`（保险给 32）；
  然后开 `SD_SetDmaMode(1U)` 满配 96k 长录 ≥30min。（H1、H2 都零风险低成本、互不冲突，
  一次同加省一轮烧录；若想区分贡献可后续拆测。）
  - **证伪判据**：坏行/NUL 对照历史 CKBX0079 的 0.40%。=0 → H1/H2 成立，IDMA 可安全用
    （**但仍需 §7 验吞吐**）；仍 >0 → 强烈指向 H3 硅级，转 L2/L3。
- **L2（中风险，改链接脚本）**：bounce/工作区分到不同 SRAM bank（SRAM2/3），再开 IDMA 长录。
  - 证伪判据同 L1。
- **L3（保底，永远可用）**：放弃 IDMA，纯轮询 + 出路2（§7）。**这是掉帧的真正解。**

## 7. ★ 独立且更重要的结论：IDMA 治不了掉帧

无论 IDMA 损坏能否根治，**它都不解决满配长录掉帧**。三条硬理由：

1. 瓶颈是**卡 PROGRAMMING 串行等待**（flash 写时间，卡侧行为），与谁当总线主机无关。
   IDMA 只换"搬 FIFO→SRAM 那 ~2ms"，不碰 PROGRAMMING 等待。
2. **单卡无法流水线**：一张 SD 同时只跑一条命令，IDMA 给不了"边写边算下一块"的重叠。
3. **实测已判死刑**：记忆载"DMA 全传感器下写次数 ~431/s，信号量往返开销反让 ring 全线溢出"；
   DATALOG1（经验证的高速 logger）的 `SD_write` 同样是"发起 DMA→等完成→死等 TRANSFER_OK"，
   **无任何单卡流水线魔法**，靠的是大块 DMA+应用层双缓冲，不是"DMA 重叠掉 PROGRAMMING"。

**降掉帧的真正杠杆（出路2，纯轮询、零损坏风险）**：
- **减少写次数（最高杠杆）**：`APP_RING_FLUSH_CHUNK` 16KB→64KB。多块命令 32 扇区/条→128
  扇区/条，PROGRAMMING 等待次数 ÷4。FatFs 已支持多块合并（`ff.c:3679` 传 cc 扇区），白捡。
  RAM 够（现环总 ~516KB/768KB，bounce 升 64KB 仍宽裕）。
- **加环余量吸收突发 GC 停顿**：GC 停顿突发性，环够深能骑过去。buy 余量，与上一条叠加。
- 预期把 27.9% 压到个位数，**但必须实测才敢下结论**（不重犯拍脑袋报数字）。

## 8. 建议执行顺序

1. **L0**：查 ES0499（无硬件即可做）。
2. 若诉求是"降掉帧" → **直接做 §7 出路2 的 A/B**（不碰 IDMA，风险最低、直击瓶颈）。
3. 若诉求是"根治损坏以便未来能用 DMA"（如做 USB-MSC 高速传输）→ 做 **L1 屏障试点**。
4. L2/L3 视 L0/L1 结果再定。

## 8.5 社区 + ARM 官方深度调研（2026-07-08 补）

沿"根治"方向又扫了一轮 ST 社区 + ARM 官方文档，收敛出**按可能性排序的三层根因**，
外加一条**决定性的同症状案例**。

### ★ 决定性同症状案例：H7 "SD corruption after f_write"（FreeRTOS）

`community.st.com/.../sd-card-corruption-after-f-write-with-stm32h7-sdmmc-fatfs-freertos-165963`
失效签名与我们**逐条吻合**：只在**写**后损坏、`f_write/f_sync/f_close` 全返回成功、
**读正常**、FS 结构完好。楼主定位的机制：
> "invalidate cache → CPU memcpy 填 scratch → 启动 DMA，**但 memcpy 后没有 cache clean**
> → CPU 写的新字节还在 cache 里没落 RAM，DMA 读到 RAM 里的旧内容 → 坏数据落卡。
> 读正常是因为只有写才把 CPU 手里的新数据经 DMA 推出去。"
修法二选一：① scratch 放 non-cacheable MPU 区（`__attribute__((section(".NCACHE"),aligned(32)))`）；
② memcpy 后、启动 DMA 前 `SCB_CleanDCache_by_Addr(scratch, ...)`（楼主首选，更干净）。

**这就是我们的病，只是核不同、"CPU 手里"的载体不同：**
- H7(M7)："CPU 手里" = **D-Cache** → 修法 = `SCB_CleanDCache_by_Addr`。
- U575(M33)：**无 D-Cache**，"CPU 手里" = **写缓冲(write buffer)** → 修法 = **`__DMB()`**。
同一类"CPU 写的新数据尚未对 DMA 这个第二总线主机可见"的病，**按核换药**。

### ARM 官方对 M33 "换药"的背书（及诚实的反证）

- ★ 支持：ARM《Programming Guide to Memory Barrier Instructions》(DAI0321A) 明写
  **"当系统中有 DMA 控制器时，启动 DMA 操作前需要 DMB"**，且"DMB/DSB 确保处理器**写缓冲**
  在后续操作前完成"。→ 直接支持"启动 IDMA 前 `__DMB()`"。项目 `frame_ring.c:92` 已在
  CPU↔CPU 方向用 `__DSB()`，唯独 CPU→IDMA 这一跳漏了。
- ⚠️ 反证（必须承认，不选择性引用）：同文档另一句 **"DMB 在 Cortex-M 上很少需要，因为它们
  不重排内存事务"**(BIHEDAAF)。
- 调和：核**内**不重排、写缓冲对本核后续读透明；但**另一总线主机(IDMA)看到 SRAM 的时刻**
  与 CPU store 退休时刻之间存在写缓冲/总线矩阵延迟窗口——"启动 DMA 前需 DMB"正是为这个
  少数例外存在。两句不矛盾：日常单核不用 DMB，"交给 DMA 之前"是那个例外。

### 三层根因假设（按可能性 + 修复成本排序）

| 层 | 假设 | 证据强度 | 修法 | 成本 |
|---|---|---|---|---|
| **H1** | 缺 `__DMB()`：memcpy 新数据未落 RAM，IDMA 读旧字节 | 强（H7 同症状 + ARM 官方 DMA 前需 DMB） | memcpy 后 / 启动 IDMA 前 `__DMB()` | 改 1 行 |
| **H2** | bounce/ring **无 4 字节对齐**：IDMA 对 FIFO 做 32-bit 访问，源非对齐→丢/错字节 | 中（多帖证 sd_diskio 需对齐，与 cache 无关、对 M33 适用；实测我们 `s_sd_bounce`/所有 ring 都是裸 `uint8_t[]` 无 `__ALIGN`） | bounce/ring 加 `__attribute__((aligned(4)))`（保险给 32） | 改几行 |
| **H3** | 真硅 errata / 总线仲裁级 | 弱（未在 ES0499 证实，待查） | 换 SRAM bank 隔离 或 放弃 IDMA | 中～大 |

**H1+H2 可叠加、且互不冲突**——建议 A/B 时**一次同时加**（DMB + aligned），因为两者都零风险、
低成本，分开测反而多烧一轮。若加完仍坏→强烈指向 H3（硅级），那时再谈换 bank / 放弃 IDMA。

### 被排除项（本轮进一步确认）
- 32-**字节**对齐 / cache maintenance / `ENABLE_SD_DMA_CACHE_MAINTENANCE`：M7 D-cache line
  专属，**M33 无 D-cache 不适用**（勿照搬 H7 的 `SCB_CleanDCache`——U5 上是空操作/不存在）。
- U5+IDMA+CMD18 "strange data" 帖：纯 DBLOCKSIZE 配错，HAL 标准路径不会犯，排除。
- TrustZone/GTZC：已确认关闭，排除。

## 8.6 ★ L1 硬件 A/B 判决(2026-07-08):H1+H2 证伪

同机同配置(满配 96k mic + LSM6664 + QMA1600 + H3 400)两次长录：

| 通道 | 第一次 轮询(=0) | 第二次 IDMA+H1+H2(=1) |
|---|---|---|
| ACC_LOW | 0.0000%(2/915万,掉电末截断) | **0.1332%**(48981/3676万) |
| GYR_LOW | 0.0000% | **0.0675%** |
| ACC_HIGH | 0 | **0.0793%** |
| ACC_MID | 0 | **0.0331%** |
| MAG/ENV/TMP | 0 | **0**（全零） |

两次 `dma_err=0`、retry/fail/deinit 全 0（完全静默）。

**结论：H1(`__DMB()`)+ H2(bounce 32 字节对齐)【证伪】。** 加了屏障+对齐，损坏照样全面回来，
量级同历史 CKBX0079 的 0.40%。→ **不是缺屏障、不是未对齐。**

**坏行模式（逐字吻合 CKBX0292 历史记录）**：
- 字节插入：`1189,89,260708182020,...`（多插 `89,`）
- 吞逗号：`1607,260708182020,-9-144.4,...`（`-9,` 塌成 `-9`）
- 整行塌成 2 字节：`16` / `2` / `1`
- ★ 插入的字节（`89/10/20/21/99`）**都是 datetime `260708182020` 的片段** → 相邻 SRAM 字节被 IDMA 搬串位。

**决定性规律：仅高速率通道坏（ACC/GYR/H3/QMA），MAG/ENV/TMP 零坏。**
→ 损坏强度 ∝ CPU 写该 ring 的频率 → 这是"IDMA 读源 与 CPU 高频写区 撞同一 SRAM"的
总线矩阵/bank 级冒险签名。**三层假设里 H1+H2 死，H3(硅/仲裁级)得证。**

**唯一未试的软件方向 = L2 换 bank**（理论支撑因上述规律反而增强，见 §6 L2）。
但两点务必记住：① 仍不保证有效（可能是总线矩阵级而非 bank 级隔离能解决的）；
② **即便 L2 成功根治损坏，也治不了掉帧**（§7，PROGRAMMING-bound，与总线主机无关）。

## 9. 未决 / 需要用户或硬件才能推进的点

- ES0499 逐条内容（需下载 PDF）。
- 所有 A/B 需真机烧录 + 长录 + 读卡审计（需用户在 Keil GUI 编译、烧录、跑）。
- 「散布式 0.4% 损坏」是否单靠开头屏障可全解——只能实测回答。

