# STM32U575 三项基础设施优化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完善 STM32U575 FreeRTOS 传感器项目的三项基础设施：错误诊断、堆内存监控、SD 卡 DMA 传输。

**Architecture:** 分三个阶段实施。Phase 1 错误处理（为后续调试提供诊断能力）→ Phase 2 堆监控（低风险自包含）→ Phase 3 SD DMA（最复杂，依赖前两项）。每阶段独立可测试。

**Tech Stack:** STM32U575 (Cortex-M33), FreeRTOS, FatFs, SDMMC1 IDMA, HAL SD DMA

---

## File Structure

| 文件 | 职责 | 操作 |
|------|------|------|
| `Core/Src/stm32u5xx_it.c` | 故障处理器诊断输出 | 修改 |
| `Core/Src/main.c` | 启用可配置故障异常 | 修改 |
| `Core/Inc/FreeRTOSConfig.h` | configASSERT 改进 + hook 启用 | 修改 |
| `Core/Src/app_freertos.c` | 堆监控 + hook 实现 + DMA 信号量 | 修改 |
| `Core/Src/sdmmc.c` | 启用 SDMMC1 中断 | 修改 |
| `Core/Src/sd_diskio.c` | DMA 读写 + 完成回调 | 修改 |

---

## Phase 1: 错误处理改进

### Task 1: 启用可配置故障处理器

**Files:**
- Modify: `Core/Src/main.c:89` (HAL_Init 之后)

- [ ] **Step 1: 在 main() 中启用 MemManage/BusFault/UsageFault**

在 `main()` 函数中，`HAL_Init()` 之后、`osKernelInitialize()` 之前添加：

```c
  /* Enable configurable fault handlers — MemManage/BusFault/UsageFault
   * now enter their dedicated ISRs instead of silently escalating to HardFault. */
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
               | SCB_SHCSR_BUSFAULTENA_Msk
               | SCB_SHCSR_USGFAULTENA_Msk;
```

- [ ] **Step 2: 编译验证**

Run: 在 Keil MDK 中 Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 3: Commit**

```bash
git add Core/Src/main.c
git commit -m "feat: enable configurable fault handlers on Cortex-M33"
```

---

### Task 2: HardFault 诊断输出

**Files:**
- Modify: `Core/Src/stm32u5xx_it.c` (整个文件)

- [ ] **Step 1: 添加全局故障诊断变量**

在文件顶部 `USER CODE BEGIN PV` 区添加：

```c
/* USER CODE BEGIN PV */
volatile uint32_t g_fault_cfsr;
volatile uint32_t g_fault_hfsr;
volatile uint32_t g_fault_mmfar;
volatile uint32_t g_fault_bfar;
volatile uint32_t g_fault_stacked_pc;
volatile uint32_t g_fault_stacked_lr;
volatile uint32_t g_fault_type;  /* 1=Hard, 2=Mem, 3=Bus, 4=Usage */
/* USER CODE END PV */
```

- [ ] **Step 2: 添加 stdio.h include**

在 includes 区添加：
```c
#include <stdio.h>
```

- [ ] **Step 3: 实现 HardFault 处理器（naked + C 诊断）**

替换现有的 `HardFault_Handler`：

```c
void HardFault_Handler(void)
{
    __asm volatile(
        "TST LR, #4\n"
        "ITE EQ\n"
        "MRSEQ R0, MSP\n"
        "MRSNE R0, PSP\n"
        "B hard_fault_c\n"
    );
}

__attribute__((used))
static void hard_fault_c(uint32_t stacked_sp)
{
    g_fault_type = 1U;

    uint32_t *sp = (uint32_t *)stacked_sp;
    g_fault_stacked_pc = sp[6];
    g_fault_stacked_lr = sp[5];

    g_fault_cfsr  = SCB->CFSR;
    g_fault_hfsr  = SCB->HFSR;
    g_fault_mmfar = SCB->MMFAR;
    g_fault_bfar  = SCB->BFAR;

    if (g_fault_cfsr & 0xFFU)
    {
        printf("[FAULT] MMFSR=0x%02lx", (unsigned long)(g_fault_cfsr & 0xFFU));
        if (g_fault_cfsr & (1U << 0)) printf(" IACCVIOL");
        if (g_fault_cfsr & (1U << 1)) printf(" DACCVIOL");
        if (g_fault_cfsr & (1U << 3)) printf(" MMARVALID");
        if (g_fault_cfsr & (1U << 4)) printf(" MLSPERR");
        if (g_fault_cfsr & (1U << 5)) printf(" MSTKERR");
        if (g_fault_cfsr & (1U << 6)) printf(" MUNSTKERR");
        printf(" MMFAR=0x%08lx", (unsigned long)g_fault_mmfar);
        printf("\r\n");
    }

    if (g_fault_cfsr & 0xFF00U)
    {
        printf("[FAULT] BFSR=0x%02lx", (unsigned long)((g_fault_cfsr >> 8) & 0xFFU));
        if (g_fault_cfsr & (1U << 8))  printf(" IBUSERR");
        if (g_fault_cfsr & (1U << 9))  printf(" PRECISERR");
        if (g_fault_cfsr & (1U << 10)) printf(" IMPRECISERR");
        if (g_fault_cfsr & (1U << 11)) printf(" STKERR");
        if (g_fault_cfsr & (1U << 12)) printf(" UNSTKERR");
        if (g_fault_cfsr & (1U << 15)) printf(" BFARVALID");
        printf(" BFAR=0x%08lx", (unsigned long)g_fault_bfar);
        printf("\r\n");
    }

    if (g_fault_cfsr & 0xFFFF0000U)
    {
        printf("[FAULT] UFSR=0x%04lx", (unsigned long)((g_fault_cfsr >> 16) & 0xFFFFU));
        if (g_fault_cfsr & (1U << 16)) printf(" UNDEFINSTR");
        if (g_fault_cfsr & (1U << 17)) printf(" INVSTATE");
        if (g_fault_cfsr & (1U << 18)) printf(" INVPC");
        if (g_fault_cfsr & (1U << 19)) printf(" NOCP");
        if (g_fault_cfsr & (1U << 24)) printf(" STKOF");
        printf("\r\n");
    }

    printf("[FAULT] HFSR=0x%08lx\r\n", (unsigned long)g_fault_hfsr);
    printf("[FAULT] PC=0x%08lx LR=0x%08lx SP=0x%08lx\r\n",
           (unsigned long)g_fault_stacked_pc,
           (unsigned long)g_fault_stacked_lr,
           (unsigned long)stacked_sp);

    __BKPT(0);
    while (1) { }
}
```

- [ ] **Step 4: 实现可配置故障处理器**

替换 MemManage_Handler / BusFault_Handler / UsageFault_Handler：

```c
void MemManage_Handler(void)
{
    g_fault_type = 2U;
    g_fault_cfsr  = SCB->CFSR;
    g_fault_mmfar = SCB->MMFAR;
    printf("[FAULT] MemManage CFSR=0x%08lx MMFAR=0x%08lx\r\n",
           (unsigned long)g_fault_cfsr, (unsigned long)g_fault_mmfar);
    __BKPT(0);
    while (1) { }
}

void BusFault_Handler(void)
{
    g_fault_type = 3U;
    g_fault_cfsr = SCB->CFSR;
    g_fault_bfar = SCB->BFAR;
    printf("[FAULT] BusFault CFSR=0x%08lx BFAR=0x%08lx\r\n",
           (unsigned long)g_fault_cfsr, (unsigned long)g_fault_bfar);
    __BKPT(0);
    while (1) { }
}

void UsageFault_Handler(void)
{
    g_fault_type = 4U;
    g_fault_cfsr = SCB->CFSR;
    printf("[FAULT] UsageFault CFSR=0x%08lx\r\n", (unsigned long)g_fault_cfsr);
    __BKPT(0);
    while (1) { }
}
```

- [ ] **Step 5: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 6: Commit**

```bash
git add Core/Src/stm32u5xx_it.c
git commit -m "feat: add diagnostic fault handlers with printf on HardFault/MemManage/BusFault/UsageFault"
```

---

### Task 3: 改进 configASSERT

**Files:**
- Modify: `Core/Inc/FreeRTOSConfig.h:164`

- [ ] **Step 1: 替换 configASSERT 定义**

将现有的：
```c
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
```

替换为：
```c
#define configASSERT( x )                                        \
    if ((x) == 0) {                                              \
        printf("[ASSERT] %s:%d\r\n", __FILE__, __LINE__);        \
        taskDISABLE_INTERRUPTS();                                \
        __BKPT(0);                                               \
        for( ;; );                                               \
    }
```

- [ ] **Step 2: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/FreeRTOSConfig.h
git commit -m "feat: improve configASSERT with file/line diagnostic output"
```

---

## Phase 2: 堆内存监控

### Task 4: 启用 FreeRTOS Hook

**Files:**
- Modify: `Core/Inc/FreeRTOSConfig.h`

- [ ] **Step 1: 添加 hook 配置**

在 `USER CODE BEGIN 0` 区添加：

```c
#define configUSE_MALLOC_FAILED_HOOK      1
#define configCHECK_FOR_STACK_OVERFLOW    2
```

- [ ] **Step 2: 编译验证**

Run: Keil Build (F7)
Expected: 编译器会报未定义符号错误（hook 函数未实现），这是预期的

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/FreeRTOSConfig.h
git commit -m "feat: enable FreeRTOS malloc failed hook and stack overflow check"
```

---

### Task 5: 实现 Hook + 堆追踪

**Files:**
- Modify: `Core/Src/app_freertos.c` (多处)

- [ ] **Step 1: 添加堆监控全局变量**

在 `USER CODE BEGIN Variables` 区添加：

```c
/* Heap monitoring — tracks minimum free heap over time */
static volatile uint32_t g_min_free_heap = 0xFFFFFFFF;
static volatile uint32_t g_last_heap_sample_tick = 0U;
#define HEAP_SAMPLE_INTERVAL_TICKS  1000U  /* sample every 1 second */
```

- [ ] **Step 2: 添加堆追踪函数**

在 `USER CODE BEGIN Application` 区添加：

```c
/* ---- Heap monitoring ---- */
void App_HeapTrackUpdate(void)
{
    uint32_t now = osKernelGetTickCount();
    if ((now - g_last_heap_sample_tick) >= HEAP_SAMPLE_INTERVAL_TICKS)
    {
        g_last_heap_sample_tick = now;
        uint32_t free_heap = xPortGetFreeHeapSize();
        if (free_heap < g_min_free_heap)
        {
            g_min_free_heap = free_heap;
        }
    }
}

uint32_t App_HeapGetMinFree(void)
{
    return g_min_free_heap;
}
```

- [ ] **Step 3: 实现 Malloc Failed Hook**

```c
void vApplicationMallocFailedHook(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    const char *name = (task != NULL) ? pcTaskGetTaskName(task) : "??";
    printf("[HOOK] Malloc failed in task: %s, free heap: %lu\r\n",
           name, (unsigned long)xPortGetFreeHeapSize());
    __BKPT(0);
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}
```

- [ ] **Step 4: 实现 Stack Overflow Hook**

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("[HOOK] Stack overflow in task: %s\r\n", pcTaskName);
    (void)xTask;
    __BKPT(0);
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}
```

- [ ] **Step 5: 在 Logger 任务中调用堆追踪**

找到 Logger 任务主循环（`StartLoggerTask` 的 `for(;;)` 循环开头），添加：

```c
App_HeapTrackUpdate();
```

- [ ] **Step 6: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 7: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat: add heap monitoring with min-free tracking and malloc/stack overflow hooks"
```

---

### Task 6: 更新 USB Status 输出

**Files:**
- Modify: `Core/Src/app_freertos.c` (UsbCmd_Status 函数)

- [ ] **Step 1: 修改 status 输出增加 heap_min**

找到 `USB_STATUS_LINE("tick=%lu heap=%lu\r\n"` 调用，替换为：

```c
USB_STATUS_LINE("tick=%lu heap=%lu heap_min=%lu\r\n",
                (unsigned long)tick,
                (unsigned long)xPortGetFreeHeapSize(),
                (unsigned long)App_HeapGetMinFree());
```

- [ ] **Step 2: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 3: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat: add heap_min to USB status output"
```

---

## Phase 3: SD 卡 DMA 迁移

### Task 7: 启用 SDMMC1 中断

**Files:**
- Modify: `Core/Src/sdmmc.c` (HAL_SD_MspInit 函数)

- [ ] **Step 1: 启用 SDMMC1 IRQ 并设置优先级**

替换 `HAL_SD_MspInit` 中关于 IRQ disabled 的注释块（约 lines 103-106）：

将：
```c
  /* SDMMC1 IRQ disabled -- polling mode only ... */
```

替换为：
```c
  /* Enable SDMMC1 interrupt for DMA completion notification.
   * Priority 6 > configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5),
   * so xSemaphoreGiveFromISR() is safe from HAL_SD_IRQHandler callback. */
  HAL_NVIC_SetPriority(SDMMC1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
```

- [ ] **Step 2: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 3: Commit**

```bash
git add Core/Src/sdmmc.c
git commit -m "feat: enable SDMMC1 IRQ for DMA completion notification"
```

---

### Task 8: 创建 DMA 完成信号量

**Files:**
- Modify: `Core/Src/app_freertos.c`

- [ ] **Step 1: 添加信号量声明**

在 `USER CODE BEGIN Variables` 区添加：

```c
static SemaphoreHandle_t s_sdmmc_dma_sem;  /* signaled by HAL SD DMA completion ISR */
```

- [ ] **Step 2: 创建信号量**

在 `MX_FREERTOS_Init()` 的 `USER CODE BEGIN RTOS_SEMAPHORES` 区添加：

```c
s_sdmmc_dma_sem = xSemaphoreCreateBinary();
```

- [ ] **Step 3: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 4: Commit**

```bash
git add Core/Src/app_freertos.c
git commit -m "feat: add SDMMC DMA completion binary semaphore"
```

---

### Task 9: 实现 HAL SD DMA 回调

**Files:**
- Modify: `Core/Src/sd_diskio.c`

- [ ] **Step 1: 添加 includes 和 extern**

在文件顶部添加：

```c
#include "sd_diskio.h"
#include "sdmmc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>

extern SemaphoreHandle_t s_sdmmc_dma_sem;
```

- [ ] **Step 2: 实现 TxCpltCallback（写完成）**

```c
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (s_sdmmc_dma_sem != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_sdmmc_dma_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

- [ ] **Step 3: 实现 RxCpltCallback（读完成）**

```c
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (s_sdmmc_dma_sem != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_sdmmc_dma_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

- [ ] **Step 4: 实现 ErrorCallback**

```c
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    if (s_sdmmc_dma_sem != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(s_sdmmc_dma_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

- [ ] **Step 5: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 6: Commit**

```bash
git add Core/Src/sd_diskio.c
git commit -m "feat: implement HAL SD DMA completion callbacks with FreeRTOS semaphore"
```

---

### Task 10: 替换 SD_disk_write 为 DMA 模式

**Files:**
- Modify: `Core/Src/sd_diskio.c` (SD_disk_write 函数)

- [ ] **Step 1: 替换 SD_disk_write 函数**

将整个 `SD_disk_write` 函数替换为：

```c
DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  uint8_t retries;

  if ((pdrv != SDDISKIO_DRIVE_NUM) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SD_GetDriveStatus(pdrv) & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  for (retries = 0U; retries < 3U; retries++)
  {
    if (HAL_SD_WriteBlocks_DMA(&hsd1, buff, (uint32_t)sector, (uint32_t)count) != HAL_OK)
    {
      HAL_SD_DeInit(&hsd1);
      MX_SDMMC1_SD_Init();
      continue;
    }

    if (xSemaphoreTake(s_sdmmc_dma_sem, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
      if (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
      {
        return RES_OK;
      }
    }

    HAL_SD_DeInit(&hsd1);
    MX_SDMMC1_SD_Init();
  }

  return RES_ERROR;
}
```

- [ ] **Step 2: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 3: Commit**

```bash
git add Core/Src/sd_diskio.c
git commit -m "feat: migrate SD_disk_write from polling to IDMA with semaphore wait"
```

---

### Task 11: 替换 SD_disk_read 为 DMA 模式

**Files:**
- Modify: `Core/Src/sd_diskio.c` (SD_disk_read 函数)

- [ ] **Step 1: 替换 SD_disk_read 函数**

将整个 `SD_disk_read` 函数替换为：

```c
DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  uint8_t retries;

  if ((pdrv != SDDISKIO_DRIVE_NUM) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((SD_GetDriveStatus(pdrv) & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  for (retries = 0U; retries < 3U; retries++)
  {
    if (HAL_SD_ReadBlocks_DMA(&hsd1, buff, (uint32_t)sector, (uint32_t)count) != HAL_OK)
    {
      HAL_SD_DeInit(&hsd1);
      MX_SDMMC1_SD_Init();
      continue;
    }

    if (xSemaphoreTake(s_sdmmc_dma_sem, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
      if (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
      {
        return RES_OK;
      }
    }

    HAL_SD_DeInit(&hsd1);
    MX_SDMMC1_SD_Init();
  }

  return RES_ERROR;
}
```

- [ ] **Step 2: 编译验证**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 3: Commit**

```bash
git add Core/Src/sd_diskio.c
git commit -m "feat: migrate SD_disk_read from polling to IDMA with semaphore wait"
```

---

### Task 12: 端到端验证

- [ ] **Step 1: 编译完整项目**

Run: Keil Build (F7)
Expected: 0 errors, 0 warnings

- [ ] **Step 2: Phase B 冒烟测试**

烧录后通过 USB CDC 发送 Phase B 测试命令，验证：
- SD 卡挂载成功
- 写入测试文件成功
- 回读内容匹配
- 底层扇区读取验证通过

Expected: `[FatFs] 阶段B测试成功: 挂载/写入/关闭/回读 已通过`

- [ ] **Step 3: SD 采集测试**

发送 `acq_start sd 5000`，验证：
- 会话目录创建成功
- LSM_IMU.CSV / H3_ACC.CSV / QMA_ACC.CSV / LSM_TMP.CSV 数据正常
- 采集停止后文件关闭正常

Expected: 数据行数 > 0，采样率匹配配置值

- [ ] **Step 4: USB 采集测试**

发送 `acq_start usb 2000`，验证 CSV 数据输出正常

- [ ] **Step 5: 堆监控验证**

发送 `status` 命令，验证输出包含 `heap_min=` 字段

Expected: `heap_min` 值合理（不应为 0）

- [ ] **Step 6: Commit 最终状态**

```bash
git add -A
git commit -m "chore: verify SD DMA + heap monitoring + error handling integration"
```
