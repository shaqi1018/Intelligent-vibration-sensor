# DMA高速采样实施计划

**项目：** STM32U575 三传感器平台 DMA高速采样
**目标采样率：** 25.6kHz（可调）
**当前状态：** 阶段C完成（轮询模式，100Hz）
**编制日期：** 2026-05-05

---

## 一、项目背景

### 1.1 当前系统状态

**已完成阶段：**
- ✅ 阶段A：USBX CDC ACM枚举成功
- ✅ 阶段B：基础命令通道与USB/SD智能切换
- ✅ 阶段C：独立USB上传任务与架构优化

**当前性能：**
- 采样率：100Hz（10ms周期）
- 传输方式：SPI轮询模式
- CPU占用：~1.5%
- 数据流：传感器 → 快照 → USB/SD

**性能瓶颈：**
- 单传感器SPI轮询：~500μs
- 3传感器总耗时：~1500μs
- 最大理论采样率：~666Hz
- **目标采样率25.6kHz需要DMA支持**

### 1.2 目标需求

**核心目标：**
- 实现25.6kHz连续采样（3个传感器）
- 保持系统稳定性
- 保留现有USB/SD数据记录功能
- 支持采样率可调（1kHz ~ 25.6kHz）

**性能指标：**
- 采样率：25.6kHz ±0.1%
- 数据丢失率：<0.01%
- CPU占用：<50%
- 内存占用：<100KB额外

---

## 二、技术方案设计

### 2.1 整体架构

**新架构设计：**

```
┌─────────────────────────────────────────────────────────────┐
│                    定时器触发源 (TIM6)                        │
│                    25.6kHz 精确时钟                           │
└────────────┬────────────────────────────────────────────────┘
             │ 触发
             ↓
┌─────────────────────────────────────────────────────────────┐
│                    DMA传输层                                  │
├─────────────────────────────────────────────────────────────┤
│  SPI1 + DMA  →  LSM6DSOX循环缓冲区 (512样本)                 │
│  SPI2 + DMA  →  H3LIS100DL循环缓冲区 (512样本)               │
│  SPI2 + DMA  →  QMA6100P循环缓冲区 (512样本)                 │
└────────────┬────────────────────────────────────────────────┘
             │ DMA半满/全满中断
             ↓
┌─────────────────────────────────────────────────────────────┐
│                数据处理任务 (高优先级)                        │
├─────────────────────────────────────────────────────────────┤
│  - 原始数据解析                                               │
│  - 数据校验                                                   │
│  - 降采样/滤波 (25.6kHz → 1kHz)                              │
│  - 峰值检测                                                   │
└────────────┬────────────────────────────────────────────────┘
             │ 批量更新
             ↓
┌─────────────────────────────────────────────────────────────┐
│                快照更新 (1kHz)                                │
│            AppSensorSnapshot_t                                │
└────────────┬────────────────────────────────────────────────┘
             │ 消费
             ↓
┌─────────────────────────────────────────────────────────────┐
│        loggerTask (100Hz)  |  usbUploadTask (100Hz)          │
│        SD卡记录            |  USB实时上传                     │
└─────────────────────────────────────────────────────────────┘
```

**关键变化：**
1. 定时器触发DMA，CPU不参与采样
2. 循环缓冲区后台自动填充
3. 中断驱动的数据处理
4. 多级降采样减少后端压力

### 2.2 定时器配置

**TIM6配置（采样时钟源）：**

```c
/* 25.6kHz = 160MHz / (6250 * 1) */
TIM6->PSC = 0;              /* 预分频器：1 */
TIM6->ARR = 6249;           /* 自动重载：6250 */
TIM6->CR2 |= TIM_CR2_MMS_1; /* TRGO on update event */
```

**时序精度：**
- 系统时钟：160MHz
- 定时器周期：160MHz / 6250 = 25.6kHz
- 精度：±0.00625% (晶振精度)

**可调采样率实现：**
```c
void SetSamplingRate(uint32_t rate_hz)
{
  uint32_t arr = (160000000 / rate_hz) - 1;
  TIM6->ARR = arr;
}
```

支持范围：1kHz ~ 50kHz

### 2.3 SPI DMA配置

**DMA通道分配：**

| 外设 | DMA控制器 | 通道 | 方向 | 优先级 |
|------|-----------|------|------|--------|
| SPI1_RX | GPDMA1 | CH0 | 外设→内存 | Very High |
| SPI1_TX | GPDMA1 | CH1 | 内存→外设 | High |
| SPI2_RX | GPDMA1 | CH2 | 外设→内存 | Very High |
| SPI2_TX | GPDMA1 | CH3 | 内存→外设 | High |

**循环缓冲区设计：**

```c
/* 每个传感器的DMA缓冲区 */
#define DMA_BUFFER_SIZE  512  /* 样本数量 */

typedef struct {
  uint8_t tx_buf[DMA_BUFFER_SIZE * 16];  /* 发送缓冲（SPI命令） */
  uint8_t rx_buf[DMA_BUFFER_SIZE * 16];  /* 接收缓冲（传感器数据） */
  uint32_t half_cplt_count;              /* 半满中断计数 */
  uint32_t full_cplt_count;              /* 全满中断计数 */
  uint32_t error_count;                  /* 错误计数 */
} SensorDmaBuffer_t;

SensorDmaBuffer_t lsm6dsox_dma __attribute__((aligned(32)));
SensorDmaBuffer_t h3lis_dma __attribute__((aligned(32)));
SensorDmaBuffer_t qma_dma __attribute__((aligned(32)));
```

**内存对齐说明：**
- STM32U5有DCache，需要32字节对齐
- 避免Cache一致性问题
- 提高DMA传输效率

**SPI1 DMA配置示例（LSM6DSOX）：**

```c
void SPI1_DMA_Init(void)
{
  /* 配置DMA请求 */
  LL_DMA_SetPeriphRequest(GPDMA1, LL_DMA_CHANNEL_0, LL_GPDMA1_REQUEST_SPI1_RX);
  LL_DMA_SetPeriphRequest(GPDMA1, LL_DMA_CHANNEL_1, LL_GPDMA1_REQUEST_SPI1_TX);
  
  /* RX通道：循环模式 */
  LL_DMA_SetMode(GPDMA1, LL_DMA_CHANNEL_0, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetDataLength(GPDMA1, LL_DMA_CHANNEL_0, DMA_BUFFER_SIZE * 16);
  LL_DMA_SetMemoryAddress(GPDMA1, LL_DMA_CHANNEL_0, (uint32_t)lsm6dsox_dma.rx_buf);
  LL_DMA_SetPeriphAddress(GPDMA1, LL_DMA_CHANNEL_0, (uint32_t)&SPI1->RXDR);
  
  /* TX通道：循环模式 */
  LL_DMA_SetMode(GPDMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetDataLength(GPDMA1, LL_DMA_CHANNEL_1, DMA_BUFFER_SIZE * 16);
  LL_DMA_SetMemoryAddress(GPDMA1, LL_DMA_CHANNEL_1, (uint32_t)lsm6dsox_dma.tx_buf);
  LL_DMA_SetPeriphAddress(GPDMA1, LL_DMA_CHANNEL_1, (uint32_t)&SPI1->TXDR);
  
  /* 使能半满和全满中断 */
  LL_DMA_EnableIT_HT(GPDMA1, LL_DMA_CHANNEL_0);
  LL_DMA_EnableIT_TC(GPDMA1, LL_DMA_CHANNEL_0);
  
  /* 配置中断优先级 */
  NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0);  /* 最高优先级 */
  NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
}
```

### 2.4 SPI2共享总线处理

**挑战：** H3LIS100DL和QMA6100P共享SPI2，无法同时DMA传输

**解决方案：时分复用**

```
时间轴（39μs周期）:
├─ 0-15μs:  H3LIS100DL DMA传输
├─ 15-16μs: CS切换延迟
├─ 16-31μs: QMA6100P DMA传输
├─ 31-39μs: 空闲时间
```

**实现方式：**
1. TIM6触发H3LIS100DL DMA
2. H3LIS DMA完成中断触发QMA6100P DMA
3. 两个传感器顺序采样，共享同一个25.6kHz时钟

**代码框架：**
```c
void GPDMA1_Channel2_IRQHandler(void)  /* SPI2 RX (H3LIS) */
{
  if (LL_DMA_IsActiveFlag_TC(GPDMA1, LL_DMA_CHANNEL_2))
  {
    LL_DMA_ClearFlag_TC(GPDMA1, LL_DMA_CHANNEL_2);
    
    /* H3LIS传输完成，立即启动QMA传输 */
    H3LIS_CS_HIGH();
    QMA_CS_LOW();
    LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_3);  /* 启动QMA DMA */
  }
}
```

### 2.5 数据处理与降采样策略

**多级降采样架构：**

```
25.6kHz DMA采样 (原始数据)
    ↓ 降采样 1:25.6
1kHz 数据处理 (快照更新)
    ↓ 降采样 1:10
100Hz 数据记录 (USB/SD)
```

**降采样算法选择：**

| 方法 | CPU开销 | 精度 | 适用场景 |
|------|---------|------|----------|
| 抽取 | 极低 | 低 | 快速预览 |
| 平均 | 低 | 中 | 平滑信号 |
| 中值滤波 | 中 | 高 | 去噪声 |
| FIR滤波 | 高 | 最高 | 精确分析 |

**推荐方案：** 抽取+峰值保留

```c
/* 数据处理任务 */
void DataProcessTask(void *argument)
{
  uint32_t sample_count = 0;
  
  for (;;)
  {
    /* 等待DMA半满/全满信号 */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    /* 处理256个样本 */
    for (uint32_t i = 0; i < 256; i++)
    {
      /* 解析原始数据 */
      ParseSensorData(&lsm6dsox_dma.rx_buf[i * 16], &raw_data);
      
      /* 每25.6个样本更新一次快照（1kHz） */
      if ((sample_count % 26) == 0)
      {
        UpdateSnapshot(&raw_data);
      }
      
      sample_count++;
    }
  }
}
```

**峰值保留策略：**
```c
/* 在降采样窗口内保留最大值 */
typedef struct {
  int16_t max_value;
  int16_t min_value;
  int32_t sum;
  uint32_t count;
} DecimationWindow_t;

void UpdateDecimationWindow(DecimationWindow_t *win, int16_t value)
{
  if (value > win->max_value) win->max_value = value;
  if (value < win->min_value) win->min_value = value;
  win->sum += value;
  win->count++;
}
```

### 2.6 内存管理

**内存需求估算：**

| 项目 | 大小 | 数量 | 总计 |
|------|------|------|------|
| LSM6DSOX DMA缓冲 | 8KB | 1 | 8KB |
| H3LIS DMA缓冲 | 8KB | 1 | 8KB |
| QMA DMA缓冲 | 8KB | 1 | 8KB |
| 数据处理缓冲 | 4KB | 1 | 4KB |
| USB发送缓冲 | 4KB | 1 | 4KB |
| SD写入缓冲 | 8KB | 1 | 8KB |
| **总计** | - | - | **40KB** |

**当前内存状态：**
- SRAM总量：192KB
- 当前占用：74KB
- DMA新增：40KB
- 剩余可用：78KB
- **结论：内存充足**

**Cache一致性处理：**

```c
/* DMA传输前：清除DCache */
SCB_CleanDCache_by_Addr((uint32_t*)tx_buf, buffer_size);

/* DMA传输后：使DCache无效 */
SCB_InvalidateDCache_by_Addr((uint32_t*)rx_buf, buffer_size);
```

### 2.7 中断优先级设计

**中断优先级分配：**

| 中断源 | 优先级 | 抢占优先级 | 子优先级 | 说明 |
|--------|--------|-----------|---------|------|
| DMA传输完成 | 0 | 0 | 0 | 最高，保证数据不丢失 |
| SPI错误 | 1 | 1 | 0 | 及时处理传输错误 |
| TIM6更新 | 2 | 2 | 0 | 定时器触发 |
| USB中断 | 5 | 5 | 0 | 中等优先级 |
| FreeRTOS SysTick | 15 | 15 | 0 | 最低，可被抢占 |

**FreeRTOS配置：**
```c
/* FreeRTOSConfig.h */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  (5 << 4)  /* 优先级5以下可调用API */
```

**中断延迟要求：**
- DMA中断响应时间：<5μs
- 数据处理延迟：<1ms
- 总延迟：<10ms（对于1kHz快照更新）

---

## 三、实施路线图

### 3.1 阶段划分

**总体时间：10-15天**

```
阶段1: 单传感器DMA验证 (3-4天)
    ↓
阶段2: 多传感器DMA集成 (2-3天)
    ↓
阶段3: 数据流优化 (2-3天)
    ↓
阶段4: 压力与异常测试 (3-5天)
```

### 3.2 阶段1：单传感器DMA验证（3-4天）

**目标：** LSM6DSOX实现25.6kHz DMA采样

**任务清单：**

**Day 1: 基础配置**
- [ ] 配置TIM6定时器（25.6kHz）
- [ ] 配置GPDMA1通道0/1（SPI1 RX/TX）
- [ ] 实现循环缓冲区
- [ ] 编写DMA中断处理函数
- [ ] 验证：定时器触发频率正确

**Day 2: 数据传输**
- [ ] 准备SPI命令序列（读取加速度+陀螺仪）
- [ ] 启动DMA循环传输
- [ ] 实现半满/全满中断处理
- [ ] 验证：DMA缓冲区正确填充

**Day 3: 数据解析**
- [ ] 编写原始数据解析函数
- [ ] 实现数据校验（CRC/范围检查）
- [ ] 创建数据处理任务
- [ ] 验证：解析后的数据正确

**Day 4: 集成测试**
- [ ] 集成到现有快照模型
- [ ] 实现降采样（25.6kHz → 1kHz）
- [ ] 测试USB/SD数据记录
- [ ] 性能分析（CPU占用、内存使用）

**验收标准：**
- ✅ LSM6DSOX稳定运行在25.6kHz
- ✅ 数据丢失率<0.01%
- ✅ CPU占用<20%
- ✅ 连续运行1小时无错误

### 3.3 阶段2：多传感器DMA集成（2-3天）

**目标：** 3个传感器同时运行在25.6kHz

**任务清单：**

**Day 5: SPI2 DMA配置**
- [ ] 配置GPDMA1通道2/3（SPI2 RX/TX）
- [ ] 实现H3LIS100DL DMA传输
- [ ] 实现QMA6100P DMA传输
- [ ] 实现SPI2时分复用逻辑

**Day 6: 时序同步**
- [ ] 调整H3LIS和QMA的时序关系
- [ ] 优化CS切换延迟
- [ ] 验证3个传感器数据时间戳对齐
- [ ] 测试时序稳定性

**Day 7: 集成优化**
- [ ] 统一数据处理流程
- [ ] 优化中断处理开销
- [ ] 调整任务优先级
- [ ] 性能调优

**验收标准：**
- ✅ 3个传感器稳定运行在25.6kHz
- ✅ 时间戳对齐误差<1μs
- ✅ CPU占用<40%
- ✅ 连续运行2小时无错误

### 3.4 阶段3：数据流优化（2-3天）

**目标：** 优化整体数据流性能

**任务清单：**

**Day 8: 降采样优化**
- [ ] 实现多级降采样（25.6kHz→1kHz→100Hz）
- [ ] 实现峰值保留算法
- [ ] 优化快照更新频率
- [ ] 测试降采样精度

**Day 9: USB/SD优化**
- [ ] 实现USB批量上传
- [ ] 实现SD卡批量写入
- [ ] 优化缓冲区管理
- [ ] 测试数据吞吐量

**Day 10: 内存与性能**
- [ ] 内存使用分析
- [ ] CPU占用分析
- [ ] 中断延迟测量
- [ ] 性能瓶颈识别与优化

**验收标准：**
- ✅ CPU占用<50%
- ✅ 内存占用<100KB额外
- ✅ USB上传不丢数据
- ✅ SD卡写入不阻塞采样

### 3.5 阶段4：压力与异常测试（3-5天）

**目标：** 验证系统稳定性和可靠性

**测试项目：**

**1. 长时间稳定性测试（Day 11-12）**
- [ ] 24小时连续运行测试
- [ ] 监控数据丢失率
- [ ] 监控内存泄漏
- [ ] 监控CPU温度
- [ ] 记录所有错误和异常

**2. 极限性能测试（Day 12-13）**
- [ ] 测试最高采样率（50kHz）
- [ ] 测试最低采样率（1kHz）
- [ ] 测试采样率动态切换
- [ ] 测试多任务并发压力
- [ ] 测试USB/SD同时高速写入

**3. 异常场景测试（Day 13-14）**
- [ ] 传感器通信失败
- [ ] DMA传输错误
- [ ] USB断开/重连
- [ ] SD卡拔出/插入
- [ ] 电源波动模拟
- [ ] 温度变化测试

**4. 数据完整性测试（Day 14-15）**
- [ ] 数据校验和验证
- [ ] 时间戳连续性检查
- [ ] 峰值检测准确性
- [ ] 降采样精度验证
- [ ] USB/SD数据一致性

**验收标准：**
- ✅ 24小时运行无崩溃
- ✅ 数据丢失率<0.01%
- ✅ 所有异常场景正确处理
- ✅ 数据完整性100%

---

## 四、风险分析与应对

### 4.1 技术风险

**风险1：DMA传输冲突**
- **描述：** SPI2共享总线可能导致DMA冲突
- **影响：** 高
- **概率：** 中
- **应对：** 
  - 实现严格的时分复用
  - 增加CS切换保护时间
  - 备选方案：降低SPI2传感器采样率

**风险2：中断延迟过高**
- **描述：** 高频中断可能导致系统响应变慢
- **影响：** 中
- **概率：** 中
- **应对：**
  - 优化中断处理代码
  - 使用批量处理减少中断频率
  - 调整中断优先级

**风险3：Cache一致性问题**
- **描述：** DCache可能导致数据不一致
- **影响：** 高
- **概率：** 低
- **应对：**
  - 所有DMA缓冲区32字节对齐
  - 正确使用Cache清除/无效化API
  - 考虑禁用DCache（性能换稳定性）

**风险4：内存不足**
- **描述：** DMA缓冲区占用大量内存
- **影响：** 中
- **概率：** 低
- **应对：**
  - 减小缓冲区大小
  - 优化内存分配策略
  - 监控内存使用情况

### 4.2 项目风险

**风险5：时间估算偏差**
- **描述：** 实际开发时间可能超出预期
- **影响：** 低
- **概率：** 中
- **应对：**
  - 预留20%缓冲时间
  - 分阶段验收，及时调整
  - 优先保证核心功能

**风险6：硬件限制**
- **描述：** 传感器或MCU性能不足
- **影响：** 高
- **概率：** 低
- **应对：**
  - 提前进行性能测试
  - 准备降级方案（降低采样率）
  - 考虑硬件升级路径

---

## 五、关键交付物

### 5.1 代码模块

**新增文件：**
```
Core/Src/dma_sampling.c          /* DMA采样核心逻辑 */
Core/Inc/dma_sampling.h          /* DMA采样接口 */
Core/Src/data_processing.c       /* 数据处理任务 */
Core/Inc/data_processing.h       /* 数据处理接口 */
Core/Src/decimation.c            /* 降采样算法 */
Core/Inc/decimation.h            /* 降采样接口 */
```

**修改文件：**
```
Core/Src/app_freertos.c          /* 任务创建与配置 */
Core/Src/main.c                  /* 定时器初始化 */
Core/Inc/stm32u5xx_hal_conf.h   /* HAL配置 */
MDK-ARM/SensorProj.uvprojx       /* 工程文件 */
```

### 5.2 文档交付

- [ ] DMA配置说明文档
- [ ] 数据流架构图
- [ ] 性能测试报告
- [ ] 用户使用手册
- [ ] 故障排查指南

### 5.3 测试报告

- [ ] 单元测试报告
- [ ] 集成测试报告
- [ ] 性能测试报告
- [ ] 稳定性测试报告
- [ ] 异常场景测试报告

---

## 六、成功标准

### 6.1 功能标准

- ✅ 支持25.6kHz连续采样（3个传感器）
- ✅ 支持采样率可调（1kHz ~ 50kHz）
- ✅ 数据丢失率<0.01%
- ✅ 时间戳精度±1μs
- ✅ USB/SD数据记录正常工作
- ✅ 保留现有命令通道功能

### 6.2 性能标准

- ✅ CPU占用<50%（25.6kHz采样）
- ✅ 内存占用<100KB额外
- ✅ DMA中断延迟<5μs
- ✅ 数据处理延迟<1ms
- ✅ USB上传速率>100KB/s
- ✅ SD卡写入速率>50KB/s

### 6.3 稳定性标准

- ✅ 24小时连续运行无崩溃
- ✅ 所有异常场景正确处理
- ✅ 内存无泄漏
- ✅ 温度范围：-20°C ~ 70°C
- ✅ 电源波动：±10%

### 6.4 代码质量标准

- ✅ 编译0错误0警告
- ✅ 代码注释覆盖率>30%
- ✅ 关键函数有单元测试
- ✅ 符合MISRA-C规范（可选）
- ✅ 代码审查通过

---

## 七、总结

### 7.1 项目概述

本计划旨在将STM32U575三传感器平台从当前的100Hz轮询采样升级到25.6kHz DMA高速采样，以满足实际工程应用中的高速数据采集需求。

### 7.2 关键技术点

1. **定时器触发DMA** - 精确的25.6kHz采样时钟
2. **循环缓冲区** - 后台自动填充，减少CPU干预
3. **时分复用** - 解决SPI2共享总线问题
4. **多级降采样** - 平衡数据量与处理能力
5. **Cache一致性** - 保证DMA数据正确性

### 7.3 预期收益

**性能提升：**
- 采样率：100Hz → 25.6kHz（256倍提升）
- CPU效率：轮询阻塞 → DMA后台传输
- 数据质量：支持峰值检测和高频信号分析

**架构优化：**
- 更清晰的数据流
- 更好的实时性
- 更强的扩展性

### 7.4 实施建议

**优先级：**
1. **高优先级**：阶段1（单传感器DMA验证）
2. **中优先级**：阶段2（多传感器集成）
3. **中优先级**：阶段3（数据流优化）
4. **低优先级**：阶段4（压力测试）

**资源需求：**
- 开发时间：10-15天
- 人力：1人全职
- 硬件：STM32U575开发板 + 3个传感器
- 工具：Keil MDK-ARM + 逻辑分析仪（可选）

### 7.5 下一步行动

**立即开始：**
1. 确认计划细节
2. 准备开发环境
3. 开始阶段1实施

**关键里程碑：**
- Day 4：单传感器DMA验证完成
- Day 7：多传感器集成完成
- Day 10：数据流优化完成
- Day 15：全部测试完成

---

## 八、附录

### 8.1 参考资料

- STM32U575 Reference Manual (RM0456)
- STM32U5 GPDMA Application Note (AN5593)
- LSM6DSOX Datasheet
- H3LIS100DL Datasheet
- QMA6100P Datasheet
- FreeRTOS Documentation

### 8.2 相关文档

- 阶段A工作总结：`plan_doc/1/阶段A_USBX枚举_工作总结.md`
- 阶段B工作总结：`plan_doc/2/阶段B_基础命令通道_工作总结.md`
- 阶段C工作总结：`plan_doc/2/阶段C_USB上传任务_工作总结.md`

### 8.3 联系方式

- 项目负责人：[待填写]
- 技术支持：[待填写]
- 紧急联系：[待填写]

---

**文档版本：** v1.0
**编制日期：** 2026-05-05
**编制人：** Claude Sonnet 4.6
**审核状态：** 待审核

---

**批准签字：**

项目负责人：____________  日期：______

技术负责人：____________  日期：______
