/**
  ******************************************************************************
  * @file    sai.c
  * @brief   SAI1 Block A master-RX (I2S 16-bit) + circular GPDMA + 12.288 MHz
  *          MCLK clock tree for the ES8311 microphone codec.
  *
  * Hand-written in the project's CubeMX-free style (mirrors i2c.c / dma_sampling.c)
  * because this project never regenerates from CubeMX (see memory eide_builder_params).
  *
  * Clock tree (target SAI1 kernel clock = 49.152 MHz):
  *   MSI = 48 MHz -> PLL2 (src=MSI)
  *     PLL2M = 4   -> VCI = 12 MHz   (RCC_PLLVCIRANGE_1, 8..16 MHz)
  *     PLL2N = 32, PLL2FRACN = 6291  -> VCO = 12 * (32 + 6291/8192) = 393.2153 MHz
  *     PLL2P = 8   -> PLL2CLK = 49.1519 MHz  (target 49.152 MHz, ~21 ppm error)
  *   SAI1 kernel clock = PLL2P = 49.152 MHz.
  *
  * The SAI master clock generator then divides the 49.152 MHz kernel by MCKDIV
  * (computed by HAL from AudioFrequency) to output MCLK = 256 x Fs:
  *   Fs = 96000 -> MCKDIV = 2  -> MCLK = 24.576 MHz  (double speed; ES8311 fs_mode=1)
  *   Fs = 48000 -> MCKDIV = 4  -> MCLK = 12.288 MHz   (the design's nominal MCLK)
  *   Fs = 16000 -> MCKDIV = 12 -> MCLK = 4.096 MHz
  *   Fs = 8000  -> MCKDIV = 24 -> MCLK = 2.048 MHz
  * BCLK = 32 x Fs (I2S 16-bit x 2 slots), LRCK = Fs.
  *
  * IMPORTANT design caveat: the SAI cannot output a FIXED 12.288 MHz MCLK while
  * LRCK tracks 8k/16k/48k from one block (MCLK and the frame clock share MCKDIV,
  * MCLK is locked to 256xFs). So MCLK == 12.288 MHz holds only at Fs = 48000.
  * The ES8311 codec table (Task1) must therefore be indexed by the actual
  * MCLK/Fs ratio (= 256 for all rates here), or the mic should run at 48 kHz.
  ******************************************************************************
  */

#include "sai.h"

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef handle_GPDMA1_Channel_SAI;

/* Circular linked-list descriptor for the SAI RX GPDMA channel. */
static DMA_NodeTypeDef  s_sai_dma_node;
static DMA_QListTypeDef s_sai_dma_queue;

/* Map a sample rate to the SAI AudioFrequency enum. */
static uint32_t sai_audio_freq(uint32_t sr)
{
  switch (sr)
  {
    case 8000U:  return SAI_AUDIO_FREQUENCY_8K;
    case 16000U: return SAI_AUDIO_FREQUENCY_16K;
    case 48000U: return SAI_AUDIO_FREQUENCY_48K;
    case 96000U: return SAI_AUDIO_FREQUENCY_96K;   /* 双速：MCLK=24.576MHz, MCKDIV=2 */
    default:     return SAI_AUDIO_FREQUENCY_16K;
  }
}

/* Configure PLL2 to feed the SAI1 kernel clock with ~49.152 MHz. */
static HAL_StatusTypeDef sai_clock_config(void)
{
  RCC_PeriphCLKInitTypeDef pclk = {0};

  pclk.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  pclk.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PLL2;   /* SAI1 clk = PLL2 "P" */

  pclk.PLL2.PLL2Source   = RCC_PLLSOURCE_MSI;           /* MSI = 48 MHz */
  pclk.PLL2.PLL2M        = 4U;                           /* VCI = 12 MHz */
  pclk.PLL2.PLL2N        = 32U;                          /* VCO base */
  pclk.PLL2.PLL2FRACN    = 6291U;                        /* fractional part */
  pclk.PLL2.PLL2P        = 8U;                           /* PLL2P = 49.152 MHz */
  pclk.PLL2.PLL2Q        = 2U;                           /* unused, keep legal */
  pclk.PLL2.PLL2R        = 2U;                           /* unused, keep legal */
  pclk.PLL2.PLL2RGE      = RCC_PLLVCIRANGE_1;            /* VCI 8..16 MHz */
  pclk.PLL2.PLL2ClockOut = RCC_PLL2_DIVP;               /* enable PLL2P output */

  return HAL_RCCEx_PeriphCLKConfig(&pclk);
}

/* HAL MspInit: enable clocks, configure the 4 SAI1 pins, build the circular
 * GPDMA channel and link it to the SAI RX handle. Called from HAL_SAI_Init(). */
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance != SAI1_Block_A)
  {
    return;
  }

  GPIO_InitTypeDef g = {0};

  __HAL_RCC_SAI1_CLK_ENABLE();
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* PB8 = SAI1_MCLK_A, PB9 = SAI1_FS_A  (AF13 —— 经 CubeMX 生成参考核对，
   * U5 SAI1_A 这组引脚统一为 AF13；此前误设 AF3 导致 MCLK/LRCK 不出引脚，
   * ES8311 收不到主时钟 → ADC 不工作 → SDOUT 恒 0 → 录音全静音) */
  g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOB, &g);

  /* PA8 = SAI1_SCK_A  (AF13) */
  g.Pin       = GPIO_PIN_8;
  g.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOA, &g);

  /* PC3 = SAI1_SD_A  (AF13) */
  g.Pin       = GPIO_PIN_3;
  g.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOC, &g);

  /* ---- Circular GPDMA: SAI1_A RX FIFO -> memory, 16-bit, GPDMA1 Channel 4 ----
   * Channels 0..3 are used by SPI1/SPI2 (dma_sampling.c); use Channel 4. */
  DMA_NodeConfTypeDef node = {0};
  node.NodeType                        = DMA_GPDMA_LINEAR_NODE;
  node.Init.Request                    = GPDMA1_REQUEST_SAI1_A;
  node.Init.BlkHWRequest               = DMA_BREQ_SINGLE_BURST;
  node.Init.Direction                  = DMA_PERIPH_TO_MEMORY;
  node.Init.SrcInc                     = DMA_SINC_FIXED;
  node.Init.DestInc                    = DMA_DINC_INCREMENTED;
  node.Init.SrcDataWidth               = DMA_SRC_DATAWIDTH_HALFWORD;
  node.Init.DestDataWidth              = DMA_DEST_DATAWIDTH_HALFWORD;
  node.Init.SrcBurstLength             = 1U;
  node.Init.DestBurstLength            = 1U;
  node.Init.TransferAllocatedPort      = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  node.Init.TransferEventMode          = DMA_TCEM_BLOCK_TRANSFER;
  node.Init.Mode                       = DMA_NORMAL;
  node.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  node.DataHandlingConfig.DataAlignment= DMA_DATA_RIGHTALIGN_ZEROPADDED;
  node.TriggerConfig.TriggerPolarity   = DMA_TRIG_POLARITY_MASKED;
  /* Source/dest addresses and size are filled in by HAL_SAI_Receive_DMA(). */
  node.SrcAddress                      = 0U;
  node.DstAddress                      = 0U;
  node.DataSize                        = 0U;

  (void)HAL_DMAEx_List_ResetQ(&s_sai_dma_queue);
  (void)HAL_DMAEx_List_BuildNode(&node, &s_sai_dma_node);
  (void)HAL_DMAEx_List_InsertNode_Tail(&s_sai_dma_queue, &s_sai_dma_node);
  (void)HAL_DMAEx_List_SetCircularModeConfig(&s_sai_dma_queue, &s_sai_dma_node);

  handle_GPDMA1_Channel_SAI.Instance                         = GPDMA1_Channel4;
  handle_GPDMA1_Channel_SAI.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
  handle_GPDMA1_Channel_SAI.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  handle_GPDMA1_Channel_SAI.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  handle_GPDMA1_Channel_SAI.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  handle_GPDMA1_Channel_SAI.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel_SAI) != HAL_OK)
  {
    return;
  }
  if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel_SAI, &s_sai_dma_queue) != HAL_OK)
  {
    return;
  }

  __HAL_LINKDMA(hsai, hdmarx, handle_GPDMA1_Channel_SAI);
  (void)HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel_SAI, DMA_CHANNEL_NPRIV);

  /* Priority 6: below SDMMC, leaves headroom for higher-priority IRQs. */
  HAL_NVIC_SetPriority(GPDMA1_Channel4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);

  /* SAI1 peripheral IRQ: services SAI-side faults (FIFO over/underrun OVRUDR,
   * wrong-clock config) so they reach HAL_SAI_ErrorCallback. The GPDMA channel
   * IRQ above only catches DMA bus errors — without this, a SAI FIFO overrun
   * would silently stall capture (L5). Same priority band as the SAI DMA
   * channel; SAI1_IRQHandler only flags, it calls no RTOS API. */
  HAL_NVIC_SetPriority(SAI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SAI1_IRQn);
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance != SAI1_Block_A)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(GPDMA1_Channel4_IRQn);
  HAL_NVIC_DisableIRQ(SAI1_IRQn);
  if (hsai->hdmarx != NULL)
  {
    (void)HAL_DMA_DeInit(hsai->hdmarx);
  }
  (void)HAL_DMAEx_List_ResetQ(&s_sai_dma_queue);

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);

  __HAL_RCC_SAI1_CLK_DISABLE();
}

HAL_StatusTypeDef MX_SAI1_Init(uint32_t sample_rate_hz)
{
  /* 1) SAI1 kernel clock = ~49.152 MHz via PLL2. */
  if (sai_clock_config() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 2) SAI Block A: master receiver, MCLK output enabled, I2S 16-bit.
   *    HAL_SAI_InitProtocol() fills FrameInit/SlotInit (32-bit frame, two
   *    16-bit slots) and calls HAL_SAI_MspInit() for GPIO + DMA. */
  hsai_BlockA1.Instance             = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode       = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.Synchro         = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive     = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider       = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold   = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA1.Init.AudioFrequency  = sai_audio_freq(sample_rate_hz);
  hsai_BlockA1.Init.SynchroExt      = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MckOutput       = SAI_MCK_OUTPUT_ENABLE;
  hsai_BlockA1.Init.MonoStereoMode  = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode  = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState        = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  /* ADC drives ASDOUT on the SCLK falling edge -> SAI latches on falling. */
  hsai_BlockA1.Init.ClockStrobing   = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.FirstBit        = SAI_FIRSTBIT_MSB;

  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD,
                           SAI_PROTOCOL_DATASIZE_16BIT, 2U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void MX_SAI1_DeInit(void)
{
  (void)HAL_SAI_DeInit(&hsai_BlockA1);
}
