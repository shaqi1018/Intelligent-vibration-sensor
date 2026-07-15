#include "user_ctrl.h"
#include "board_io.h"
#include "app_acq.h"
#include "battery_adc.h"
#include "boot_mode.h"
#include "fatfs_sd.h"   /* FatFs_SD_IsLoggerActive — 等文件真正关好再断电/复位 */
#include "cmsis_os2.h"

/* 外部声明系统异常标志(定义在 app_freertos.c) */
extern volatile uint8_t g_system_error;

#define UC_USER_ACT_MS  2000U  /* 用户键按住 2s → 切换采集开/停 */
#define UC_PWR_OFF_MS   3000U  /* 电源键按住 3s → 关机 */
#define UC_LED_MIN_MS     50U
#define UC_LED_MAX_MS    150U
#define UC_POLL_MS        20U
/* Require this many consecutive non-pressed polls to reset the 3s power-off
 * timer — prevents mechanical button bounce from restarting the countdown. */
#define UC_PWR_RELEASE_DEBOUNCE  3U

/* 低电量保护:电压持续低于阈值 → 优雅关机(停采→关文件+f_sync→断电),避免电池
 * 保护板/稳压器在写卡途中硬切导致 SD 文件系统损坏(续航测试实测问题)。阈值写死
 * 3.4V(高于常见硬切点 2.4~3.0V,留足关文件时间);每 UC_BAT_CHECK_MS 读一次电压,
 * 连续 UC_BAT_LOW_CONFIRM 次落在计数带才触发(去抖:满载写卡瞬间电压会下陷,防误触发
 * 中断有效录制)。容量无关——按带载电压触发,400mAh/2000mAh 通用。 */
#define UC_BAT_LOW_MV        3400U   /* 低电量带上沿:带载持续低于此 → 优雅关机 */
#define UC_BAT_CHECK_MS      500U    /* 读电压间隔(原 2000→500,触发提速 4×) */
#define UC_BAT_LOW_CONFIRM   3U      /* 低带确认次数:3×500ms ≈ 1.5s(原 4×2000≈8s) */
/* ★ 临界带(B2,实测 96kHz 满配掉电漏关机的补强):真电池垂死时带载电压可能【跳崖】
 * 式下滑,1.5s 的低带确认窗口可能来不及在硬切点前关好文件。低于 UC_BAT_CRIT_MV 时改用
 * 更短的确认次数(≈1.0s),抢在断电前完成优雅关机。用短确认(而非纯单次)是为了不被
 * 满载写卡瞬间的电压下陷误触发中断有效录制——1s 持续 <3.0V 已是真正的供电垂死,非瞬时。 */
#define UC_BAT_CRIT_MV       3000U   /* 临界带上沿:带载低于此 → 短确认即关机 */
#define UC_BAT_CRIT_CONFIRM  2U      /* 临界带确认次数:2×500ms ≈ 1.0s */
/* ADC 故障地板:读数低于此值才视为 ADC 读失败(返回≈0/异常),而非真实低电量。
 * ★ 关键(实测 CTBX_2026-07-04-17-22 满配 96kHz 掉电未优雅关机的根因修复):
 * 旧值 2500 太高——真电池垂死时带载电压是【一路向下穿过 2500】掉到硬切点的,一旦
 * 读到 <2500 旧逻辑就把它当"无效"清零去抖计数,恰在最该关机时把进度丢掉 → 优雅关机
 * 永不触发 → 硬切损坏。改用远低于任何真实运行电压(系统在保护板硬切点 ~2.4-3.0V 以下
 * 就断电了)的 1500mV 做地板:计数带拓宽为 [1500, 3400),整个末段下滑区都持续计数,
 * 只有 ≈0 的真故障读数才清零。 */
#define UC_BAT_ADC_FAULT_MV  1500U

/* 电量显示：短按电源键后的 LED 快闪参数 */
#define UC_BAT_FLASH_ON_MS    80U   /* 每次闪亮持续时间 */
#define UC_BAT_FLASH_OFF_MS   120U  /* 每次闪灭间隔时间 */

static uint32_t s_rng = 0x12345678U;

static uint32_t xorshift32(void)
{
  s_rng ^= s_rng << 13;
  s_rng ^= s_rng >> 17;
  s_rng ^= s_rng << 5;
  return s_rng;
}

typedef enum { UC_BTN_IDLE = 0, UC_BTN_PRESSED, UC_BTN_HANDLED } BtnState_t;

static BtnState_t s_user_state     = UC_BTN_IDLE;
static BtnState_t s_pwr_state      = UC_BTN_IDLE;
static uint32_t   s_user_tick      = 0U;
static uint32_t   s_pwr_tick       = 0U;
static uint32_t   s_led_next       = 0U;
static uint8_t    s_pwr_release_ct = 0U; /* release debounce counter */
static uint32_t   s_bat_next_check = 0U; /* 下次电池电压检查的 tick */
static uint8_t    s_bat_low_ct     = 0U; /* 连续低于阈值计数(去抖) */

/* 根据电量百分比决定闪烁次数
 * ≥80% → 5次, ≥60% → 4次, ≥40% → 3次, ≥20% → 2次, <20% → 1次 */
static uint8_t BatPercent_ToFlashCount(uint8_t pct)
{
  if (pct >= 80U) { return 5U; }
  if (pct >= 60U) { return 4U; }
  if (pct >= 40U) { return 3U; }
  if (pct >= 20U) { return 2U; }
  return 1U;
}

/* LED 快闪 n 次（阻塞，仅在触发电量显示时调用，持续约 1~1.5s）*/
static void BatLed_Flash(uint8_t count)
{
  LED_Set(1U);  /* 确保从灭态开始 */
  osDelay(200U);

  for (uint8_t i = 0U; i < count; i++)
  {
    LED_Set(0U);                    /* 亮 (active-low) */
    osDelay(UC_BAT_FLASH_ON_MS);
    LED_Set(1U);                    /* 灭 */
    if (i < count - 1U)
    {
      osDelay(UC_BAT_FLASH_OFF_MS); /* 最后一闪后不再等待 */
    }
  }
}

/* path/sd: 只要识别到 USB 接入就进 MSC(把 SD 卡当 U 盘给电脑读录制文件)。
 * 触发条件:任何时刻发现 PC7 HIGH(USB 已插) 即触发 —— 包括"USB 上电开机即 HIGH"
 *   和"电池运行中插入 USB"两种场景,本次开机只触发一次(s_msc_triggered)。
 *   防 normal↔MSC 复位死循环:MSC 因无卡/枚举失败弹回时,main.c 在进 RTOS 前就直接
 *   复位回 DATA_LOG(不会运行到本任务);正常 USB 拔出弹回时 PC7 已 LOW,不会重触发。
 * 进 MSC 前先优雅停采、等 logger 关文件 flush,保证刚录的数据完整(用户要求)。
 * 逻辑自包含,不改 fatfs_sd.c 等共享文件(保 cherry-pick)。 */
static uint8_t s_msc_triggered = 0U;

/* 等 logger 真正把文件关好落盘。关键:AppAcqStop() 会让 AppAcqIsRunning() 立即变 0,
 * 但文件是 logger 任务【随后异步】关的(finalize WAV 头 + 对全部文件 f_sync/f_close),
 * 完成时才清 g_logger_active。所以断电/复位前必须等 FatFs_SD_IsLoggerActive()==0,
 * 否则会在写文件系统途中掉电 → WAV 头没回填、CSV 末行截断(实测 CTBX_..19-24 损坏根因)。
 * 停采后传感器不再产数,logger 只需排空少量积压+关文件,通常 1~3s;最多等 ~5s 兜底。 */
static void Uc_WaitLoggerClosed(void)
{
  for (uint32_t i = 0U; i < 250U; i++)
  {
    if (FatFs_SD_IsLoggerActive() == 0U) { break; }
    osDelay(20U);
  }
  osDelay(100U);  /* 小余量,确保最后一次 f_sync 落盘 */
}

static void Uc_CheckUsbToMsc(void)
{
  if (s_msc_triggered != 0U) { return; }       /* 本次开机已触发过,不重入 */
  if (UsbDet_IsPresent() == 0U) { return; }    /* USB 未插,不动作 */

  /* 识别到 USB 接入 → 进 MSC */
  s_msc_triggered = 1U;
  if (AppAcqIsRunning() != 0U)
  {
    AppAcqStop();
    Uc_WaitLoggerClosed();  /* 等文件真正关好再复位进 MSC,保证卡上文件完整 */
  }
  LED_Set(1U);                        /* 灭灯 */
  BootMode_Write(BOOT_MODE_USB_MSC);  /* 写 TAMP 标志 */
  NVIC_SystemReset();                 /* 复位 → 开机枚举为 U 盘(MSC) */
}

/* 优雅关机:停采集 → 轮询等 logger 真正收尾(关闭全部文件 + f_sync 刷 FAT 元数据)
 * → 断电锁存自关机(电池)/复位(USB)。电源键长按 3s 与低电量保护共用此路径,保证
 * SD 文件系统一致后再掉电。轮询等待(而非固定延时)确保文件确实关好才断电。 */
static void Uc_SafeShutdown(void)
{
  /* ★取证:确认 SafeShutdown 真被调用了(区分"低电量完全没触发"vs"触发了但收尾没跑完")。
   * 掉电日志里若有此行=触发了,再看下面 logger closed 结果判断收尾是否完成。 */
  printf("[LowBat] SafeShutdown enter\r\n");
  if (AppAcqIsRunning() != 0U)
  {
    AppAcqStop();
  }
  Uc_WaitLoggerClosed();  /* 等文件真正关好(WAV finalize + f_sync/f_close)再断电 */
  printf("[LowBat] logger closed=%u (0=收尾完成)\r\n",
         (unsigned)FatFs_SD_IsLoggerActive());
  LED_Set(1U);  /* 灭灯(active-low:高=灭) */
  if (BoardIO_IsBatteryLatched())
  {
    /* 电池供电:断电锁存,设备真正关机 */
    PowerCtl_Set(0U);
    for (;;) { __WFI(); }
  }
  else
  {
    /* USB 供电:无法断开 USB,重启 MCU */
    NVIC_SystemReset();
  }
}

void UserCtrl_Init(void) { /* no RTOS objects needed */ }

void StartUserCtrlTask(void *argument)
{
  (void)argument;

  /* 开机即查 USB:若上电时 USB 已插着,立刻进 MSC,不先启动采集(避免无谓开录又
   * 马上停)。Uc_CheckUsbToMsc 内部已防重入,USB 未插则直接返回。 */
  Uc_CheckUsbToMsc();

  /* 开机即采：等系统稳定（传感器/SD 已在 main 初始化、各任务已就绪）后触发一次。
   * boot_acquire=0 时为空操作，行为同既往。MSC 模式不进 RTOS，不会到这里。 */
  osDelay(1000U);
  AppBootAcquireIfConfigured();

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    /* path/sd: USB 插入 → 优雅停采收尾 → 复位进 MSC 读卡(优先于按键逻辑) */
    Uc_CheckUsbToMsc();

    /* ── 低电量保护:带载电压持续偏低 → 优雅关机(存好数据再掉电) ──
     * 仅电池供电时生效(USB 供电不会低电量);每 500ms 读一次电压。两级计数带,共用
     * 同一去抖计数 s_bat_low_ct:
     *   · 低带 [1500, 3400):连续 3 次(≈1.5s)触发——正常低电量下滑;
     *   · 临界带 [1500, 3000):连续 2 次(≈1.0s)即触发——垂死电池跳崖式下滑时抢时间。
     * 用短确认而非纯单次,避免满载写卡瞬时电压下陷误触发中断有效录制。计数带下探到
     * 1500(而非 2500)让垂死电池穿过 2500 继续往下的读数仍持续计数(旧逻辑在此丢触发,
     * 实测 96kHz 掉电损坏)。只有 ≈0 的真故障读数(<1500)才清零。不返回。 */
    if (BoardIO_IsBatteryLatched() && ((int32_t)(now - s_bat_next_check) >= 0))
    {
      s_bat_next_check = now + UC_BAT_CHECK_MS;
      uint32_t mv = BatteryADC_ReadMillivolts();
      if ((mv >= UC_BAT_ADC_FAULT_MV) && (mv < UC_BAT_LOW_MV))
      {
        if (s_bat_low_ct < 255U) { s_bat_low_ct++; }
        /* 临界带用更少的确认次数抢时间;低带用标准确认次数 */
        uint8_t need = (mv < UC_BAT_CRIT_MV) ? UC_BAT_CRIT_CONFIRM : UC_BAT_LOW_CONFIRM;
        /* ★取证(2026-07-09):满配掉电保护第三次漏,加日志定性"没触发"vs"触发太晚"。
         * 每次落带都打印电压/计数/阈值,掉电前的串口尾巴能还原检测过程。精简单行、
         * 尽早刷出(掉电瞬间串口可能来不及,故每次都打而非仅触发时)。 */
        printf("[LowBat] mv=%lu ct=%u/%u\r\n", (unsigned long)mv,
               (unsigned)s_bat_low_ct, (unsigned)need);
        if (s_bat_low_ct >= need)
        {
          Uc_SafeShutdown();  /* 停采→关文件+sync→断电,不返回 */
        }
      }
      else if (mv < UC_BAT_ADC_FAULT_MV)
      {
        /* ADC 真故障(读数<1.5V≈0,非真实运行电压):直接清零。不能把读失败当低电量累积,
         * 否则 ADC 偶发抽风会误触发优雅关机中断有效录制。 */
        if (s_bat_low_ct != 0U)
        {
          printf("[LowBat] fault mv=%lu (ct %u->0)\r\n",
                 (unsigned long)mv, (unsigned)s_bat_low_ct);
        }
        s_bat_low_ct = 0U;
      }
      else
      {
        /* 电压回升(mv>=UC_BAT_LOW_MV):★去抖改【衰减】而非清零(2026-07-10 修复)。
         * 实测(CTBX_..00-22)满配大电流下带载电压在阈值(3400)附近反复抖动(3392~3416),旧逻辑
         * 一次回升就把计数清零 → 计数永远攒不满 → 优雅关机触发全靠运气(第 1-3 次满配掉电就
         * 因此漏触发损坏)。改为每次回升只 ct-- 递减(滞回):抖动时计数缓慢波动不归零,电压真正
         * 持续下滑时才能累积到 need 触发。既滤瞬时下陷误触发,又不被回升打断真实低电量累积。 */
        if (s_bat_low_ct != 0U)
        {
          s_bat_low_ct--;
          printf("[LowBat] rebound mv=%lu (ct->%u)\r\n",
                 (unsigned long)mv, (unsigned)s_bat_low_ct);
        }
      }
    }

    /* ── 用户按键：按住 2s 切换采集开/停 ── */
    uint8_t user_dn = UserBtn_IsPressed();
    if (user_dn)
    {
      if (s_user_state == UC_BTN_IDLE)
      {
        s_user_state = UC_BTN_PRESSED;
        s_user_tick  = now;
      }
      else if (s_user_state == UC_BTN_PRESSED)
      {
        if ((now - s_user_tick) >= UC_USER_ACT_MS)
        {
          if (AppAcqIsRunning() != 0U)
          {
            AppAcqStop();
            LED_Set(0U);
          }
          else
          {
            AppAcqStart(APP_ACQ_SINK_SD, 0U);
          }
          s_user_state = UC_BTN_HANDLED;
        }
      }
    }
    else
    {
      s_user_state = UC_BTN_IDLE;
    }

    /* ── 电源按键：短按显示电量，长按 3s 关机 ──
     *
     * 状态机：
     *   IDLE → 按下 → PRESSED（记录时刻）
     *   PRESSED + 仍按住 >= 3s → 执行关机（HANDLED）
     *   PRESSED + 松开 < 3s   → 显示电量（短按），回到 IDLE
     */
    uint8_t pwr_dn = PwrBtn_IsPressed();
    if (pwr_dn)
    {
      s_pwr_release_ct = 0U;  /* any press sample clears release counter */
      if (s_pwr_state == UC_BTN_IDLE)
      {
        s_pwr_state = UC_BTN_PRESSED;
        s_pwr_tick  = now;
      }
      else if (s_pwr_state == UC_BTN_PRESSED)
      {
        if ((now - s_pwr_tick) >= UC_PWR_OFF_MS)
        {
          /* 长按 3s → 关机(与低电量保护共用同一优雅关机路径) */
          s_pwr_state = UC_BTN_HANDLED;
          Uc_SafeShutdown();  /* 停采→等关文件+sync→断电,不返回 */
        }
      }
    }
    else
    {
      /* 按键松开 */
      if (s_pwr_state == UC_BTN_PRESSED)
      {
        /* 防抖：需要连续几次未按下才确认松开 */
        if (s_pwr_release_ct < UC_PWR_RELEASE_DEBOUNCE)
        {
          s_pwr_release_ct++;
        }
        else
        {
          /* 松开时长按未到 3s → 短按，显示电量 */
          uint8_t pct   = BatteryADC_GetPercentage();
          uint8_t count = BatPercent_ToFlashCount(pct);
          BatLed_Flash(count);

          s_pwr_state      = UC_BTN_IDLE;
          s_pwr_release_ct = 0U;
        }
      }
      else if (s_pwr_state == UC_BTN_HANDLED)
      {
        /* 关机后松开，等待断电 */
        if (s_pwr_release_ct < UC_PWR_RELEASE_DEBOUNCE)
        {
          s_pwr_release_ct++;
        }
        else
        {
          s_pwr_state      = UC_BTN_IDLE;
          s_pwr_release_ct = 0U;
        }
      }
    }

    /* ── LED 闪烁（采集中随机间隔） ──
     * 用 AppCaptureActive 而非 AppAcqIsRunning:麦克风启用时,灯要等 SAI DMA 真正
     * 开录后才闪,这样"灯亮=麦克风和传感器都在采",用户见灯再开口不会被切开头。
     *
     * 最高优先级:系统异常时(SD 写失败/传感器初始化失败)LED 常亮告警,跳过其他逻辑。 */
    if (g_system_error != 0U)
    {
      LED_Set(0U);  /* 常亮(硬件 active-low:低电平=亮) */
    }
    else if (AppCaptureActive() != 0U)
    {
      if ((int32_t)(now - s_led_next) >= 0)
      {
        LED_Toggle();
        uint32_t iv = UC_LED_MIN_MS +
                      (xorshift32() % (UC_LED_MAX_MS - UC_LED_MIN_MS + 1U));
        s_led_next = now + iv;
      }
    }
    else
    {
      LED_Set(1U);  /* 空闲时熄灭(硬件 active-low:高电平=灭) */
    }

    osDelay(UC_POLL_MS);
  }
}
