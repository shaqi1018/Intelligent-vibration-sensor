#include "user_ctrl.h"
#include "board_io.h"
#include "app_acq.h"
#include "battery_adc.h"
#include "boot_mode.h"
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

static void Uc_CheckUsbToMsc(void)
{
  if (s_msc_triggered != 0U) { return; }       /* 本次开机已触发过,不重入 */
  if (UsbDet_IsPresent() == 0U) { return; }    /* USB 未插,不动作 */

  /* 识别到 USB 接入 → 进 MSC */
  s_msc_triggered = 1U;
  if (AppAcqIsRunning() != 0U)
  {
    AppAcqStop();
    /* 等采集真正收尾(logger 关闭并 flush 文件),最多 ~3s */
    for (uint32_t i = 0U; i < 150U; i++)
    {
      if (AppAcqIsRunning() == 0U) { break; }
      osDelay(20U);
    }
    osDelay(300U);  /* 余量:确保 f_close/f_sync 落盘 */
  }
  LED_Set(1U);                        /* 灭灯 */
  BootMode_Write(BOOT_MODE_USB_MSC);  /* 写 TAMP 标志 */
  NVIC_SystemReset();                 /* 复位 → 开机枚举为 U 盘(MSC) */
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
          /* 长按 3s → 关机 */
          s_pwr_state = UC_BTN_HANDLED;
          if (AppAcqIsRunning() != 0U)
          {
            AppAcqStop();
            osDelay(1200U);
          }
          LED_Set(0U);
          if (BoardIO_IsBatteryLatched())
          {
            /* 电池供电：断电锁存，设备真正关机 */
            PowerCtl_Set(0U);
            for (;;) { __WFI(); }
          }
          else
          {
            /* USB 供电：无法断开 USB，重启 MCU 让 USB 重新枚举 */
            NVIC_SystemReset();
          }
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
