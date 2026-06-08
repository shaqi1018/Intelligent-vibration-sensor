#include "user_ctrl.h"
#include "board_io.h"
#include "app_acq.h"
#include "cmsis_os2.h"

#define UC_USER_ACT_MS  2000U  /* 用户键按住 2s → 切换采集开/停 */
#define UC_PWR_OFF_MS   3000U  /* 电源键按住 3s → 关机 */
#define UC_LED_MIN_MS     50U
#define UC_LED_MAX_MS    150U
#define UC_POLL_MS        20U
/* Require this many consecutive non-pressed polls to reset the 3s power-off
 * timer — prevents mechanical button bounce from restarting the countdown. */
#define UC_PWR_RELEASE_DEBOUNCE  3U

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

void UserCtrl_Init(void) { /* no RTOS objects needed */ }

void StartUserCtrlTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

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

    /* ── 电源按键：按住 3s 关机 ── */
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
      /* Require UC_PWR_RELEASE_DEBOUNCE consecutive non-pressed polls before
       * resetting state — one bounce (20ms) won't restart the 3s counter. */
      if (s_pwr_state != UC_BTN_IDLE)
      {
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

    /* ── LED 闪烁（采集中随机间隔） ── */
    if (AppAcqIsRunning() != 0U)
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
      LED_Set(0U);
    }

    osDelay(UC_POLL_MS);
  }
}
