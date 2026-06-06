#include "user_ctrl.h"
#include "board_io.h"
#include "app_acq.h"
#include "cmsis_os2.h"
#define UC_USER_SHORT_MS    1000U
#define UC_USER_LONG_MS     2000U
#define UC_PWR_LONG_MS      3000U
#define UC_LED_MIN_MS         50U
#define UC_LED_MAX_MS        150U
#define UC_POLL_MS            20U

static uint32_t s_rng = 0x12345678U; /* XorShift32 state — local to this task */

static uint32_t xorshift32(void)
{
  s_rng ^= s_rng << 13;
  s_rng ^= s_rng >> 17;
  s_rng ^= s_rng << 5;
  return s_rng;
}

typedef enum { UC_BTN_IDLE = 0, UC_BTN_PRESSED, UC_BTN_HANDLED } BtnState_t;

static BtnState_t s_user_state = UC_BTN_IDLE;
static BtnState_t s_pwr_state  = UC_BTN_IDLE;
static uint32_t   s_user_tick  = 0U;
static uint32_t   s_pwr_tick   = 0U;
static uint32_t   s_led_next   = 0U;

void UserCtrl_Init(void) { /* no RTOS objects needed */ }

void StartUserCtrlTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    /* ── 用户按键 ── */
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
        uint32_t held = now - s_user_tick;
        if ((held >= UC_USER_LONG_MS) && (AppAcqIsRunning() != 0U))
        {
          AppAcqStop();
          LED_Set(0U);
          s_user_state = UC_BTN_HANDLED;
        }
        else if ((held >= UC_USER_SHORT_MS) && (AppAcqIsRunning() == 0U))
        {
          AppAcqStart(APP_ACQ_SINK_SD, 0U);
          s_user_state = UC_BTN_HANDLED;
        }
      }
    }
    else
    {
      s_user_state = UC_BTN_IDLE;
    }

    /* ── 电源按键 ── */
    uint8_t pwr_dn = PwrBtn_IsPressed();
    if (pwr_dn)
    {
      if (s_pwr_state == UC_BTN_IDLE)
      {
        s_pwr_state = UC_BTN_PRESSED;
        s_pwr_tick  = now;
      }
      else if (s_pwr_state == UC_BTN_PRESSED)
      {
        if ((now - s_pwr_tick) >= UC_PWR_LONG_MS)
        {
          if (AppAcqIsRunning() != 0U)
          {
            AppAcqStop();
            osDelay(1200U);
          }
          LED_Set(0U);
          PowerCtl_Set(0U);
          for (;;) { __WFI(); }  /* supply gone; never reached in practice */
        }
      }
    }
    else
    {
      s_pwr_state = UC_BTN_IDLE;
    }

    /* ── LED 闪烁 ── */
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
