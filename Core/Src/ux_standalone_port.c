#include "main.h"

#include "ux_api.h"

unsigned int tx_interrupt_control(unsigned int new_posture)
{
  unsigned int previous_posture = __get_PRIMASK();

  if (new_posture != 0U)
  {
    __disable_irq();
  }
  else
  {
    __enable_irq();
  }

  return previous_posture;
}

ALIGN_TYPE _ux_utility_interrupt_disable(VOID)
{
  return (ALIGN_TYPE)tx_interrupt_control(TX_INT_DISABLE);
}

VOID _ux_utility_interrupt_restore(ALIGN_TYPE flags)
{
  tx_interrupt_control((unsigned int)flags);
}