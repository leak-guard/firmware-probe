#include "soft_timer.h"

volatile uint32_t sys_tick = 0;

void SoftTimer_Init(void)
{
  HAL_TIM_Base_Start_IT(&htim2);
}

void SoftTimer_Start(SoftTimer* timer, uint32_t delay_ms)
{
  timer->start_tick = sys_tick;
  timer->delay = delay_ms;
  timer->active = true;
}

bool SoftTimer_Expired(SoftTimer* timer)
{
  if (!timer->active) return false;

  if ((sys_tick - timer->start_tick) >= timer->delay) {
    timer->active = false;
    return true;
  }
  return false;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2) {
    sys_tick++;
  }
}
