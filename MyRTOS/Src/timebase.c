#include "timebase.h"
#include "stm32f4xx.h"

volatile uint32_t g_curr_tick;
volatile uint32_t g_curr_tick_p;
volatile uint32_t tick_frequency =1;

void timerbase_init()
{
  /*reload the timer with number of cycles per second*/
  SysTick->LOAD = FREQUENCY-1;
  /*clear SysTick current value register*/
  SysTick->VAL=0;
  /*select the internal clock source*/
  SysTick->CTRL=CTRL_CLCKSRC;
  /*enable interrupt*/
  SysTick->CTRL |= CTRL_TICKINT;
  /*enable SysTick*/
  SysTick->CTRL |= CTRL_ENABLE;
  /*enable the global interrupt*/
  __enable_irq();
}

uint32_t get_tick()
{
  __disable_irq();
  g_curr_tick_p = g_curr_tick;
  __enable_irq();
  return g_curr_tick_p;
}
void delay(uint32_t delay)
{
  uint32_t start_tick=get_tick();
  uint32_t wait =delay;
  if(wait<MAX_DELAY)
  {
    wait += (uint32_t)tick_frequency;
  }
  while(get_tick()-start_tick<wait){};
}

void tick_inreament()
{
  g_curr_tick+=tick_frequency;
}


//void SysTick_Handler()
//{
//	tick_inreament();
//}
