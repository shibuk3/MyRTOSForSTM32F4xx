#ifndef __TIMER_H__
#define __TIMER_H__
#include "stm32f4xx.h"

#define TIMER2_EN     (1<<0)
#define CR1_EN        (1<<0)
#define DIER_UIE      (1<<0)
#define SR_UIF        (1<<0)
void time2_1Hz_Interrupt_Init();

#endif
