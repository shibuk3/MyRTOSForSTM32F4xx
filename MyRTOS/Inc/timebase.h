#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__
#include<stdint.h>

#define CTRL_ENABLE    (1<<0)
#define CTRL_TICKINT   (1<<1)
#define CTRL_CLCKSRC   (1<<2)
#define CTRL_COUNTFLAG (1<<16)
#define FREQUENCY      (16000000)
#define MAX_DELAY      (0xFFFFFFFF)

void timerbase_init();
uint32_t get_tick();
void delay(uint32_t delay);
void tick_inreament();


#endif
