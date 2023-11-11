#ifndef __MAILBOX_H__
#define __MAILBOX_H__

#include<stdint.h>
#include "stm32f4xx.h"
#include "osKernel.h"

void MB_task0();
void MB_task1();
void MB_task2();

#endif
