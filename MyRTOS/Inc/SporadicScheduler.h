#ifndef __SPORADIC_SCHEDULER_H__
#define __SPORADIC_SCHEDULER_H__

#include "stm32f4xx.h"
#include "osKernel.h"
#include "led.h"
#include "config.h"

void osEdgeTriggerInit(int32_t *semaphore);
void sporadicTask();
void NormalTask1();
void NormalTask2();

#endif
