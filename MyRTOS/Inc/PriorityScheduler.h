#ifndef __PRIORITY_SCHEDULER_H__
#define __PRIORITY_SCHEDULER_H__
#include "osKernel.h"
#include "config.h"
uint8_t osKernelAddThread(void(*Task0)(void),uint32_t p0,
                          void(*Task1)(void),uint32_t p1,
                          void(*Task2)(void),uint32_t p2,
                          void(*Task3)(void),uint32_t p3,
                          void(*Task4)(void),uint32_t p4,
                          void(*Task5)(void),uint32_t p5,
                          void(*Task6)(void),uint32_t p6,
                          void(*Task7)(void),uint32_t p7);

void prioritySchedulerTask0();
void prioritySchedulerTask1();
void prioritySchedulerTask2();
void prioritySchedulerTask3();
void prioritySchedulerTask4();
void prioritySchedulerTask5();
void prioritySchedulerTask6();
void prioritySchedulerTask7();

//void osPriorityScheduler();

#endif
