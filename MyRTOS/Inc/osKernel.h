#ifndef __OSKERNEL_H__
#define __OSKERNEL_H__

#include "stm32f4xx.h"
#include "config.h"

#define STACK_SIZE      100

#define CTRL_ENABLE    (1<<0)
#define CTRL_TICKINT   (1<<1)
#define CTRL_CLCKSRC   (1<<2)
#define CTRL_COUNTFLAG (1<<16)
#define BUS_FREQ       (16000000)
#define SYSTICK_RST    0
#define QUANTA         10
#define ICSR           (*((volatile uint32_t *)0xE000ED04))
#define PENDSTSET      (1<<26)

#define PERIOD          100

#if (SCHEDULER == PRIORITY_SCHEDULER)
#define NUM_OF_THREADS 8
struct tcb
{
  uint32_t * stackPt;
  struct tcb * nextPt;
  uint32_t sleepTime;
  uint32_t blocked;
  uint32_t priority;
};

#else
#define NUM_OF_THREADS     3
struct tcb
{
  uint32_t * stackPt;
  struct tcb * nextPt;
};
#endif
typedef struct tcb tcbType;

void osKernelStackInit(uint32_t i);
uint8_t osKernelAddThreads(void(*task0)(void),void(*task1)(void),void(*task2)(void));
void osKernelLaunch(uint32_t quanta);
void osSchedulerLaunch();
void osKernelInit();
void osThreadYield();
#if(SCHEDULER == PERIODIC_SCHEDULER)
void osPeriodicScheduler();
void task3();
#endif

void osSemaphoreInit(int32_t *semaphore , int32_t value);
void osSemaphoreGive(int32_t *semaphore);
void osSemaphoreWait(int32_t *semaphore);

#endif
