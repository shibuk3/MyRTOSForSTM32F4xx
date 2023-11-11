#include "PriorityScheduler.h"

extern tcbType *currentPt;

extern tcbType tcbs[NUM_OF_THREADS];
extern uint32_t TCB_Stacks[NUM_OF_THREADS][STACK_SIZE];

#if (SCHEDULER == PRIORITY_SCHEDULER)
uint8_t osKernelAddThread(void(*Task0)(void),uint32_t p0,
                          void(*Task1)(void),uint32_t p1,
                          void(*Task2)(void),uint32_t p2,
                          void(*Task3)(void),uint32_t p3,
                          void(*Task4)(void),uint32_t p4,
                          void(*Task5)(void),uint32_t p5,
                          void(*Task6)(void),uint32_t p6,
                          void(*Task7)(void),uint32_t p7)
{
  __disable_irq();
  tcbs[0].nextPt=&tcbs[1];
  tcbs[1].nextPt=&tcbs[2];
  tcbs[2].nextPt=&tcbs[3];
  tcbs[3].nextPt=&tcbs[4];
  tcbs[4].nextPt=&tcbs[5];
  tcbs[5].nextPt=&tcbs[6];
  tcbs[6].nextPt=&tcbs[7];
  tcbs[7].nextPt=&tcbs[0];

  osKernelStackInit(0);
  TCB_Stacks[0][STACK_SIZE-2]=(uint32_t)(Task0);
  osKernelStackInit(1);
  TCB_Stacks[1][STACK_SIZE-2]=(uint32_t)(Task1);
  osKernelStackInit(2);
  TCB_Stacks[2][STACK_SIZE-2]=(uint32_t)(Task2);
  osKernelStackInit(3);
  TCB_Stacks[3][STACK_SIZE-2]=(uint32_t)(Task3);
  osKernelStackInit(4);
  TCB_Stacks[4][STACK_SIZE-2]=(uint32_t)(Task4);
  osKernelStackInit(5);
  TCB_Stacks[5][STACK_SIZE-2]=(uint32_t)(Task5);
  osKernelStackInit(6);
  TCB_Stacks[6][STACK_SIZE-2]=(uint32_t)(Task6);
  osKernelStackInit(7);
  TCB_Stacks[7][STACK_SIZE-2]=(uint32_t)(Task7);

  currentPt = &tcbs[0];


  for(uint8_t i=0;i<NUM_OF_THREADS;i++)
  {
    tcbs[i].blocked=0;
    tcbs[i].sleepTime=0;
  }

  tcbs[0].priority=p0;
  tcbs[1].priority=p1;
  tcbs[2].priority=p2;
  tcbs[3].priority=p3;
  tcbs[4].priority=p4;
  tcbs[5].priority=p5;
  tcbs[6].priority=p6;
  tcbs[7].priority=p7;

  __enable_irq();
  return 1;
}

#endif
//void osPriorityScheduler()
//{
//  tcbType * _currentPt=currentPt;
//  tcbType * nextThreadToRun = currentPt;
//  uint8_t highestPriorityFound=255;
//  do {
//    _currentPt=_currentPt->nextPt;
//    if((_currentPt->priority < highestPriorityFound)
//      &&(_currentPt->sleepTime==0) && (_currentPt->blocked ==0))
//      {
//        nextThreadToRun=_currentPt;
//        highestPriorityFound=_currentPt->priority;
//      }
//  } while(_currentPt!=currentPt);
//  currentPt=nextThreadToRun;
//}


uint32_t PriorityTaskCounter0,
         PriorityTaskCounter1,
         PriorityTaskCounter2,
         PriorityTaskCounter3,
         PriorityTaskCounter4,
         PriorityTaskCounter5,
         PriorityTaskCounter6,
         PriorityTaskCounter7;

void prioritySchedulerTask0()
{
  while(1)
  {
    PriorityTaskCounter0++;
  }
}

void prioritySchedulerTask1()
{
  while(1)
  {
    PriorityTaskCounter1++;
    // if(PriorityTaskCounter1>60000)
    // {
    //   osThreadSleep(1000);
    // }
  }
}

void prioritySchedulerTask2()
{
  while(1)
  {
    PriorityTaskCounter2++;
  }
}

void prioritySchedulerTask3()
{
  while(1)
  {
    PriorityTaskCounter3++;
  }
}

void prioritySchedulerTask4()
{
  while(1)
  {
    PriorityTaskCounter4++;
  }
}

void prioritySchedulerTask5()
{
  while(1)
  {
    PriorityTaskCounter5++;
  }
}

void prioritySchedulerTask6()
{
  while(1)
  {
    PriorityTaskCounter6++;
  }
}

void prioritySchedulerTask7()
{
  while(1)
  {
    PriorityTaskCounter7++;
  }
}

//int main()
//{
//  //init
//	  osKernelAddThread(&prioritySchedulerTask0,5,&prioritySchedulerTask1,1,
//			  	  	  	&prioritySchedulerTask2,1,&prioritySchedulerTask3,2,
//	                    &prioritySchedulerTask4,5,&prioritySchedulerTask5,1,
//						&prioritySchedulerTask,1,&prioritySchedulerTask7,3);
//}
