#include "osKernel.h"


tcbType *currentPt;

tcbType tcbs[NUM_OF_THREADS];
uint32_t TCB_Stacks[NUM_OF_THREADS][STACK_SIZE];

uint32_t MILLIS_PRESCALER;

void osKernelStackInit(uint32_t i)
{
  tcbs[i].stackPt=&TCB_Stacks[i][STACK_SIZE-16];    /*stack pointer*/
  /*Set bit 21(T bit) to 1 in PSR to operate in thumb mode*/
  TCB_Stacks[i][STACK_SIZE-1] = (1<<24);            /*PSR*/

/*We dont have thread function yet so we can initialise PC*/

  /*Block below is option, only for degbuggin purpose*/
  TCB_Stacks[i][STACK_SIZE-3] =0xAAAAAAAA;  /*LR*/
  TCB_Stacks[i][STACK_SIZE-4] =0xAAAAAAAA;  /*R12*/
  TCB_Stacks[i][STACK_SIZE-5] =0xAAAAAAAA;  /*R3*/
  TCB_Stacks[i][STACK_SIZE-6] =0xAAAAAAAA;  /*R2*/
  TCB_Stacks[i][STACK_SIZE-7] =0xAAAAAAAA;  /*R1*/
  TCB_Stacks[i][STACK_SIZE-8] =0xAAAAAAAA;  /*R0*/


  TCB_Stacks[i][STACK_SIZE-9] =0xAAAAAAAA;  /*R11*/
  TCB_Stacks[i][STACK_SIZE-10] =0xAAAAAAAA; /*R10*/
  TCB_Stacks[i][STACK_SIZE-11] =0xAAAAAAAA; /*R9*/
  TCB_Stacks[i][STACK_SIZE-12] =0xAAAAAAAA; /*R8*/
  TCB_Stacks[i][STACK_SIZE-13] =0xAAAAAAAA; /*R7*/
  TCB_Stacks[i][STACK_SIZE-14] =0xAAAAAAAA; /*R6*/
  TCB_Stacks[i][STACK_SIZE-15] =0xAAAAAAAA; /*R5*/
  TCB_Stacks[i][STACK_SIZE-16] =0xAAAAAAAA; /*R4*/
}

uint8_t osKernelAddThreads(void(*task0)(void),void(*task1)(void),void(*task2)(void))
{
  /*disable global interrup*/
  __disable_irq();
  tcbs[0].nextPt = &tcbs[1].stackPt;
  tcbs[1].nextPt = &tcbs[2].stackPt;
  tcbs[2].nextPt = &tcbs[0].stackPt;

/*initialise thread0*/
  osKernelStackInit(0);
  /*initialise PC*/
  TCB_Stacks[0][STACK_SIZE-2] = (uint32_t)(task0);

  osKernelStackInit(1);
  TCB_Stacks[1][STACK_SIZE-2] = (uint32_t)(task1);

  osKernelStackInit(2);
  TCB_Stacks[2][STACK_SIZE-2] = (uint32_t)(task2);

  currentPt = &tcbs[0];

  __enable_irq();
  return 1;
}
void osKernelInit()
{
  MILLIS_PRESCALER = (BUS_FREQ/1000);
}

void osKernelLaunch(uint32_t quanta)
{
  /*Reset systick*/
  SysTick->CTRL = SYSTICK_RST;
  /*clear systick register value */
  SysTick->VAL = 0;
  /*Load quanta*/
  SysTick->LOAD = (quanta*MILLIS_PRESCALER)-1 ;
  /*set systick to low priority*/
  NVIC_SetPriority(SysTick_IRQn,15);
  /*Enable systick , select internal clck*/
  SysTick->CTRL = CTRL_ENABLE| CTRL_CLCKSRC;
  /*Enable systick interrupt*/
  SysTick->CTRL |= CTRL_TICKINT;
  /*Launch scheduler*/
  osSchedulerLaunch();
}

__attribute__((naked)) void SysTick_Handler(void)
{
  /*SUSPEND the current thread*/
  /*Disable the global interrupt*/
  __asm("CPSID I");
  /*save r4,r5.r6,r7,r8,r9,r10,r11*/
  __asm("PUSH {R4-R11}");
  /*load the address of currentPt in r0, r0 = &currentPt*/
  __asm("LDR R0,=currentPt");
  /*load r1 with value at address of r0 i.e. r1=currentPt */
  __asm("LDR R1,[R0]");
  /*store cotex-m sp at the address of r1 i.e. currentPt->stackPt = SP(save sp into tcb)*/
  __asm("STR SP,[R1]");

#if(SCHEDULER == PERIODIC_SCHEDULER)
  __asm("PUSH {R1,LR}");
  __asm("BL osPeriodicScheduler");
  __asm("POP {R1,LR}");
#endif

#if (SCHEDULER != PRIORITY_SCHEDULER)
/*choose next thread*/
  /*load r1 with the loaction 4 bytes above the address of r1, i.e. r1= currentPt->nextPt*/
  __asm("LDR R1,[R1,#4]");
  /*store value of r1 into address pointed by r0 . i.e. currentPt = currentPt->nextPt */
  __asm("STR R1,[R0]");
  /*load the cortex -M SP with value at the address point by r1, i.e. SP = (currentPt->stackPt)*/
  __asm("LDR SP,[R1]");

#else
  __asm("PUSH {R0,LR}");
  __asm("BL osPriorityScheduler");
  __asm("POP {R0,LR}");
  /*R1 =  currentPt i.e. New Thread*/
  __asm("LDR		R1,[R0]");
  /*SP  = currentPt->StackPt*/
  __asm("LDR		SP,[R1]");
#endif
  /*restore r4,r5.r6,r7,r8,r9,r10,r11*/
  __asm("POP {R4-R11}");
    /*enable the global interrupt*/
  __asm("CPSIE I");
  /*return from exception and restore r0,r1,r2,r3,r12,lr,pc and psr*/
  __asm("BX LR");
}

void osSchedulerLaunch()
{
  /*load the address of currentPt in r0, r0 = &currentPt*/
  __asm("LDR R0,=currentPt");
  /*load value at address pointed by r0 to r2. i.e. r2 = currentPt */
  __asm("LDR R2,[R0]");
  /*load the cortex -M SP with value at the address point by r2, i.e. SP = (currentPt->stackPt)*/
  __asm("LDR SP,[R2]");
  /*restore r4,r5.r6,r7,r8,r9,r10,r11*/
  __asm("POP {R4-R11}");
  /*restore r12*/
  __asm("POP {R12}");
  /*restore r0,r1,r2,r3*/
  __asm("POP {R0-R3}");
  /*skip */
  __asm("ADD SP,SP,#4");
  /*create new start location by poping LR*/
  __asm("POP {LR}");
  /*skip PSR*/
  __asm("ADD SP,SP,#4");
    /*enable the global interrupt*/
  __asm("CPSIE I");
  /*return from exception*/
  __asm("BX LR");
}

void osThreadYield()
{
	  /*clear systick register value */
	  SysTick->VAL = 0;
	  /*Trigger SysTick*/
	  ICSR |= PENDSTSET;
}

#if(SCHEDULER == PERIODIC_SCHEDULER)
uint32_t period_tick;
void osPeriodicScheduler()
{
  if((++period_tick)==PERIOD)
  {
    (*task3)();
    period_tick=0;
  }
}
#endif

void osSemaphoreInit(int32_t *semaphore , int32_t value)
{
  *semaphore = value;
}

void osSemaphoreGive(int32_t *semaphore)
{
  __disable_irq();
  *semaphore+=1;
  __enable_irq();
}

void osSemaphoreWait(int32_t *semaphore)
{
  __disable_irq();
  while(*semaphore<=0)
  {
      __disable_irq();
      __enable_irq();
  }
    *semaphore-=1;
  __enable_irq();
}

#if (SCHEDULER == PRIORITY_SCHEDULER)

void osPriorityScheduler()
{
  tcbType * _currentPt=currentPt;
  tcbType * nextThreadToRun = currentPt;
  uint8_t highestPriorityFound=255;
  do {
    _currentPt=_currentPt->nextPt;
    if((_currentPt->priority < highestPriorityFound)
      &&(_currentPt->sleepTime==0) && (_currentPt->blocked ==0))
      {
        nextThreadToRun=_currentPt;
        highestPriorityFound=_currentPt->priority;
      }
  } while(_currentPt!=currentPt);
  currentPt=nextThreadToRun;
}
#endif


//Need to use PendSV for scheduling but currently not working
//void SysTick_Handler()
//{
//	// Trigger PendSV
//	ICSR |= (1<<28);
//}
