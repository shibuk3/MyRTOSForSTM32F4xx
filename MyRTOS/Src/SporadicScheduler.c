#include "SporadicScheduler.h"

//using edge detection for Sporadic task

/*Enable PC13 for Edge detection*/
void BSPEdgeTriggerInit()
{
  __disable_irq();
  RCC->AHB1ENR |=4;         //Enable clock to port c
  RCC->APB2ENR |=0x4000;    //Enabe clock to SYSCFG
  GPIOC->MODER &= ~0x0C000000; //setting the pin in Input mode

  SYSCFG->EXTICR[3] &= ~0x00F0; //clear port selection for EXTI13
  SYSCFG->EXTICR[3] |= 0x0020;  //select the port c for EXTI13
  EXTI->IMR |= 0x2000;   //unmask EXTI13
  EXTI->FTSR |=0x2000;   //select falling edge trigger
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  __enable_irq();
}
int32_t *edgeSemaphore;
void osEdgeTriggerInit(int32_t *semaphore)
{
  edgeSemaphore=semaphore;
  BSPEdgeTriggerInit();
}

#if SCHEDULER == SPORADIC_SCHEDULER
extern int32_t edgeSem;

void EXTI15_10_IRQHandler()
{
	osSemaphoreGive(&edgeSem);
  EXTI->PR |=0x2000;
}

uint32_t sporadicSchedulerCounter;
void sporadicTask()
{
  while(1)
  {
	osSemaphoreWait(&edgeSem);
    sporadicSchedulerCounter++;
  }
}
#endif

void NormalTask1()
{
  while(1)
  {
    //we can generate falling edge using frequency generator
    //or push button. here we are generating by connecting led pin to PC13
	  led2_toggle();
    //if we comment counter should not increase
  }
}
void NormalTask2()
{
  while(1)
  {

  }
}
//main.c
//int32_t *edgeSem;
//int main()
//{
//  osEdgeTriggerInit(&edgeSem);
//  osKernelAddThread(&sporadicTask,&NormalTask1,&NormalTask2)
//}


