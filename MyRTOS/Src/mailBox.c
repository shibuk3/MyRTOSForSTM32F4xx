#include "mailBox.h"

static uint32_t MB_data;
static uint8_t MB_has_data;
static uint32_t MB_sem;

void osMailBoxInit()
{
  MB_has_data=0;
  MB_data = 0;
  osSemaphoreInit(&MB_sem,0);
}
void osMailBoxSend(uint32_t data)
{
  __disable_irq();
  if(MB_has_data)
  {
    __enable_irq();
    return;
  }
  MB_data = data;
  MB_has_data = 1;
  __enable_irq();
  osSemaphoreGive(&MB_sem);
}

uint32_t osMailBoxRecv()
{

	osSemaphoreWait(&MB_sem);
  uint32_t data;
  __disable_irq();
  data = MB_data;
  MB_has_data = 0;
  __enable_irq();
  return data;
}
uint32_t sensorValue = 455;
void MB_task0()
{
  while(1)
  {
    osMailBoxSend(sensorValue);
  }
}
uint32_t reveivedValue;
void MB_task1()
{
  while(1)
  {
    reveivedValue = osMailBoxRecv();
  }
}
uint32_t MB_counter;
void MB_task2()
{
  while(1)
  {
	  MB_counter++;
  }
}

//main

//int main()
//{
//	init;
//#if (MAILBOX==1)
//	  osKernelAddThreads(&MB_task0,&MB_task1,&MB_task2);
//#endif
//}
