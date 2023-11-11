#include "FIFO.h"

#define FIFO_SIZE 15
uint32_t PutI;
uint32_t GetI;
uint32_t osFifo[FIFO_SIZE];
uint32_t current_fifo_size;
uint32_t lost_data;

void osFifoInit()
{
  PutI=0;
  GetI=0;
  osSemaphoreInit(&current_fifo_size,0);
  lost_data=0;
}
int8_t osFifoPut(uint32_t data)
{
  if(current_fifo_size==FIFO_SIZE)
  {
    lost_data++;
    return -1;
  }
  osFifo[PutI]=data;
  PutI = (PutI+1)%FIFO_SIZE;
  osSemaphoreGive(&current_fifo_size);
  return 1;
}

uint32_t osFifoGet()
{
  uint32_t data;
  osSemaphoreWait(&current_fifo_size);
  __disable_irq();
  data=osFifo[GetI];
  GetI=(GetI+1)%FIFO_SIZE;
  __enable_irq();
  return data;
}

//uint32_t FIFOsensorValue = 455;
void FIFO_task0()
{
  while(1)
  {
    for(uint8_t i=0; i<20;i++)
    {
      osFifoPut(i);
    }
  }
}
uint32_t FIFOreveivedValue;
void FIFO_task1()
{
  while(1)
  {
	  FIFOreveivedValue = osFifoGet();
  }
}
uint32_t FIFO_Counter;
void FIFO_task2()
{
  while(1)
  {
	  FIFO_Counter++;
  }
}

//main

//int main()
//{
//	init;
//#if (FIFO==1)
//osFifoInit();
//osKernelAddThreads(&FIFO_task0,&FIFO_task1,&FIFO_task2);
//#endif
