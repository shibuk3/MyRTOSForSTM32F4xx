#include "led.h"
#include "usart.h"
#include "gpio.h"
//#include "timebase.h"
#include<stdio.h>
#include<string.h>
#include "osKernel.h"
#include "timer.h"

#include "config.h"
#include "PriorityScheduler.h"
#include "SporadicScheduler.h"
#include "mailBox.h"
#include "FIFO.h"
//#define BL_RX_LEN  200
//uint8_t bl_rx_buffer2[BL_RX_LEN];
//int main()
//{
//	led2_init();
//	usart_init();
//	uint8_t rcv_len=0;
//	while(1)
//	{
//		gpio_init();
//		led2_on();
//		for(int i=0;i<900000;i++){}
//		led2_off();
//		for(int i=0;i<900000;i++){}
//		memset(bl_rx_buffer2,0,200);
//		//here we will read and decode the commands coming from host
//		//first read only one byte from the host , which is the "length" field of the command packet
//		usart_receive_bytes(bl_rx_buffer2,2);
//		usart_send_bytes(bl_rx_buffer2,2);
//		rcv_len= bl_rx_buffer2[0]-'0';
//		usart_receive_bytes(&bl_rx_buffer2[2],rcv_len+1);
//		usart_send_bytes(bl_rx_buffer2+2,2);
//		uint8_t payLoad[100];
//		memset(payLoad,0,100);
//		sprintf(payLoad,"BL_DEBUG_MSG:command code received from host:%d %d%d \r\n" , bl_rx_buffer2[0], bl_rx_buffer2[2],bl_rx_buffer2[3]);
//	    usart_send_bytes(payLoad,sizeof(payLoad));
//		uint8_t data[50];
//		uint8_t length = 2;
//		usart_receive_bytes(data,length);
//		usart_send_bytes(data,length);
//		if(data == 'h')
//		{
//			led2_on();
//			for(int i=0;i<900000;i++){}
//		}
//		else
//		{
//			led2_off();
//			for(int i=0;i<900000;i++){}
//		}
//		uint8_t payLoad[100];
//		memset(payLoad,0,sizeof(payLoad));
//		sprintf(payLoad,"Hi, Hello world %d !\r\n",20);
//		//uint8_t payLoad[] = "Hi, Hello world" +20+"!\r\n";
//		usart_send_bytes(payLoad,sizeof(payLoad));
//		printf("Hello from STM32F4.........\n\r");
//	}
//}


//int main()
//{
//	gpio_init();
//	led2_init();
//	usart_init();
//	uint8_t payLoad[100];
//	while(1)
//	{
//	  if ( isButtonPressed() == 0 )
//	  {
//		  sprintf(payLoad,"BL_DEBUG_MSG:Button is pressed .. going to BL mode\n\r") ;
//		  usart_send_bytes(payLoad,sizeof(payLoad));
//		  //we should continue in bootloader mode
//		  bootloader_uart_read_data();
//
//	  }
//	  else
//	  {
//		  sprintf(payLoad,"BL_DEBUG_MSG:Button is not pressed .. executing user app\n" );
//		  usart_send_bytes(payLoad,sizeof(payLoad));
//			//jump to user application
//			bootloader_jump_to_user_app();
//
//	  }
//	}
//}


//testing systick timer
//int main()
//{
//	usart_init();
//	timerbase_init();
//	while(1)
//	{
//		delay(2);
//		uint8_t payLoad[100];
//		memset(payLoad,0,100);
//		sprintf(payLoad,"A second has passed\r\n");
//		usart_send_bytes(payLoad,sizeof(payLoad));
//
//	}
//}

void motor_on()
{
	print("Motor is starting... \n\r");
}

void motor_off()
{
	print("Motor is stoping... \n\r");
}

void valve_open()
{
	print("Valve is opening... \n\r");
}

void valve_close()
{
	print("Valve is closing... \n\r");
}

// int motor_main()
// {
// 	while(1)
// 	{
// 		delay(1);
// 		motor_on();
// 		delay(1);
// 		motor_off();
// 	}
// }
//
// int valve_main()
// {
// 	while(1)
// 	{
// 		delay(1);
// 		valve_open();
// 		delay(1);
// 		valve_close();
// 	}
// }
// //here most of time cpu is busy in delay function
// int main()
// {
// 	int flag=0;
// 	usart_init();
// 	timerbase_init();
// 	if(flag)
// 	{
// 		motor_main();
// 	}
// 	{
// 		valve_main();
// 	}
//
// //	while(1)
// //	{
// //		delay(1);
// //		motor_on();
// //		delay(1);
// //		motor_off();
// //	}
//
// }
typedef uint32_t TaskProfiler;
TaskProfiler Task0_Profiler, Task1_Profiler , Task2_Profiler;

int32_t semaphore1,semaphore2;
void task0()
{
	while(1)
	{
		 Task0_Profiler++;
//		osSemaphoreWait(&semaphore1);
//		motor_on();
//		osSemaphoreGive(&semaphore2);
		// osThreadYield();
	}
}

void task1()
{
	while(1)
	{
		 Task1_Profiler++;
//		osSemaphoreWait(&semaphore2);
//		valve_open();
//		osSemaphoreGive(&semaphore1);
	}
}

void task2()
{
	while(1)
	{
		Task2_Profiler++;
	}
}
#if(SCHEDULER == PERIODIC_SCHEDULER)
TaskProfiler Task3_Profiler;
void task3()
{
	Task3_Profiler++;
}
#endif

#if(SCHEDULER == PERIODIC_SCHEDULER_WITH_HARDWARE)
TaskProfiler Task4_Profiler;
void TIM2_IRQHandler()
{
	/*clear the update interrupt flag*/
	TIM2->SR &= ~SR_UIF;

	Task4_Profiler++;
}
#endif

#if SCHEDULER == SPORADIC_SCHEDULER
int32_t edgeSem;
#endif
int main()
{
	#if(SCHEDULER == PERIODIC_SCHEDULER_WITH_HARDWARE)
	time2_1Hz_Interrupt_Init();
	#endif
	usart_init();
	osSemaphoreInit(&semaphore1,1);
	osSemaphoreInit(&semaphore2,0);
	osKernelInit();
#if SCHEDULER == SPORADIC_SCHEDULER
	  osEdgeTriggerInit(&edgeSem);
	  osKernelAddThreads(&sporadicTask,&NormalTask1,&NormalTask2);
#elif (SCHEDULER == PRIORITY_SCHEDULER)
	  osKernelAddThread(&prioritySchedulerTask0,5,&prioritySchedulerTask1,1,
			  	  	  	&prioritySchedulerTask2,1,&prioritySchedulerTask3,2,
	                    &prioritySchedulerTask4,5,&prioritySchedulerTask5,1,
						&prioritySchedulerTask6,1,&prioritySchedulerTask7,3);
#else
	osKernelAddThreads(&task0,&task1,&task2);
#endif
	osKernelLaunch(QUANTA);
}
