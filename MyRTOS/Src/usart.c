#include "usart.h"

void usart_init()
{

	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER &= ~(1<<4);
	GPIOA->MODER |= (1<<5);

	GPIOA->AFR[0] |= (1<<8);
	GPIOA->AFR[0] |= (1<<9);
	GPIOA->AFR[0] |= (1<<10);
	GPIOA->AFR[0] &= ~(1<<11);


//	USART2->CR1 &= ~(1<<12);
//
//	USART2->CR2 &= ~(1<<12);
//	USART2->CR2 &= ~(1<<13);
//
//	USART2->CR3 &= ~(1<<7);
	RCC->APB1ENR |= (1<<17);
	USART2->BRR = (((16000000/(16*115200))<<4) + ((16000000/(16*115200)) & 0x0F));

	USART2->CR1 |= (1<<3);
	USART2->CR1 |= (1<<13);



//	RCC->AHB1ENR |= (1<<0);

	GPIOA->MODER &= ~(1<<6);
	GPIOA->MODER |= (1<<7);

	GPIOA->AFR[0] |= (1<<12);
	GPIOA->AFR[0] |= (1<<13);
	GPIOA->AFR[0] |= (1<<14);
	GPIOA->AFR[0] &= ~(1<<15);


//	RCC->APB1ENR |= (1<<17);

//	USART2->CR1 |= (1<<13);
//	USART2->CR1 &= ~(1<<12);
	USART2->CR1 |= (1<<2);

	USART2->CR2 &= ~(1<<12);
	USART2->CR2 &= ~(1<<13);

//	USART2->BRR = (((16000000/(16*115200))<<4) + ((16000000/(16*115200)) & 0x0F));

	USART2->CR3 &= ~(1<<6);
}

void usart_send_char(uint8_t data)
{
	while(!(USART2->SR & (1<<6))){}
	USART2->DR = data;
}

void usart_send_bytes(uint8_t data[],uint8_t byteLength)
{
	for(uint8_t i=0;i<byteLength-1;i++)
	{
		usart_send_char(data[i]);
	}
}

void usart_receive_char(uint8_t *data)
{
	while(!(USART2->SR & (1<<5))){}
	*data = USART2->DR;
}

void usart_receive_bytes(uint8_t data[],uint8_t byteLength)
{
	for(uint8_t i=0;i<byteLength-1;i++)
	{
		usart_receive_char(&data[i]);
	}
}

void print(char * buffer)
{
	uint8_t payLoad[100];
	memset(payLoad,0,100);
	sprintf(payLoad,buffer);
	usart_send_bytes(payLoad,sizeof(payLoad));
}
