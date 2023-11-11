#include "led.h"
#define GPIOAEN  (1<<0)         /* IO port A clock enable*/
#define LED_ON   (1<<5)
void led2_init()
{
	/*Enable clock access to led port (Port A). If we Enable it after selecting Mode it wont work*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set led pin(i.e. PA5) as output pin*/
	GPIOA->MODER |= (1<<10);
	GPIOA->MODER &= ~(1<<11);
}

void led2_on()
{
	/*Set led pin HIGH (PA5)*/
	GPIOA->ODR |= LED_ON;
}

void led2_off()
{
	/*Set led pin LOW (PA5)*/
	GPIOA->ODR &= ~LED_ON;
}

void led2_toggle()
{
	/*Toggle led pin LOW (PA5)*/
	GPIOA->ODR ^= LED_ON;
}
