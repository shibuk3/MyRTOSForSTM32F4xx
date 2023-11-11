#include "timer.h"
void time2_1Hz_Interrupt_Init()
{
  /*enable clock access to timer2*/
  RCC->APB1ENR |= TIMER2_EN;
  /*set the timer prescaler*/
  TIM2->PSC = 1600-1;   //counter frequency = 16 000 000 / 1600=10000;
  /*set auto reload register*/
  TIM2->ARR = 10000-1;     //  10000/counter frequency = 1 sec;
  /*clear counter*/
  TIM2->CNT = 0;
  /*enable counter*/
  TIM2->CR1 |= CR1_EN;
  /*enable interrupt*/
  TIM2->DIER |= DIER_UIE;
  /*enable timer interrupt in NVIC*/
  NVIC_EnableIRQ(TIM2_IRQn);
}
