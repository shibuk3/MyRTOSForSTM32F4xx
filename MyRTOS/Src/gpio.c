#include "gpio.h"

void gpio_init()
{
	RCC->AHB1ENR |= (1<<2);

	/*Set push button(i.e. PC13) as input pin*/
	GPIOC->MODER &= ~(1<<26);
	GPIOC->MODER &= ~(1<<27);
}

uint8_t isButtonPressed()
{
	return ((GPIOC->IDR & (1<<13)) == (1<<13));
}


/**
  * @brief  Sets or clears the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  */
void GPIO_SetReset_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{

  if(PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
  }
}
