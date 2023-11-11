#ifndef __GPIO_H__
#define __GPIO_H__
#include "stm32f4xx.h"

#define GPIO_PIN5  (1<<5)

typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

void gpio_init();
uint8_t isButtonPressed();
void GPIO_SetReset_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
#endif
