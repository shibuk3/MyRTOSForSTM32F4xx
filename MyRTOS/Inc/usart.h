#ifndef __USART_H__
#define __USART_H__
#include "stm32f4xx.h"

void usart_init();
void usart_send_char(uint8_t data);
void usart_send_bytes(uint8_t data[],uint8_t byteLength);
void usart_receive_char(uint8_t *data);
void usart_receive_bytes(uint8_t data[],uint8_t byteLength);

void print(char * buffer);

#endif


