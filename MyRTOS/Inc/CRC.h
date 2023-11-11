#ifndef __CRC_H__
#define __CRC_H__
#include "stm32f4xx.h"
void crc_init();
uint32_t caluculate_crc(uint32_t buffer[],uint32_t length);
void crc_reset();
#endif
