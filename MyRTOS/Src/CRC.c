#include "CRC.h"

void crc_init()
{
	RCC->AHB1ENR |= (1<<12);
}

void crc_reset()
{
	CRC->CR |= (1<<0);
}
uint32_t caluculate_crc(uint32_t buffer[],uint32_t length)
{
	crc_reset();
	for(uint32_t i=0;i<length;i++)
	{
		CRC->DR = buffer[length];
	}
	return CRC->DR;
}

uint32_t accumulate_crc(uint32_t buffer[],uint32_t length)
{
	for(uint32_t i=0;i<length;i++)
	{
		CRC->DR = buffer[length];
	}
	return CRC->DR;
}
