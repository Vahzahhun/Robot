/*
 * delay.cpp
 */

#include "delay.h"

void tick_init(void)
{
	SysTick_Config(168);
}

void tick_increment(void)
{
	++tick;
}

void delay_us(uint32_t us)
{
	uint32_t tick_stop = tick + us;
	while(tick < tick_stop);
}

void delay_ms(uint32_t ms)
{
	uint32_t tick_stop = tick + ms * 1000;
	while(tick < tick_stop);
}

uint32_t micros(void)
{
	return tick;
}

uint32_t millis(void)
{
	return tick * 0.001;
}
