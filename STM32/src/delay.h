/*
 * delay.h
 */

#ifndef DELAY_H_
#define DELAY_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx.h"

uint32_t tick;

void tick_init(void);
void tick_increment(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint32_t micros(void);
uint32_t millis(void);

#ifdef __cplusplus
}
#endif

#endif /* DELAY_H_ */
