#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f103xb.h"

void delay_init(void);
static void cycleCounterInit(void);

uint32_t micros(void);
uint32_t millis(void);

void delay_us(uint32_t us);

void delay_ms(uint32_t ms);


#endif





























