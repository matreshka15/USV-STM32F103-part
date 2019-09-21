#ifndef DELAY
#define DELAY

#endif

#include "stm32f10x.h"
void delay_init(u8 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u16 nms);
