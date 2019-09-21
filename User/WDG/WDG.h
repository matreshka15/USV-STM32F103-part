#ifndef __WDG
#define __WDG
#include "stm32f10x.h"

void WDG_Initialize(uint8_t prescaler,uint16_t reload);
void WDG_Start(void);
void feed_dog(void);
#endif
