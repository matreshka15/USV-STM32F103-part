#ifndef __INPUT_CAPTURE
#define __INPUT_CAPTURE
#include "stm32f10x.h"

extern u32 CHANNEL_HIGHOUTPUT[4];

void TIM3_Capture_Init(void);
u8 TIM3_4channel_process(void);
void TIM3_IRQHandler(void);

#endif
