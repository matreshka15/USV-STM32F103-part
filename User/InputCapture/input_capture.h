#ifndef input_capture
#define input_capture
	#include "stm32f10x.h"
	void TIM2_Capture_Init(u16 arr,u16 psc);
	u8 TIM2_4channel_process(u16 arr,u16 psc);
	void TIM2_IRQHandler(void);

	extern u16 TIM2_CH1_STATUS,TIM2_CH2_STATUS,TIM2_CH3_STATUS,TIM2_CH4_STATUS;
	extern u16 TIM2_CH2_START_VALUE,TIM2_CH2_END_VALUE,TIM2_CH1_START_VALUE,TIM2_CH1_END_VALUE;
	extern u16 TIM2_CH3_START_VALUE,TIM2_CH3_END_VALUE,TIM2_CH4_START_VALUE,TIM2_CH4_END_VALUE;
	extern u32 CHANNEL_HIGHOUTPUT[4];
#endif

