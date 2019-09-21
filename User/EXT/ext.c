#include "stm32f10x.h"

void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//用到外部中断，需要使能AFIO时钟
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource5);		//映射中断线
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;				//中断线5启动中断
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		//中断通道：EXTI  9-5号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5))
	{
		;
	}
	EXTI_ClearITPendingBit(EXTI_Line5);
}
