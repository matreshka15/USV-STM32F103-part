#include "stm32f10x.h"

//50Hz的电调控制方波： arr=400 --20000/400=50--  psc=3599
//PWM 频率=72000/3600=20khz=0.05ms  
//一般我们脉宽的范围设置为1000us~2000us
//使用TIM_SetCompare1(TIMx,数值);改变占空比

void TIM8_PWM_Init(u16 arr,u16 psc)//PWM 频率=72000/3600=20khz=0.05ms  
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	TIM_DeInit(TIM8);
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	
	TIM_OCStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	TIM_OCStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStructure.TIM_Pulse = arr; //初始化反转时机为arr，防止电机突转
	TIM_OC1Init(TIM8,&TIM_OCStructure);
	TIM_OC2Init(TIM8,&TIM_OCStructure);
	TIM_OC3Init(TIM8,&TIM_OCStructure);//通道三输出给舵机
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); //MOE 主输出使能
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM8,ENABLE);//自动重装载使能
	
	TIM_Cmd(TIM8,ENABLE);
}
