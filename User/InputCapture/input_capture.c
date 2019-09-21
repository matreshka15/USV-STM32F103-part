#include "stm32f10x.h"
#include <stdio.h>
u16 TIM2_CH1_STATUS,TIM2_CH2_STATUS,TIM2_CH3_STATUS,TIM2_CH4_STATUS;
u16 TIM2_CH2_START_VALUE,TIM2_CH2_END_VALUE,TIM2_CH1_START_VALUE,TIM2_CH1_END_VALUE;
u16 TIM2_CH3_START_VALUE,TIM2_CH3_END_VALUE,TIM2_CH4_START_VALUE,TIM2_CH4_END_VALUE;
u8 USART_PRINT_CHAR(u8 dataout);
u32 CHANNEL_HIGHOUTPUT[4];
/*
说明：TIM2定时器的四个通道对应引脚分别分PA0\PA1\PA2\PA3.
Warning！ arr的数值越小，通道测量精度越高；但测量通道的高电平时间最大值减小！
					实测：arr的值为10时，测量最大值为180ms左右。
					psc设置为72-1，则（arr+1）即定时器更新中断一次的时间。单位为us.
*/
void TIM2_Capture_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	/*时钟使能*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	/*设置TIM*/
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	/*设置GPIO*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	//捕获高电平，下拉端口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);

	/*设置捕获*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0;  //不滤波
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿触发
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
	/*设置捕获*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0;  //不滤波
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿触发
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
	/*设置捕获*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ITConfig(TIM2,TIM_IT_CC3,ENABLE);
	/*设置捕获*/
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE);
	/*设置NVIC*/
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		/*使能指定终端类型*/
		TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);
		
		TIM_Cmd(TIM2,ENABLE);
}


u8 TIM2_4channel_process(u16 arr,u16 psc)
{
	u8 processed=0;
	u32 STATUS_CH;
	/*当通道1数据有效时*/
	TIM_Cmd(TIM2,DISABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
	if(TIM2_CH1_STATUS&0x8000)
	{
		STATUS_CH = TIM2_CH1_STATUS&0x3FFF;
		processed=1;						/*如果STATUS>0*/
		if(STATUS_CH>0) 
		{
			CHANNEL_HIGHOUTPUT[0] = (STATUS_CH-1)*(arr+1) + (arr+1-TIM2_CH1_START_VALUE)+TIM2_CH1_END_VALUE;
		}
		else 
		{
			if(TIM2_CH1_END_VALUE>=TIM2_CH1_START_VALUE) CHANNEL_HIGHOUTPUT[0] = TIM2_CH1_END_VALUE-TIM2_CH1_START_VALUE;
			else CHANNEL_HIGHOUTPUT[0] = TIM2_CH1_START_VALUE - TIM2_CH1_END_VALUE;
		}
	}
	
	if(TIM2_CH2_STATUS&0x8000)
	{
		STATUS_CH = TIM2_CH2_STATUS&0x3FFF;
		processed=1;
		if(STATUS_CH>0) 
		{
			CHANNEL_HIGHOUTPUT[1] = (STATUS_CH-1)*(arr+1) + (arr+1-TIM2_CH2_START_VALUE)+TIM2_CH2_END_VALUE;
		}
		else 
		{
			if(TIM2_CH2_END_VALUE>=TIM2_CH2_START_VALUE) CHANNEL_HIGHOUTPUT[1] = TIM2_CH2_END_VALUE-TIM2_CH2_START_VALUE;
			else CHANNEL_HIGHOUTPUT[1] = TIM2_CH2_START_VALUE - TIM2_CH2_END_VALUE;
		}
	}

	if(TIM2_CH3_STATUS&0x8000)
	{
		STATUS_CH = TIM2_CH3_STATUS&0x3FFF;
		processed=1;
		if(STATUS_CH>0) 
		{
			CHANNEL_HIGHOUTPUT[2] = (STATUS_CH-1)*(arr+1) + (arr+1-TIM2_CH3_START_VALUE)+TIM2_CH3_END_VALUE;
		}
		else 
		{
			if(TIM2_CH3_END_VALUE>=TIM2_CH3_START_VALUE) CHANNEL_HIGHOUTPUT[2] = TIM2_CH3_END_VALUE-TIM2_CH3_START_VALUE;
			else CHANNEL_HIGHOUTPUT[2] = TIM2_CH3_START_VALUE - TIM2_CH3_END_VALUE;
		}
	}

	if(TIM2_CH4_STATUS&0x8000)
	{
		STATUS_CH = TIM2_CH4_STATUS&0x3FFF;
		processed=1;
		if(STATUS_CH>0) 
		{
			CHANNEL_HIGHOUTPUT[3] = (STATUS_CH-1)*(arr+1) + (arr+1-TIM2_CH4_START_VALUE)+TIM2_CH4_END_VALUE;
		}
		else 
		{
			if(TIM2_CH4_END_VALUE>=TIM2_CH4_START_VALUE) CHANNEL_HIGHOUTPUT[3] = TIM2_CH4_END_VALUE-TIM2_CH4_START_VALUE;
			else CHANNEL_HIGHOUTPUT[3] = TIM2_CH4_START_VALUE - TIM2_CH4_END_VALUE;
		}
	}	

	/*
	处理完数据后必须清除标志位，重新开始下一次读取
	*/
	TIM2_CH1_STATUS = 0;
	TIM2_CH2_STATUS = 0;
	TIM2_CH3_STATUS = 0;
	TIM2_CH4_STATUS = 0;
	TIM_Cmd(TIM2,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);	
	return processed;
}


void TIM2_IRQHandler()
{
	/*
	============更新中断处理==============
	65536us偏差：解决：
	计65536:
	SRART_VALUE:300
	END_VALUE:700
	实际：65536 - 300 + 700 us
	*/
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
		if((!(TIM2_CH1_STATUS&0x8000))&&(TIM2_CH1_STATUS&0x4000))
		{
			if((TIM2_CH1_STATUS&0x3FFF)==0x3FFF)
			{
				TIM2_CH1_STATUS |= 0x8000;
				TIM2_CH1_END_VALUE=0xffff;
			}else 
			{
				TIM2_CH1_STATUS ++;			
			}
		}
		
		if((!(TIM2_CH2_STATUS&0x8000))&&(TIM2_CH2_STATUS&0x4000))
		{
				if((TIM2_CH2_STATUS&0x3FFF)==0x3FFF)
				{
					TIM2_CH2_STATUS |= 0x8000;
					TIM2_CH2_END_VALUE=0xffff;
				}else 
				{
					TIM2_CH2_STATUS ++;			
				}
		}
		
		if((!(TIM2_CH3_STATUS&0x8000))&&(TIM2_CH3_STATUS&0x4000))
		{
				if((TIM2_CH3_STATUS&0x3FFF)==0x3FFF)
				{
					TIM2_CH3_STATUS |= 0x8000;
					TIM2_CH3_END_VALUE=0xffff;
				}else 
				{
					TIM2_CH3_STATUS ++;			
				}
		}
		
		if((!(TIM2_CH4_STATUS&0x8000))&&(TIM2_CH4_STATUS&0x4000))
		{
				if((TIM2_CH4_STATUS&0x3FFF)==0x3FFF)
				{
					TIM2_CH4_STATUS |= 0x8000;
					TIM2_CH4_END_VALUE=0xffff;
				}else 
				{
					TIM2_CH4_STATUS ++;			
				}
		}
		
		
	}
	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	
	/*
	==========捕获中断处理============
	*/
	if(TIM_GetITStatus(TIM2,TIM_IT_CC1))	//捕获中断
	{
		if(TIM2_CH1_STATUS&0x4000)
		{	
			TIM2_CH1_END_VALUE = TIM_GetCapture1(TIM2);
			TIM2_CH1_STATUS |= 0x8000;
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising);	//重改为上升沿触发
		}
		else
		{
			TIM2_CH1_START_VALUE = TIM_GetCapture1(TIM2);
			TIM2_CH1_STATUS= 0;
			TIM2_CH1_STATUS |= 0x4000;
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
	}
	
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2))	//捕获中断
	{
		if(TIM2_CH2_STATUS&0x4000)
		{
			TIM2_CH2_END_VALUE = TIM_GetCapture2(TIM2);
			TIM2_CH2_STATUS |= 0x8000;
			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising);	//重改为上升沿触发
		}
		else
		{
			TIM2_CH2_START_VALUE = TIM_GetCapture2(TIM2);
			TIM2_CH2_STATUS= 0;
			TIM2_CH2_STATUS |= 0x4000;
			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling);
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC2);
	}
	
	if(TIM_GetITStatus(TIM2,TIM_IT_CC3))	//捕获中断
	{
		if(TIM2_CH3_STATUS&0x4000)
		{
			TIM2_CH3_END_VALUE = TIM_GetCapture3(TIM2);
			TIM2_CH3_STATUS |= 0x8000;
			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising);	//重改为上升沿触发
		}
		else
		{
			TIM2_CH3_START_VALUE = TIM_GetCapture3(TIM2);
			TIM2_CH3_STATUS= 0;
			TIM2_CH3_STATUS |= 0x4000;
			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling);
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC3);
	}

	if(TIM_GetITStatus(TIM2,TIM_IT_CC4))	//捕获中断
	{
		if(TIM2_CH4_STATUS&0x4000)
		{
			TIM2_CH4_END_VALUE = TIM_GetCapture4(TIM2);
			TIM2_CH4_STATUS |= 0x8000;
			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising);	//重改为上升沿触发
		}
		else
		{
			TIM2_CH4_START_VALUE = TIM_GetCapture4(TIM2);
			TIM2_CH4_STATUS= 0;
			TIM2_CH4_STATUS |= 0x4000;
			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling);
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_CC4);
	}		
	//TIM_ClearITPendingBit(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4);
}
