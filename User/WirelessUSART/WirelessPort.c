#define WirelessPort 1
#include "..\CONFIGURATION.h"

u8 WirelessPort_Online = 0;
void WirelessPort_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_GPIOPORT,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=MD0;
	GPIO_Init(GPIOPORT,&GPIO_InitStructure);//MD0
	GPIO_ResetBits(GPIOPORT,MD0);
	
	GPIO_InitStructure.GPIO_Pin=AUX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOPORT,&GPIO_InitStructure);//AUX
}

u8 Read_AUX_Status(void)
{
	return GPIO_ReadInputDataBit(GPIOPORT,AUX);
}
