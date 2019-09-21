#ifndef __WIRELESSUART
#define __WIRELESSUART
#include "stm32f10x.h"

extern u8 WirelessPort_Online;

void WirelessPort_GPIO_Init(void);
u8 Read_AUX_Status(void);

#endif
