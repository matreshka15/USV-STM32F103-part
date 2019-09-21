#ifndef USART
#define USART



#include "stm32f10x.h"
#include <stdio.h>

extern	u8 DMA_RX;
extern	u8 DMA_TX;
void usart_init(u32 baudrate,u8 DMA_RX,u8 DMA_TX);
int fputc(int ch,FILE *f);
void USART1_IRQHandler(void);
u8 USART1_SENDDATA(u8 datain);
u8 WirelessUSART_Config(void);
void USART1_DMA_ENABLE_ONCE(u16 size);
void Usart1_DMA_Tx_Config(u32 DMA_MemoryBaseAddr,u32 size);
#endif
