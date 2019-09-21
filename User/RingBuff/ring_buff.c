#include "stm32f10x.h"
#define RINGBUFF 1
#include "..\CONFIGURATION.h"


/*=====================供GPS使用的RING BUFF==========================*/
void ringBuff_Init(void)
{
	ringBuff.Head = 0;
	ringBuff.Tail = 0;
	ringBuff.Length = 0;
}

u8 WriteRingBuff(u8 data)
{
	if(ringBuff.Length >= RINGBUFF_LEN)
	{
		return FALSE;
	}
	ringBuff.Ring_Buff[ringBuff.Tail] = data;
	ringBuff.Tail = (ringBuff.Tail+1)%RINGBUFF_LEN;
	ringBuff.Length++;
	return TRUE;
}

u8 Read_ringBuff(u8 *rData)
{
	#if USE_USART3_GPS_DMA_RX
		//DMA_Cmd(DMA1_Channel3, DISABLE);//禁用DMA，防止错位
	#endif
	if(ringBuff.Length == 0)
	{
		return FALSE;
	}
	*rData = ringBuff.Ring_Buff[ringBuff.Head];
	ringBuff.Head = (ringBuff.Head+1)%RINGBUFF_LEN;
	ringBuff.Length--;
	#if USE_USART3_GPS_DMA_RX
		//DMA_Cmd(DMA1_Channel3, ENABLE);
	#endif
	return TRUE;
}

void Offset(u8 number)
{
	u8 counter;
	u8 skipByte	= number;
	u8 dataIn;
		for(counter=0;counter<skipByte;counter++)
		{//偏移 n Byte，直接偏移到要读取的pDOP数据处
			Read_ringBuff(&dataIn);
		}
}
u16 Get_Buff_Data_Number_GPS(void)
{
	return ringBuff.Length;
}
/*==================供IMU使用的RING BUFF====================*/
void ringBuff_IMU_Init(void)
{
	ringBuff_IMU.Head = 0;
	ringBuff_IMU.Tail = 0;
	ringBuff_IMU.Length = 0;
}

u8 Write_IMU_ringBuff(u8 data)
{
	if(ringBuff_IMU.Length >= RINGBUFF_LEN)
	{
		return FALSE;
	}
	ringBuff_IMU.Ring_Buff[ringBuff_IMU.Tail] = data;
	ringBuff_IMU.Tail = (ringBuff_IMU.Tail+1)%RINGBUFF_LEN;
	ringBuff_IMU.Length++;
	return TRUE;
}

u8 Read_IMU_ringBuff(u8 *rData)
{
	if(ringBuff_IMU.Length == 0)
	{
		return FALSE;
	}
	*rData = ringBuff_IMU.Ring_Buff[ringBuff_IMU.Head];
	ringBuff_IMU.Head = (ringBuff_IMU.Head+1)%RINGBUFF_LEN;
	ringBuff_IMU.Length--;
	return TRUE;
}

void Offset_IMU(u8 number)
{
	u8 counter;
	u8 skipByte	= number;
	u8 dataIn;
		for(counter=0;counter<skipByte;counter++)
		{//偏移 n Byte，直接偏移到要读取的pDOP数据处
			Read_IMU_ringBuff(&dataIn);
		}
}

/*==================供USART1使用的RING BUFF====================*/
void ringBuff_USART1_Init(void)
{
	ringBuff_USART1.Head = 0;
	ringBuff_USART1.Tail = 0;
	ringBuff_USART1.Length = 0;
}

u8 Write_USART1_ringBuff(u8 data)
{
	if(ringBuff_USART1.Length >= RINGBUFF_LEN)
	{
		return FALSE;
	}
	ringBuff_USART1.Ring_Buff[ringBuff_USART1.Tail] = data;
	ringBuff_USART1.Tail = (ringBuff_USART1.Tail+1)%RINGBUFF_LEN;
	ringBuff_USART1.Length++;
	return TRUE;
}

u8 Read_USART1_ringBuff(u8 *rData)
{
	if(ringBuff_USART1.Length == 0)
	{
		return FALSE;
	}
	*rData = ringBuff_USART1.Ring_Buff[ringBuff_USART1.Head];
	ringBuff_USART1.Head = (ringBuff_USART1.Head+1)%RINGBUFF_LEN;
	ringBuff_USART1.Length--;
	return TRUE;
}

void Offset_USART1(u8 number)
{
	u8 counter;
	u8 skipByte	= number;
	u8 dataIn;
		for(counter=0;counter<skipByte;counter++)
		{//偏移 n Byte，直接偏移到要读取的pDOP数据处
			Read_USART1_ringBuff(&dataIn);
		}
}

u16 Get_Buff_Data_Number(void)
{
	return ringBuff_USART1.Length;
}
