#ifndef USER_RINGBUFF
#define USER_RINGBUFF
#include "stm32f10x.h"

#define LENGTH_OF_BUFF 200
extern u16 RINGBUFF_LEN;
	typedef struct
	{
		volatile u16 Head;
		volatile u16 Tail;
		volatile u16 Length;
		volatile u8 Ring_Buff[LENGTH_OF_BUFF];
	}RingBuff_t;
	/*初始变量定义*/
	
extern  RingBuff_t ringBuff;
extern RingBuff_t ringBuff_IMU;	
extern RingBuff_t ringBuff_USART1;
	
void ringBuff_Init(void);
u8 WriteRingBuff(u8 data);
u8 Read_ringBuff(u8 *rData);
void Offset(u8 number);
u16 Get_Buff_Data_Number_GPS(void);

void ringBuff_IMU_Init(void);
u8 Write_IMU_ringBuff(u8 data);
u8 Read_IMU_ringBuff(u8 *rData);
void Offset_IMU(u8 number);


void ringBuff_USART1_Init(void);
u8 Write_USART1_ringBuff(u8 data);
u8 Read_USART1_ringBuff(u8 *rData);
void Offset_USART1(u8 number);
u16 Get_Buff_Data_Number(void);
#endif

