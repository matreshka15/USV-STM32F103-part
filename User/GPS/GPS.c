#define GPS 1
#include "..\CONFIGURATION.h"
/*################################
说明：此控制协议未使用M8N的响应帧Ack
##################################*/

//USART只允许UBX协议输入输出,波特率
const u8 usart921600_cfg[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x10,0x0E,\
												0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x13,0x1E};

const u8 usart460800_cfg[] ={0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x08,0x07,\
												0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x04,0x80};
const u8 usart115200_cfg[] = 	{0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,\
												0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xB8,0x42};
//配置GPS使用UTC时间，输出频率
const u8 rate10Hz_cfg[] ={0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x00,0x00,0x79,0x10};
const u8 rate5Hz_cfg[] ={0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x00,0x00,0xDD,0x68};
//配置动态模型为海上模式
const u8 nav5_cfg_sea[] ={0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x05,0x03,0x00,0x00,0x00,0x00,0x10,0x27,\
											0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00, \
											0x10,0x27,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x4F,0xCB};
//配置动态模型为载具（汽车）模式
const u8 nav5_cfg_auto[] ={0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x04,0x03,0x00,0x00,0x00,0x00,0x10,0x27,\
											0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x00,0x00,0x00, \
											0x10,0x27,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x4E,0xA9};
//配置UBX输出PVT数据
const u8 pvt_cfg_output[] ={0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};	
//配置UBX输出DOP数据
const u8 dop_cfg_output[] ={0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x04,0x00,0x01,0x00,0x00,0x00,0x00,0x15,0xCC};
//配置UBX输出SVINFO
const u8 svinfo_cfg_output[] ={0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x30,0x00,0x01,0x00,0x00,0x00,0x00,0x41,0x00};


//向GPS模块发送数据
u8 GPS_SENDDATA(u8 datain)
{
	u16 timer=0;
	USART_SendData(USART3, datain); //向串口3/GPS 发送数据
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET)
	{
		timer++;
		if(timer>0xff) 
		{
			timer = 0;
			return 0;
		}
	}
	return 1;
}

/*=========================GPS回传数据读取函数==========================*/
/*这里只是初步实现了GPS信号是否可用的判断，即如果锁定类型不为*未锁定*，则GPS已锁定，可用。
----模块后期需要改善----*/
u8 SignalFromGPS(void)
{
	//u8 eph = (GPS_PVTData.hAcc/100);	//eph为水平精度，单位为米
	return GPS_PVTData.fixtype;
}



void DisplayDataGPS(void)
{
	u8 datain;
	while(Read_ringBuff(&datain))
		printf("%c",datain);
}
 u8 pvtFromGPS(void)
{//注意！这里dop'数据是被放大了100倍后的数据。实际数据返回值将dop'/100=pDop 
	u8 dataIn;
	u8 counter1=0;
	u8 validationFlag=0;
	u16 counter=0;
	u8 temp=0;
	const u8 Header[] = {0xB5,0x62,0x01,0x07,0x5c,0x00};
	if(Get_Buff_Data_Number_GPS()>=100)
	{
		for(counter=0;counter<=100;counter++)
		{
			if(Read_ringBuff(&dataIn))//如果环形缓冲区非空
			{
					if(dataIn==Header[counter1])//环形缓冲区内的数据流与头文件一一比对
					{
						counter1++;
						if(counter1>= (sizeof(Header)/sizeof((u8)0)) )//如果头文件比对成功，则说明帧已同步
						{
							validationFlag = 1;
							break;
						}
					}else 
					{
						;//GPS_PVTData.fixtype = 0;
					}
			}else break;
		}
	}
	
	if(validationFlag)
	{
		Offset(20);//环形缓冲区偏移 n Byte，直接偏移到要读取的pDOP数据处
		Read_ringBuff(&GPS_PVTData.fixtype);
		Read_ringBuff(&GPS_PVTData.fixFlags);
		Offset(1);
		Read_ringBuff(&GPS_PVTData.numSV);
		/*读取经度*/
		GPS_PVTData.lon = 0;
		for(counter=0;counter<4;counter++)
		{	
			Read_ringBuff(&temp);
			GPS_PVTData.lon += ((int32_t)temp<<(8*counter));
		}
		/*读取纬度*/
		GPS_PVTData.lat = 0;
		for(counter=0;counter<4;counter++)
		{
			Read_ringBuff(&temp);
			GPS_PVTData.lat += ((int32_t)temp<<(8*counter));
		}
		Offset(8);
		/*Horizontal accuracy estimate*/
		GPS_PVTData.hAcc = 0;
		for(counter=0;counter<4;counter++)
		{
			Read_ringBuff(&temp);
			GPS_PVTData.hAcc += ((u32)temp<<(8*counter));
		}
		Offset(4);
		/*读取北东地向东速度*/
		GPS_PVTData.velE = 0;
		for(counter=0;counter<4;counter++)
		{	
			Read_ringBuff(&temp);
			GPS_PVTData.velE += ((int32_t)temp<<(8*counter));
		}		
		/*读取北东地向北速度*/
		GPS_PVTData.velN = 0;
		for(counter=0;counter<4;counter++)
		{	
			Read_ringBuff(&temp);
			GPS_PVTData.velN += ((int32_t)temp<<(8*counter));
		}		
		/*读取北东地向地速度*/
		GPS_PVTData.velD = 0;
		for(counter=0;counter<4;counter++)
		{
			Read_ringBuff(&temp);
			GPS_PVTData.velD += ((int32_t)temp<<(8*counter));
		}				
		Offset(8);
		/*读取预估速度精度*/
		GPS_PVTData.sAcc = 0;
		for(counter=0;counter<4;counter++)
		{
			Read_ringBuff(&temp);
			GPS_PVTData.sAcc += ((u32)temp<<(8*counter));
		}				
		Offset(4);
		/*读取pDop*/
		GPS_PVTData.pdop = 0;
		for(counter=0;counter<2;counter++)
		{	
			Read_ringBuff(&temp);
			GPS_PVTData.pdop += ((u16)temp<<(8*counter));
			GPS_PVTData.pdop /= 100;//将pdop Scale down 100倍
		}	
		return 1;
	}
	return 0;							//数据出错
}
/*=========================GPS的配置函数================================*/
u8 UBX_CFG_USART(u32 baudrate)
{
	u8 counter;
	u8 engagedMemory=0;
	GPS_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	switch(baudrate)
	{
		case 115200:
		{
			engagedMemory=sizeof(usart115200_cfg)/sizeof((u8)0);
			for(counter=0;counter<engagedMemory;counter++)
			{
				if(!GPS_SENDDATA(usart115200_cfg[counter]))
				{
					return 0;
				}
			}
			return 1;
		}
		case 921600:
		{
			engagedMemory=sizeof(usart921600_cfg)/sizeof((u8)0);
			for(counter=0;counter<engagedMemory;counter++)
			{
				if(!GPS_SENDDATA(usart921600_cfg[counter]))
				{
					return 0;
				}
			}
			return 1;
		}
		case 460800:
		{
			engagedMemory=sizeof(usart460800_cfg)/sizeof((u8)0);
			for(counter=0;counter<engagedMemory;counter++)
			{
				if(!GPS_SENDDATA(usart460800_cfg[counter]))
				{
					return 0;
				}
			}
			return 1;
		}
		default: return 0;
	}
}

u8 UBX_CFG_RATE(void)
{
	u8 counter;
	u8 engagedMemory=0;
	GPS_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	engagedMemory=sizeof(rate5Hz_cfg)/sizeof((u8)0);
	for(counter=0;counter<engagedMemory;counter++)
	{
		if(!GPS_SENDDATA(rate5Hz_cfg[counter]))
		{
			return 0;
		}
	}
	return 1;
}

u8 UBX_CFG_DATAOUT(void)
{
	u8 counter;
	u8 engagedMemory=0;
	GPS_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	if(PVTdataout)
	{
		engagedMemory=sizeof(pvt_cfg_output)/sizeof((u8)0);
		for(counter=0;counter<engagedMemory;counter++)
		{
			if(!GPS_SENDDATA(pvt_cfg_output[counter]))
			{
				return 0;
			}
		}
	}
	if(DOPout)
	{
		engagedMemory=sizeof(dop_cfg_output)/sizeof((u8)0);
		for(counter=0;counter<engagedMemory;counter++)
		{
			if(!GPS_SENDDATA(dop_cfg_output[counter]))
			{
				return 0;
			}
		}			
	}
	if(SVinfoOut)
	{
		engagedMemory=sizeof(svinfo_cfg_output)/sizeof((u8)0);	
		for(counter=0;counter<engagedMemory;counter++)
		{
			if(!GPS_SENDDATA(svinfo_cfg_output[counter]))
			{
				return 0;
			}
		}			
	}
	
	return 1;
}

u8 UBX_CFG_MODEL(void)
{
	u8 counter;
	u8 engagedMemory=0;
	GPS_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	switch(MODEL_nVEHICLE_SEA)
	{
			case 0:
			{
				engagedMemory=sizeof(nav5_cfg_auto)/sizeof((u8)0);
				for(counter=0;counter<engagedMemory;counter++)
				{
					if(!GPS_SENDDATA(nav5_cfg_auto[counter]))
					{
						return 0;
					}
				}
				break;
			}
			default:
			{
				engagedMemory=sizeof(nav5_cfg_sea)/sizeof((u8)0);
				for(counter=0;counter<engagedMemory;counter++)
				{
					if(!GPS_SENDDATA(nav5_cfg_sea[counter]))
					{
						return 0;
					}
				}	
				break;				
			}
	}

	return 1;
}

void ringBuff_USART3_TEST(void)
{
	u8 receivedData;
	if(Read_ringBuff(&receivedData))
	{
		GPS_SENDDATA(receivedData);
	}
}

/*===============================底层驱动==================================*/
#if USE_USART3_GPS_DMA_RX
void Usart3_DMA_Rx_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel3);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ringBuff.Ring_Buff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = RINGBUFF_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8 位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 8 位
	//DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure); 
	//使能DMA
	DMA_Cmd (DMA1_Channel3,ENABLE);
	
}
#endif
void usart3_init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	//串口复位
	USART_DeInit(USART3);
	//GPIO模式设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//作TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //高速xx
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//作RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//浮空输入???
	GPIO_Init(GPIOB,&GPIO_InitStructure);			

	//开启接收时，设置接收中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;			//中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;		//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	//初始化串口
	USART_InitStructure.USART_BaudRate = baudrate;	//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//字长为8
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个终止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode =USART_Mode_Rx|USART_Mode_Tx; //收发模式
	USART_Init(USART3,&USART_InitStructure);
	
	#if USE_USART3_GPS_DMA_RX 
		USART_ITConfig(USART3,USART_IT_IDLE,ENABLE); //开启空闲中断
		/*使能串口DMA*/
		Usart3_DMA_Rx_Config();
		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); //开启DMA接收
	#else
	//开启#接收#中断
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	#endif
	//使能串口
	USART_Cmd(USART3,ENABLE);
}

void USART3_IRQHandler(void)//startup_stm32f10x_hd.s文件中定义的USART3中断函数句柄
{
#if USE_USART3_GPS_DMA_RX
	if(USART_GetITStatus(USART3,USART_IT_IDLE)!=RESET)
	{
		ringBuff.Length += (u16)RINGBUFF_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);
		ringBuff.Tail += (u16)RINGBUFF_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);
		ringBuff.Tail %= RINGBUFF_LEN;
		DMA_Cmd(DMA1_Channel3, DISABLE);	
		pvtFromGPS();
		/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		DMA1_Channel3->CNDTR = RINGBUFF_LEN;
		//启动DMA
		DMA_Cmd(DMA1_Channel3, ENABLE);
			
		//清除中断标志位
		USART_ReceiveData(USART3);
	}
	
#else
	if(USART_GetITStatus(USART3,USART_IT_RXNE))
	{
		WriteRingBuff(USART_ReceiveData(USART3));//GPS数据放入环形缓冲区
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
#endif
	/*如果需要用到USART接收，这里是个重点！*/
	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE))
	{
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3,USART_FLAG_ORE);
	}
}



void GPSDATA_Init(void)
{
	GPS_PVTData.fixtype = 0;
	GPS_PVTData.fixFlags = 0;
	GPS_PVTData.numSV = 0;
	GPS_PVTData.lon = 0;
	GPS_PVTData.lat = 0;
	GPS_PVTData.velN = 0;
	GPS_PVTData.velE = 0;
	GPS_PVTData.velD = 0;
	GPS_PVTData.sAcc = 0;
	GPS_PVTData.hAcc = 0;
	GPS_PVTData.pdop = 0;
}




