#define COMM 1
#include "..\CONFIGURATION.h"

#if USE_USART1_COMM_DMA_RX
void Usart1_DMA_Rx_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel5);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ringBuff_USART1.Ring_Buff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = RINGBUFF_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8 位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 8 位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); 
	//使能DMA
	DMA_Cmd (DMA1_Channel5,ENABLE);
	
}
#endif

#if USE_USART1_COMM_DMA_TX

void Usart1_DMA_Tx_Config(u32 DMA_MemoryBaseAddr,u32 size)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel4);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = DMA_MemoryBaseAddr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = size;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8 位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 8 位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure); 
	//使能DMA
	DMA_Cmd (DMA1_Channel4,ENABLE);
}

void USART1_DMA_ENABLE_ONCE(u16 size)
{
	DMA_Cmd(DMA1_Channel4, DISABLE); //关闭 USART1 TX DMA1 所指示的通道
	DMA_SetCurrDataCounter(DMA1_Channel4,size);//设置 DMA 缓存的大小
	DMA_Cmd(DMA1_Channel4, ENABLE); //使能 USART1 TX DMA1 所指示的通道
}
#endif

void usart_init(u32 baudrate,u8 DMA_RX,u8 DMA_TX)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,ENABLE);
	//串口复位
	USART_DeInit(USART1);
	//GPIO模式设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//作TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //高速
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//作RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//浮空输入???
	GPIO_Init(GPIOA,&GPIO_InitStructure);			

	//开启接收时，设置接收中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//中断通道
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
	USART_Init(USART1,&USART_InitStructure);
	//开启#接收#中断
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	#if USE_USART1_COMM_DMA_RX
	/*DMA接收使能*/
	if(DMA_RX)
	{
		USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		USART_ITConfig(USART1,USART_IT_IDLE,ENABLE); //开启空闲中断
		Usart1_DMA_Rx_Config();
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); //开启DMA接收
	}
	#endif
	
	#if USE_USART1_COMM_DMA_TX
	/*DMA发送使能*/
	if(DMA_TX)
	{
		Usart1_DMA_Tx_Config((u32)DATAFRAME,(u32)FRAMELENGTH);
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	}
	#endif
	//使能串口
	USART_Cmd(USART1,ENABLE);
}

void USART1_IRQHandler(void)//startup_stm32f10x_hd.s文件中定义的USART1中断函数句柄
{
#if USE_USART1_COMM_DMA_RX
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
	{
		ringBuff_USART1.Length += (u16)RINGBUFF_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
		ringBuff_USART1.Tail += (u16)RINGBUFF_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
		ringBuff_USART1.Tail %= RINGBUFF_LEN;
			
		if(resolve_frame_from_Host_computer())
		{
			MISSION_LED_BRINK();
			RaspiOfflineIndicator = MotorLockTimeAfterRaspiOffline;
		}
		DMA_Cmd(DMA1_Channel5, DISABLE);
		/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
		DMA1_Channel5->CNDTR = RINGBUFF_LEN;
		//启动DMA
		DMA_Cmd(DMA1_Channel5, ENABLE);
			
		//清除中断标志位
		USART_ReceiveData(USART1);
	}
		
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		Write_USART1_ringBuff(USART_ReceiveData(USART1));
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
#else
	/*全局变量u32 USART_RX_STATUS  USART_RX_BUFF[] */
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		Write_USART1_ringBuff(USART_ReceiveData(USART1));
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
#endif
	/*如果需要用到USART接收，这里是个重点！*/
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE))
	{
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1,USART_FLAG_ORE);
	}
}

int fputc(int ch,FILE *f)
{
    USART1->SR;  //USART_GetFlagStatus(USART1, USART_FLAG_TC) 解决第一个字符发送失败的问题
    //一个一个发送字符
    USART_SendData(USART1, (unsigned char) ch);
    //等待发送完成
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
    
    return(ch);
}

u8 USART1_SENDDATA(u8 datain)
{
	u16 timer=0;
	USART_SendData(USART1, datain); //向串口1 发送数据
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET)
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

u8 WirelessUSART_Config(void)
{
	u8 const config[] = {'A','T','+','R','E','S','E','T','\r','\n'};
	u8 const closeEcho[] = {'A','T','E','0'};
	u8 counter=0;
	u8 dataIn;
	u8 portOnline=0;
	GPIO_SetBits(GPIOD,GPIO_Pin_11);
	ringBuff_USART1_Init();
	delay_ms(30);
	for(counter=0;counter<sizeof(closeEcho)/sizeof((u8)0);counter++)
	{
		USART1_SENDDATA(closeEcho[counter]);
		delay_ms(1);
	}
	delay_ms(100);
	for(counter=0;counter<20;counter++)
	{
		Read_USART1_ringBuff(&dataIn);
		if(dataIn == 'O')
		{
			portOnline = 1;
			break;
		}
	}
	
	if(portOnline)
	{
		for(counter=0;counter<sizeof(config)/sizeof((u8)0);counter++)
		{
			delay_ms(5);
			USART1_SENDDATA(config[counter]); 
		}
		for(counter=0;counter<10;counter++)
			{
				delay_ms(20);
				MISSION_LED_BRINK();//灯闪表示无线串口已连接
				delay_ms(20);
			}
	}
	GPIO_ResetBits(GPIOD,GPIO_Pin_11);
	delay_ms(50);
	return portOnline;
}
