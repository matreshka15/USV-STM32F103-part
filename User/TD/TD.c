#define TRANSMITDATA 1
#include "..\CONFIGURATION.h"

/*==============Eastar Protocol 0.2==================*/

/*
函数功能：将当前STM32所处状态变量值整合为一帧数据FRAMELENGTH
大端在前：0x1122发送，则接收时先接收0x11，再接收0x22
*/

void synthesis_frame(void)
{
	u8 counter;
	u8 CRC_validation = 0;
	uint32_t longtitude = (uint32_t)GPS_PVTData.lon;
	uint32_t latitude = (uint32_t)GPS_PVTData.lat;
	uint16_t Yaw = yaw;
	uint16_t Pitch = pitch;
	uint16_t Roll = roll;
	u8 control_field = 0;
	
	/*处理hAcc---水平精度*/
	uint32_t hAcc_in_dm = GPS_PVTData.hAcc / 1000 ;
	if(hAcc_in_dm >= 254) hAcc_in_dm = 0xff;
	/*根据数据自动调整control_field字段*/
	/*第7-6位：GPS锁定格式*/
	if(SignalFromGPS()!=5) 
	{
		control_field |= (SignalFromGPS())<<6;	//GPS数据有效
	}
	/*第5-4位：当前遥控状态*/
	control_field |= current_state << 4;
	
	/*开始发送数据*/
	//发送经度
	for(counter = 0;counter<4;counter++)
	{
		DATAFRAME[2+counter] = (longtitude & (0xff000000>>((counter)*8))) >> (8*(3-counter));
	}
	//发送纬度
	for(counter = 0;counter<4;counter++)
	{
		DATAFRAME[2+4+counter] = (latitude & (0xff000000>>((counter)*8))) >> (8*(3-counter));
	}
	//发送偏航角
	for(counter = 0;counter<2;counter++)
	{
		DATAFRAME[2+4+4+counter] = (Yaw & (0xff00>>((counter)*8))) >> (8*(1-counter));
	}	
	//发送俯仰角（带符号）
	for(counter = 0;counter<2;counter++)
	{
		DATAFRAME[2+4+4+2+counter] = (Pitch & (0xff00>>((counter)*8))) >> (8*(1-counter));
	}	
	//发送滚转角(带符号)
	for(counter = 0;counter<2;counter++)
	{
		DATAFRAME[2+4+4+2+2+counter] = (Roll & (0xff00>>((counter)*8))) >> (8*(1-counter));
	}	
	//发送水平精度预估
	DATAFRAME[2+4+4+2+2+2] = (u8)hAcc_in_dm;
	//带拓展的空字段
	for(counter = 0;counter<3;counter++)
	{
		DATAFRAME[2+4+4+2+2+2+1+counter] = 0x00;
	}	
	//控制字段
	DATAFRAME[2+4+4+2+2+2+4] = control_field;
	
	//发送数据
	for(counter=0;counter<sizeof(DATAFRAME)/sizeof((u8)0);counter++)
	{
		if(counter>=2&&counter<(FRAMELENGTH-1))
		{
			CRC_validation += DATAFRAME[counter];
		}
		else if(counter==(FRAMELENGTH-1))
		{
			DATAFRAME[(FRAMELENGTH-1)] = CRC_validation;
		}
		#if !USE_USART1_COMM_DMA_TX
		if(WirelessPort_Online)
		{
			if(!Read_AUX_Status())
				USART1_SENDDATA(DATAFRAME[counter]);
			
		}
		else
		{
			USART1_SENDDATA(DATAFRAME[counter]);
		}
		#endif
	}
	#if USE_USART1_COMM_DMA_TX
		if(WirelessPort_Online)
		{
			if(!Read_AUX_Status())
			{
				if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)
				{
					DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道 4 传输完成标志
					USART1_DMA_ENABLE_ONCE(FRAMELENGTH);
				}
			}
		}
		else
		{
			if(!DMA_GetCurrDataCounter(DMA1_Channel4))
			{
				if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)
				{
					DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道 4 传输完成标志
					USART1_DMA_ENABLE_ONCE(FRAMELENGTH);
				}//else USART1_DMA_ENABLE_ONCE(FRAMELENGTH);					
			}

		}
	#endif
}

u8 resolve_frame_from_Host_computer(void)
{
	u16 counter=0,counter1=0;
	u8 validationFlag=0;
	u8 dataIn=0;
	u8 temp=0;
	u8 sumup=0;
	u8 Header[] = {0x63,0x73};
	u8 CRCvalidate[17];//协议校验区总长度为16bytes
	if(Get_Buff_Data_Number() >= FRAMELENGTH)//先确保缓冲区内存储了足够多的数据
	{
		/*同步信息帧，寻找引导码*/
		for(counter=0;counter<=FRAMELENGTH-1;counter++)
		{
			Read_USART1_ringBuff(&dataIn);
			if(dataIn==Header[counter1])//环形缓冲区内的数据流与头文件一一比对
			{
				//printf("counter1=%d,dataIn=%d\n",counter1,dataIn);
				counter1++;
				if(counter1 >= (sizeof(Header)/sizeof((u8)0)))//如果头文件比对成功，则说明帧已同步
				{
					/*读取信息帧，进行CRC校验*/
						for(counter=0;counter<(sizeof(CRCvalidate)/sizeof((u8)0));counter++)
						{
							Read_USART1_ringBuff(&CRCvalidate[counter]);
							sumup+=CRCvalidate[counter];
						}
						
						Read_USART1_ringBuff(&temp);
						if(sumup==temp) 
						{
							validationFlag=1;
							//printf("\nCRC passed!\n");
						}
						break;
				}
			}else continue;
			
		}
	}
	if(validationFlag)
	{
		/*读取路径偏航角*/
		routeToGo.yaw = 0;
		for(counter=0;counter<2;counter++)
		{	
			temp = CRCvalidate[counter];
			routeToGo.yaw += (((u16)temp)<<(8*(1-counter)));
		}	
		/*读取路径距离*/
		routeToGo.distance = CRCvalidate[2];
		routeToGo.distance= 0;
		for(counter=0;counter<2;counter++)
		{	
			temp = CRCvalidate[counter+2];
			routeToGo.distance += (((u16)temp)<<(8*(1-counter)));
		}			
		/*读取控制状态*/
		routeToGo.control_status = CRCvalidate[4];
		//printf("YAW = %d,distance = %d,control_status=%d\n",routeToGo.yaw,routeToGo.distance,routeToGo.control_status);
		return 1;
	}
	return 0;
}	

