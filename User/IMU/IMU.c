#define IMU 1
#include "..\CONFIGURATION.h"


#if GY901_nMPU6050
/*===GY901模块参数===*/    //MPU6050模块配置在下面。
const u8 IMU_output_cfg[] = {0xFF,0xAA,0x02,0x16,0x00};	//IMU输出：加速度、角速度、磁场信息。上电默认配置。
const u8 IMU_rate_cfg[] = {0xFF,0xAA,0x03,0x06,0x00};	//设置回传速率：0x08  50Hz  0X06  10Hz
const u8 IMU_baud_cfg[] = {0xFF,0xAA,0x04,0x09,0x00};	//设置串口波特率921600，上电初始为9600
const u8 IMU_save_cfg[] = {0xFF,0xAA,0x00,0x00,0x00};	//保存当前配置



u8 dataFromGY901(void)
{
	u8 data[FRAME_LENGTH];
	u8 sum;
	u16 counter,counter1=0,counter2;
	u8 Acceleration_flag=0,AngleSpeed_flag=0,Angle_flag=0,Compass_Main_flag=0;
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);//在处理数据时关闭后面数据的输入，以免新数据冲掉旧数据
	for(counter=0;counter<RINGBUFF_LEN;counter++)//遍历整个环形缓冲区
	{
		if(Read_IMU_ringBuff(&data[counter1]))//环形缓冲区非空
		{
			if(data[0]==0x55)//标志一帧已开始
			{
				counter1++;
				if(counter1==FRAME_LENGTH) 
				{	
					counter1 = 0; //一帧数据已处理完
					if((Acceleration_flag||!ACCESS_Acceleration)\
							&&(AngleSpeed_flag||!ACCESS_Angle_speed)\
							&&(Angle_flag||!ACCESS_Angle)\
							&&(Compass_Main_flag||!ACCESS_Mag)) return 1;//所有数据读取完毕
					switch(data[1])
					{
						case 0x51:
						{
							if(!Acceleration_flag)//仅当加速度数据未处理过时
							{
								/*CRC校验*/
								sum = 0;
								for(counter2=0;counter2<FRAME_LENGTH;counter2++)
								{
									sum+= data[counter2];
								}
								if(sum==data[FRAME_LENGTH-1])
								{
									/*校验通过*/
									eAcceleration.channelX = (float)(((int16_t)data[3]<<8)|data[2])/(float)(32768*16*G);
									eAcceleration.channelY = (float)(((int16_t)data[5]<<8)|data[4])/(float)(32768*16*G);
									eAcceleration.channelX = (float)(((int16_t)data[7]<<8)|data[6])/(float)(32768*16*G);
									Acceleration_flag = 1;
								}
								else continue;
							}
							else continue;
						}
						case 0x52:
						{
							if(!AngleSpeed_flag)//仅当加速度数据未处理过时
							{
								/*CRC校验*/
								sum = 0;
								for(counter2=0;counter2<FRAME_LENGTH;counter2++)
								{
									sum+= data[counter2];
								}
								if(sum==data[FRAME_LENGTH-1])
								{
									/*校验通过*/
									eAngleSpeed.channelX = (float)(((int16_t)data[3]<<8)|data[2])/(float)(32768*2000);
									eAngleSpeed.channelY = (float)(((int16_t)data[5]<<8)|data[4])/(float)(32768*2000);
									eAngleSpeed.channelX = (float)(((int16_t)data[7]<<8)|data[6])/(float)(32768*2000);
									AngleSpeed_flag = 1;
								}
								else continue;								
							}
							else continue;						
						}
						case 0x53:
						{
							if(!Angle_flag)//仅当加速度数据未处理过时
							{
								/*CRC校验*/
								sum = 0;
								for(counter2=0;counter2<FRAME_LENGTH;counter2++)
								{
									sum+= data[counter2];
								}
								if(sum==data[FRAME_LENGTH-1])
								{
									/*校验通过*/
									eAngle.Roll = (float)(((int16_t)data[3]<<8)|data[2])/(float)(32768*180);
									eAngle.Pitch = (float)(((int16_t)data[5]<<8)|data[4])/(float)(32768*180);
									eAngle.Yaw = (float)(((int16_t)data[7]<<8)|data[6])/(float)(32768*180);
									Angle_flag = 1;
								}
								else continue;											
							}
							else continue;						
						}
						case 0x54:
						{
							if(!Compass_Main_flag)//仅当加速度数据未处理过时
							{
								/*CRC校验*/
								sum = 0;
								for(counter2=0;counter2<FRAME_LENGTH;counter2++)
								{
									sum+= data[counter2];
								}
								if(sum==data[FRAME_LENGTH-1])
								{
									/*校验通过*/
									eCompass_Main.channelX = (((int16_t)data[3]<<8)|data[2]);
									eCompass_Main.channelY = (((int16_t)data[5]<<8)|data[4]);
									eCompass_Main.channelZ = (((int16_t)data[7]<<8)|data[6]);
									Compass_Main_flag = 1;
								}
								else continue;	/*校验不通过*/								
							}
							else continue;						
						}
						default: 
						{//如果紧跟0x55后面的Byte不在以上范围内，则重新同步数据帧。
							counter1 = 0;
							continue;
						}
					}
				} else continue;
			} else continue;
		}
		else return 0;//环形缓冲区空
	}
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	return 0xff;
}

void ringBuff_USART2_TEST(void)
{
	u8 receivedData,counter;
	for(counter=0;counter<FRAME_LENGTH;counter++)
	{
		if(Read_IMU_ringBuff(&receivedData))
		{
			printf("%c",receivedData);
		}//else continue;
	}

}
void usart2_init(u32 baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//使能外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	//串口复位
	USART_DeInit(USART2);
	//GPIO模式设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//作TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  //高速
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//作RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//浮空输入???
	GPIO_Init(GPIOA,&GPIO_InitStructure);			

	//开启接收时，设置接收中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			//中断通道
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
	USART_Init(USART2,&USART_InitStructure);
	
	//开启#接收#中断
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
	//使能串口
	USART_Cmd(USART2,ENABLE);
}

void USART2_IRQHandler(void)//startup_stm32f10x_hd.s文件中定义的USART3中断函数句柄
{
	//u8 datain;
	if(USART_GetITStatus(USART2,USART_IT_RXNE))
	{
		Write_IMU_ringBuff(USART_ReceiveData(USART2));
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
	
	/*如果需要用到USART接收，这里是个重点！*/
	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE))
	{
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2,USART_FLAG_ORE);
	}
}

u8 IMU_SENDDATA(u8 datain)
{
	u16 timer=0;
	USART_SendData(USART2, datain); //向串口2/IMU 发送数据
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET)
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


u8 IMU_CFG_OUTPUT(void)
{
	u8 counter;
	u8 engagedMemory=0;
	IMU_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	engagedMemory=sizeof(IMU_output_cfg);//配置输出数据
	for(counter=0;counter<engagedMemory;counter++)
	{
		if(!IMU_SENDDATA(IMU_output_cfg[counter]))
		{
			return 0;
		}
	}
	IMU_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	engagedMemory=sizeof(IMU_save_cfg);//保存配置
	for(counter=0;counter<engagedMemory;counter++)
	{
		if(!IMU_SENDDATA(IMU_save_cfg[counter]))
		{
			return 0;
		}
	}
	return 1;
}

u8 IMU_CFG_RATE(void)
{
	u8 counter;
	u8 engagedMemory=0;
	IMU_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	engagedMemory=sizeof(IMU_rate_cfg);//配置输出数据
	for(counter=0;counter<engagedMemory;counter++)
	{
		if(!IMU_SENDDATA(IMU_rate_cfg[counter]))
		{
			return 0;
		}
	}
	IMU_SENDDATA(0x0A);//发送一个冗余字符，防止错位
	engagedMemory=sizeof(IMU_save_cfg);//保存配置
	for(counter=0;counter<engagedMemory;counter++)
	{
		if(!IMU_SENDDATA(IMU_save_cfg[counter]))
		{
			return 0;
		}
	}	
	return 1;
}

#elif !GY901_nMPU6050
/*==========MPU6050模块配置=========*/
void MPU6050_Write_Reg(uint8_t regAddr, uint8_t regData)
{
	IIC_Start();
	IIC_Send_Byte(DEV_ADDR<<1|0x00);
	IIC_Wait_Ack();
	IIC_Send_Byte(regAddr);
	IIC_Wait_Ack();
	IIC_Send_Byte(regData);
	IIC_Wait_Ack();
	IIC_Stop();	
}

u8 MPU6050_Read_Reg(uint8_t regAddr)
{
	u8 regData;
	IIC_Start();
	IIC_Send_Byte(DEV_ADDR<<1);
	IIC_Wait_Ack();
	IIC_Send_Byte(regAddr);
	IIC_Wait_Ack();
	IIC_Stop();
	IIC_Start();
	IIC_Send_Byte(DEV_ADDR<<1|0x01);
	IIC_Wait_Ack();
	regData = IIC_Read_Byte(0);
	IIC_Stop();
	return regData;
}

int16_t MPU6050_Get_Data(uint8_t regAddr)
{
    u8 Data_H, Data_L;
    int16_t data;
    
    Data_H = MPU6050_Read_Reg(regAddr);
    Data_L = MPU6050_Read_Reg(regAddr + 1);
    data = (Data_H << 8) | Data_L;  // 合成数据 
    return data;
}
u8 verifyMPU6050(void)
{
	u8 receiveData=0xFF;
	receiveData = MPU6050_Read_Reg(WHO_AM_I);
	if(receiveData==0x68) return 1;
	else return 0;
}

void MPU6050_Init(void)
{   
    MPU6050_Write_Reg(PWR_MGMT_1, 0x00);    //解除休眠状态     
    MPU6050_Write_Reg(SMPLRT_DIV, 0x04);    //陀螺仪采样率，典型值：0x04(200Hz)     
    MPU6050_Write_Reg(CONFIG, 0x04);        //低通滤波频率，典型值：0x06(5Hz)     
    MPU6050_Write_Reg(GYRO_CONFIG, 0x00);   //陀螺仪自检及测量范围，典型值：0x00(不自检，250deg/s)     
    MPU6050_Write_Reg(ACCEL_CONFIG, 0x01);  //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) 
}

void dataFrom6050(void)
{
	eAngleSpeed.channelX = (float)(MPU6050_Get_Data(GYRO_XOUT_H)/(131*57.3));//131.072*PI/180优化为左边的值。将度数转化为弧度/s
	eAngleSpeed.channelY = (float)(MPU6050_Get_Data(GYRO_YOUT_H)/(131*57.3));
	eAngleSpeed.channelZ = (float)(MPU6050_Get_Data(GYRO_ZOUT_H)/(131*57.3));
	eAcceleration.channelX = (float)MPU6050_Get_Data(ACCEL_XOUT_H)/16384;
	eAcceleration.channelY = (float)MPU6050_Get_Data(ACCEL_YOUT_H)/16384;
	eAcceleration.channelZ = (float)MPU6050_Get_Data(ACCEL_ZOUT_H)/16384;
}

/*发送由磁力计计算的偏航角*/

void SendYawFromCompass(void)
{
	float fhi_local;
	fhi_local = PI-atan2((float)pCompass_Main.channelY,(float)pCompass_Main.channelX);
	printf("yaw=%d\n",(int16_t)((fhi_local)*180/PI - Magnatic_Static_Bias));
}

void print_IMU_data(void)
{
	printf("\n====IMU DATA====");
	printf("\nACCEL_X: %.2f ",eAcceleration.channelX );
  printf("ACCEL_Y: %.2f ", eAcceleration.channelY);
  printf("ACCEL_Z: %.2f\n",eAcceleration.channelZ);
	//131.072代表1deg/s
	printf("GYRO_X: %.2f ", eAngleSpeed.channelX);
  printf("GYRO_Y: %.2f ", eAngleSpeed.channelY);
  printf("GYRO_Z: %.2f\n", eAngleSpeed.channelZ);	
	printf("MAG_X: %d ", eCompass_Main.channelX);
  printf("MAG_Y: %d ", eCompass_Main.channelY);
  printf("MAG_Z: %d\n", eCompass_Main.channelZ);	
	printf("================\n");
}
/*
陀螺仪静漂噪音消除，开机时读取100个数据，取平均值作为噪音。
*/
void Caliberate_Gyro(u16 number_of_dataset)
{
	u16 counter;
	for(counter=0;counter<number_of_dataset;counter++)
	{
		dataFrom6050();
		delay_ms(2);
		Gyro_Cancelation.channelX +=eAngleSpeed.channelX;
		Gyro_Cancelation.channelY +=eAngleSpeed.channelY;
		Gyro_Cancelation.channelZ +=eAngleSpeed.channelZ;
	}
	Gyro_Cancelation.channelX /= (float)number_of_dataset;
	Gyro_Cancelation.channelY /= (float)number_of_dataset;
	Gyro_Cancelation.channelZ /= (float)number_of_dataset;
}

/*
磁力计静漂噪音消除，开机时读取100个数据，取平均值作为噪音。
*/
void Caliberate_Mag(void)
{
	int16_t channel_X_Max=500,channel_X_Min=-1051;
	int16_t channel_Y_Max=626,channel_Y_Min=-721;
	int16_t channel_Z_Max=1043,channel_Z_Min=190;
	
	Compass_Hard_Iron_Cancellation.channelX = (channel_X_Max+channel_X_Min)/2;
	Compass_Hard_Iron_Cancellation.channelY = (channel_Y_Max+channel_Y_Min)/2;
	Compass_Hard_Iron_Cancellation.channelZ = (channel_Z_Max+channel_Z_Min)/2;
}

#endif
