#include "inclusion.h"
/*初始化宏定义*/
#define USART1_BAUDRATE 921600
#define GPS_BAUDRATE_CONFIG 115200

void Initialize(void)
{
	/*==========初始化===========*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置 NVIC 中断分组 2:2 位
	delay_init(SystemCoreClock);
	TIM5_STSTEM_PULSE_Init();
	GPIOPin_Init();
	IIC_Init();
	ringBuff_Init();
	ringBuff_IMU_Init();
	//配置电机引脚
	Motor_IN_Pin_Init();
	Motor_Brake(ALL_WHEEL);
	/*启动电机*/
	Servo_Set_Angle(0);
	delay_ms(700); //延时，等待外设全部启动
	usart_init(USART1_BAUDRATE,0,0);//初始化USART1时不使能DMA
	/*=========配置无线串口=========*/
	WirelessPort_Online = 0;//不使用无线串口
	/*=====配置GPS====*/
	GPSDATA_Init();
	//usart3_init(GPS_BAUDRATE_CONFIG);
	//delay_ms(5);
	//UBX_CFG_USART(115200);
	delay_ms(5);
	usart3_init(115200);//GPS串口波特率设置
	delay_ms(5);
	//修改GPS波特率
	UBX_CFG_USART(GPS_BAUDRATE_CONFIG);
	
	delay_ms(5);usart3_init(GPS_BAUDRATE_CONFIG);//配置成功,则GPS波特率被修改
	delay_ms(5);UBX_CFG_RATE();
	delay_ms(5);UBX_CFG_DATAOUT();
	delay_ms(5);UBX_CFG_MODEL();
	/*配置Compass*/
	if(verifyCompass()) 
	{
		printf("\nCompass Online!\n");
		cfgCompassReg();
	}
	MPU6050_Init();
	if(verifyMPU6050()) printf("\nMPU6050 Online!\n");
	/*执行传感器的校准程序*/
	Caliberate_Gyro(100);
	Caliberate_Mag();
	/*获取初始姿态*/
	receiveComData();
	dataFrom6050();
	filter_IMU_data(0);
	theta = atan2(pAcceleration.channelY,pAcceleration.channelZ);//pitch
	psi = atan2(pAcceleration.channelX,pAcceleration.channelZ);//roll
	fhi = 3.1415926-atan2((float)pCompass_Main.channelY,(float)pCompass_Main.channelX);//yaw
	fhi -= Local_mag_bias;
	//printf("Currnet Initial Yaw:%f\nROLL:%f,\npitch:%f\n",fhi*180/3.1415926,psi*180/3.1415926,theta*180/3.1415926);
	/*将初始姿态转换为初始四元数*/
	SEq_4 = cos(fhi/2)*cos(theta/2)*cos(psi/2)+sin(fhi/2)*sin(theta/2)*sin(psi/2);
	SEq_1 = sin(fhi/2)*cos(theta/2)*cos(psi/2)-cos(fhi/2)*sin(theta/2)*sin(psi/2);
	SEq_2 = cos(fhi/2)*sin(theta/2)*cos(psi/2)+sin(fhi/2)*cos(theta/2)*sin(psi/2);
	SEq_3 = cos(fhi/2)*cos(theta/2)*sin(psi/2)-sin(fhi/2)*sin(theta/2)*cos(psi/2);
	Convert_Quaternion_To_Euler();
	//printf("Processed Initial Yaw:%f\n",fhi*180/3.1415926);

	/*配置传感器时不启动DMA
	最后进入LOOP之前启动DMA*/
	usart_init(USART1_BAUDRATE,DMA_RX,DMA_TX);
	/*启动遥控器信号捕获*/
	TIM3_Capture_Init();
	//配置看门狗
	WDG_Initialize(7,240);
	//最后启动看门狗
	WDG_Start();
 }
