#ifndef configuration
#define configuration
#include "stm32f10x.h"
#include "User\Delay\delay.h"
#include <stdio.h>
/*基本设置*/
#define USE_BRUSHED_MOTOR 1 //使用有刷电机时
/*基本定义*/
#ifndef BASIC
	#define BASIC
	/*DMA启动配置位*/
	#define USE_USART3_GPS_DMA_RX 1
	#define USE_USART1_COMM_DMA_TX 1	//主要用于synthesis_frame()函数中，若工作在Debug模式请将此位清零
	#define USE_USART1_COMM_DMA_RX 0	//DMA_Rx目前不可用
	/*当地磁偏角，转换为弧度*/
	#define Magnatic_Static_Bias (float)(8.0/(180*PI))	
	/*电机在0.1秒内走过的距离的最大值 （通过PID查误差的方法确定）*/
	#define MOTOR_MAX_SPEED 6
#endif
	
/*下位机有限状态机*/	
#ifndef STATE_MACHINE
	#define STATE_MACHINE
	/*初始化状态*/
	#define STATE_INITIALIZE 0
	/*程序正常运行（自主导航）状态*/
	#define STATE_NORMALLY_RUNNING 1
	/*GPS采点状态*/
	#define STATE_COLLECTING_POINTS 2
	/*手动遥控状态*/
	#define STATE_MANUAL_CTRL 3
	/*程序测试状态*/
	#define STATE_DEBUGGING 4	
#endif
	
/*宏函数定义*/
#ifndef MACRO_FUNCTION
	#define LED_BRINK() 		if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) GPIO_SetBits(GPIOE, GPIO_Pin_6);\
													else GPIO_ResetBits(GPIOE, GPIO_Pin_6)
														
	#define MISSION_LED_BRINK() 		if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) GPIO_SetBits(GPIOE, GPIO_Pin_5);\
													else GPIO_ResetBits(GPIOE, GPIO_Pin_5)
#endif

#ifndef IMU_MODULE_SELECT
/*
	GY901读取配置
	ACCESS_PERIPH如果是1，则MCU在读取数据时会将PERIPH数据考虑在内。否则，不予考虑。
*/
	#define GY901_nMPU6050 0  //imu模块选择编译，1表示使用GY901，0表示MPU6050
	#if GY901_nMPU6050
		#define ACCESS_Acceleration 1
		#define ACCESS_Angle_speed 1
		#define ACCESS_Angle 0
		#define ACCESS_Mag 1
	#endif	
#endif												
/*=========================全局常量配置==========================*/
#ifndef GLOBAL
#define GLOBAL
	#define TRUE 1
	#define FALSE 0
	#define PI 3.1415926
	#define G	(float)9.8	
	/*车轮直径*/
	#define WHEELDIAMETER (float)6.5		//cm		
#endif
/*==============================DATAFUSION配置===============================*/
#ifdef DATAFUSION
	#include <math.h>
	#include "User\IMU\IMU.h"
	#include "USER\GPS\GPS.h"
	#include "USER\COMPASS\COMPASS.h"
	// System constants
	#define sampleFreq 50 // sample frequency in Hz
	// estimated orientation quaternion elements with initial conditions
	float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; 
	volatile float psi=0,theta=0,fhi=0;//欧拉角，用于表示姿态。
	float yaw=0,pitch=0,roll=0;
#endif
/*=========================环形缓冲区配置==========================*/
#ifdef RINGBUFF
	#ifndef RINGBUFF_LEN
	/*=如果要改动缓冲区容量，则需要此处与相应传感器同时更改=*/
	#define LENGTH_OF_BUFF 200 //改的时候别忘了在ringbuff.h里也改
	u16 RINGBUFF_LEN=LENGTH_OF_BUFF;  //GPS环形缓冲区容量
	
	#endif
	/*环形缓冲区结构体定义*/
	typedef struct
	{
		volatile u16 Head;
		volatile u16 Tail;
		volatile u16 Length;
		volatile u8 Ring_Buff[LENGTH_OF_BUFF];
	}RingBuff_t;
	/*初始变量定义*/
	volatile RingBuff_t ringBuff;
	volatile RingBuff_t ringBuff_IMU;	
	volatile RingBuff_t ringBuff_USART1;	
#endif

/*=========================系统任务调度配置==========================*/
#ifdef LOOP
	#include "USER\IIC\IIC.h"
	#include "USER\Usart\USART.h"
	#include "USER\GPS\GPS.h"
	#include "USER\COMPASS\COMPASS.h"
	#include "USER\FILTER\FILTER.h"
	#include "USER\DATAFUSION\DATAFUSION.h"
	#include "USER\IMU\IMU.h"
	#include "User\TD\TD.h"
	#include "User\PID\PID.h"
	#include "User\Motor\Motor.h"
	#include "User\INPUT_CAPTURE\INPUT_CAPTURE.h"
	#include "User\PathPlanning\PathPlanning.h"
	#include "User\WDG\WDG.h"
	#define APB1DIV 2		//分频系数，用来调节时间片定时

	/*Madgwick滤波器参数设置*/
	#define BETA 3.5
	#define MadgwickGainDecendTime 3  //让增益在3秒内自动下降
	/*电机参数设置*/
	u8 MotorLockTimeAfterRaspiOffline = 2;   //定义电机在几秒内未收到上位机数据则锁定
	/*有限状态机驱动器*/
	u8 current_state = STATE_MANUAL_CTRL;//STATE_NORMALLY_RUNNING;
	#define CHANGE_STATE_TO(next_state) current_state = next_state
	/*电机使能标志位*/
	u8 Motor_Enabled = 0;
	extern void Initialize(void);
	u8 Analyze_Controller_Msg(void);
	/*时间片结构体定义*/
	typedef struct{	
		volatile u8 Hz200;
		volatile u8 Hz100;
		volatile u8 Hz50;
		volatile u8 Hz10;
		volatile u8 Hz5;
		volatile u8 Hz1;
	}TimeSlice;


	/*初始变量定义*/
	volatile u16 system_pulse;
	volatile TimeSlice Mission;
	volatile int16_t MadgwickGainAutoDecend=0;
	int16_t RaspiOfflineIndicator = 0;
#endif
/*==============================GPS配置===============================*/
#ifdef GPS
	#include "User\IIC\IIC.h"
	#include "User\RingBuff\ring_buff.h"
	#include "User\Usart\USART.h"
	#define MODEL_nVEHICLE_SEA 0 //模式配置，此位为1时，GPS动态模型配置为海上模型；否则为载具模型
	#define PVTdataout 1				//为1则使能相应输出；为0则失能输出。
	#define SVinfoOut 0
	#define DOPout 0
	typedef struct{	
		u8 fixtype;		//GPS锁定类型
		u8 fixFlags;	//GPS锁定类型标志位
		u8 numSV;			//用于导航解算的卫星数目
		int32_t lon;	//经度								 degree
		int32_t lat;	//维度								 degree
		int32_t velN;	//北东地坐标系向北速度 mm/s
		int32_t velE;	//北东地坐标系向东速度 mm/s
		int32_t velD;	//北东地坐标系向地速度 mm/s
		u32 sAcc;			//预估速度精度				 mm/s
		u32 hAcc;			//预估水平精度				 mm
		u16 pdop;			//方位精度因子  pdop<4,精度很好;4<pdop<7，精度可接受;pdop>7,pdop较差
	}pvtData;

	pvtData GPS_PVTData;
#endif
/*==============================上位机通信配置===============================*/
#ifdef COMM
#include "User\RingBuff\ring_buff.h"
#include "User\TD\TD.h"
#include <stdio.h>
#include "User\LoopSequence\Loop.h"
	u8 DMA_RX = USE_USART1_COMM_DMA_RX;
	u8 DMA_TX = USE_USART1_COMM_DMA_TX;
#endif
/*==============================IIC配置===============================*/
#ifdef IIC
	#define IIC_PORT GPIOC
	#define IIC_INITIAL_PORT RCC_APB2Periph_GPIOC
	#define SCL GPIO_Pin_12 
	#define SDA GPIO_Pin_11
#endif
/*=============================COMPASS配置============================*/
#ifdef COMPASS
	#include "User\IIC\IIC.h"
	#include "User\IMU\IMU.h"
	#include <math.h>
	#define REGISTERA 0x78		//测量频率：75Hz,低通滤波器启动（8个数据平均一次）
	#define REGISTERB 0X00		//Gauss增益：1090
	#define REGISTERMODE 0X00	//连续测量模式
	int16_t Local_mag_bias=Magnatic_Static_Bias;
#endif
/*==============================IMU配置===============================*/
#ifdef IMU
	#include "User\RingBuff\ring_buff.h"
	#include "User\IIC\IIC.h"
	#include "User\IMU\MPU6050.h"
	#include <math.h>
	#define FRAME_LENGTH 11	//表示GY901一帧数据的长度
	typedef struct{	
	volatile float channelX;
	volatile float channelY;
	volatile float channelZ;
	}Acceleration;

	typedef struct{	
	volatile float channelX;
	volatile float channelY;
	volatile float channelZ;
	}Angle_speed;
	
	typedef struct{	
	volatile float Roll;
	volatile float Pitch;
	volatile float Yaw;
	}Angle;
	
	typedef struct{	
	volatile int16_t channelX;
	volatile int16_t channelY;
	volatile int16_t channelZ;
	}Mag;
	
	/*
	说明：变量名带e的都表示Raw数据，未经任何处理。
				变量名前缀是p的表示Processed数据，表示已处理，可供使用。
	*/
	Acceleration eAcceleration,pAcceleration,acceFilter;
	
	Angle_speed eAngleSpeed,pAngleSpeed,anglespeedFilter;
	Angle_speed Gyro_Cancelation;//陀螺仪静差消除
	
	Angle eAngle,pAngle;
	
	Mag eCompass_Main,pCompass_Main;	
	Mag Compass_Hard_Iron_Cancellation;
#endif
/*==============================FILTER配置============================*/
#if defined FILTER 
	#include "User\IMU\IMU.h"	
	#include "USER\DATAFUSION\DATAFUSION.h"
	#include <math.h>
#endif
/*=========================TRANSMITDATA配置===========================*/
#ifdef TRANSMITDATA
	#include "User\DATAFUSION\DATAFUSION.h"
	#include "USER\Usart\USART.h"
	#include "USER\GPS\GPS.h"
	#include "User\RingBuff\ring_buff.h"
	#include "USER\WirelessUSART\WirelessPort.h"
	#include "User\LoopSequence\Loop.h"
	#define one_frame_length 22 //数据帧长度共18Byte
	u16 FRAMELENGTH = one_frame_length;
	u8 DATAFRAME[one_frame_length] = {0x73,0x63};
	typedef struct{	
	u16 yaw;
	u16 distance;
	//控制状态第7位：EndOfNav标志位,此位为1表示导航结束，停机。
	u8 control_status;
	}Route;
	
	Route routeToGo;
#endif
/*==============================MOTOR配置=============================*/
#ifdef MOTOR
#include "USER\PWM\pwm.h"
	#define FORWARD 1
	#define BACKWARD 2
	#define ENCODER_WIRE_NUMBER 1560
	#define LEFT_WHEEL 2
	#define RIGHT_WHEEL 1
	#define ALL_WHEEL 3
#endif
/*==============================PID配置===============================*/
#ifdef PID
	#include "USER\PWM\pwm.h"
	#include "USER\FILTER\FILTER.h"
	#include <math.h>
	#define Velocity_KP (float)12
	#define Velocity_KI (float)-3.5
	#define Velocity_KD (float)0

#endif
/*==============================PATHPLAN配置==========================*/
#ifdef PATHPLAN 
	#include "User\TD\TD.h"
	#include "User\Motor\Motor.h"
	#include "User\PID\PID.h"
	#include <math.h>
	#define CLOCKWISE 0
	#define COUNTER_CLOCKWISE 1
	
#endif

/*============================WIRELESS PORT配置========================*/
#ifdef WirelessPort 
	#define MD0 GPIO_Pin_11
	#define AUX GPIO_Pin_10
	#define GPIOPORT GPIOD
	#define RCC_GPIOPORT RCC_APB2Periph_GPIOD
#endif

/*========================遥控器配置====================*/
#ifdef INPUT_CATPURE 
#define CAPTURE_PERIOD (u16)19
#define CAPTURE_PRESCALER (u16)71
u32 CHANNEL_HIGHOUTPUT[4];
#endif

#endif
