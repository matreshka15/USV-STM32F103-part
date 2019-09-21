#define MOTOR 1
#include "..\CONFIGURATION.h"
static int Round_Left = 0,Round_Right = 0;	
static u8 counterForRound_left=0,counterForRound_right=0;
static u8 last_direction_left = 0,last_direction_right = 0;
/*Encoder Configurate*/
void Encoder_Init_TIM2(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器4的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PB端口时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);                          //根据设定参数初始化GPIOA

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_WIRE_NUMBER; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure); //将结构体中的内容缺省输入
  TIM_ICInitStructure.TIM_ICFilter = 10;//测试一下6、10
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//运行更新中断
	
	//中断优先级 NVIC 设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM5 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //先占优先级 1 级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级 1 级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure); //初始化 NVIC 寄存器
  //Reset counter
  TIM_SetCounter(TIM2,0); //TIM2->CNT=0
  TIM_Cmd(TIM2, ENABLE); 
}

void Encoder_Init_TIM4(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);                          //根据设定参数初始化GPIO

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_WIRE_NUMBER; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure); //将结构体中的内容缺省输入
  TIM_ICInitStructure.TIM_ICFilter = 10;//测试一下6、10
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//运行更新中断
	
	//中断优先级 NVIC 设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //TIM 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //先占优先级 1 级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级 1 级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure); //初始化 NVIC 寄存器
  //Reset counter
  TIM_SetCounter(TIM4,0); //TIM4->CNT=0
  TIM_Cmd(TIM4, ENABLE); 
}

int Read_Encoder(u8 wheel)
{
	int Encoder_TIM2,Encoder_TIM4,Direction;    
	switch(wheel)
	{
		case RIGHT_WHEEL:
		{
			Encoder_TIM2= TIM2 -> CNT;  
			Direction = (TIM2->CR1&0X10)>>4;
			if(Direction==0)//正转
			{
				Encoder_TIM2 = Encoder_TIM2;
			}
			else
			{
				Encoder_TIM2 =  Encoder_TIM2 - ENCODER_WIRE_NUMBER;
			}
			TIM2 -> CNT=0;
			return Encoder_TIM2;
		}
		case LEFT_WHEEL:
		{
			Encoder_TIM4= TIM4 -> CNT;  
			Direction = (TIM4->CR1&0X10)>>4;
			if(Direction==0)//正转
			{
				Encoder_TIM4 = Encoder_TIM4;
			}
			else
			{
				Encoder_TIM4 =  Encoder_TIM4 - ENCODER_WIRE_NUMBER;
			}
			TIM4 -> CNT=0;
			return Encoder_TIM4;
		}
	}
	return 0xff;
}

void TIM2_IRQHandler(void)
{
		static u8 direction = 0; 
		u8 direction_register;
		//last_direction_right = direction;//正
		direction = (TIM2->CR1&0X10)>>4;//反
		direction_register = direction;//反
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//溢出中断
    { 
			if(direction != last_direction_right) 
			{
				Round_Right = 0;
				direction = last_direction_right;//正
				counterForRound_right=0;
				last_direction_right = direction_register;//反
			}
			if(direction == 0)
			{
				Round_Right++;
			}				
			else 
			{
				Round_Right --;
				if(counterForRound_right==0) Round_Right = 0;
				counterForRound_right =1;
			}
    }                  
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除中断标志位        
}

void TIM4_IRQHandler(void)
{
		static u8 direction = 0; 
		u8 direction_register;
		direction = (TIM4->CR1&0X10)>>4;//反
		direction_register = direction;//反
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//溢出中断
    { 
			if(direction != last_direction_left) 
			{
				Round_Left = 0;
				direction = last_direction_left;//正
				counterForRound_left=0;
				last_direction_left = direction_register;//反
			}
			if(direction == 0)
			{
				Round_Left++;
			}				
			else 
			{
				Round_Left--;
				if(counterForRound_left==0) Round_Left = 0;
				counterForRound_left =1;
			}
    }                  
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//清除中断标志位        
}

float Read_Speed_left_wheel(void)
{
	float distance_left = 1.0;
	float angle_left = 0;
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);//运行更新中断停止
	int LEFT_WHEEL_READ_ENCODER = Read_Encoder(LEFT_WHEEL);
	angle_left = PI * Round_Left;
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//运行更新中断
	counterForRound_left = 0;
	angle_left += (PI/ENCODER_WIRE_NUMBER)*LEFT_WHEEL_READ_ENCODER;
	distance_left = (float)((angle_left/PI) * WHEELDIAMETER)*2.0;
	//printf("\nLEFT ANGLE = %f\nLEFT WHEEL Walked Throuth %f CM\n",angle_left/PI,distance_left);
	Round_Left = 0;
	return distance_left;
}

float Read_Speed_right_wheel(void)
{
	float distance_right = 1.0;
	float angle_right = 0;
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);//运行更新中断停止
	int RIGHT_WHEEL_READ_ENCODER = Read_Encoder(RIGHT_WHEEL);
	angle_right = PI * Round_Right;
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//运行更新中断
	counterForRound_right = 0;
	angle_right += (PI/ENCODER_WIRE_NUMBER)*RIGHT_WHEEL_READ_ENCODER;
	distance_right = (float)((angle_right/PI) * WHEELDIAMETER)*2.0;
	//printf("\nRIGHT ANGLE = %f\nRIGHT WHEEL Walked Throuth %f CM\n",angle_right/PI,distance_right);
	Round_Right = 0;
	return distance_right;
}

float Read_Speed_Average(void)
{
	float right_distance = Read_Speed_right_wheel();
	float left_distance = Read_Speed_left_wheel();
	float average_distance = (left_distance+right_distance)/2;
	printf("\nAverage Distance = %f\n",average_distance);
	return average_distance;
}

/*Motor Driver Board Pin Initialize*/
void Motor_Brake(u8 numberOfMotor)
{
	switch(numberOfMotor)
	{
		case 1:
			GPIO_ResetBits(GPIOE,GPIO_Pin_0|GPIO_Pin_1);
			break;
		case 2:
			GPIO_ResetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_3);
			break;		
		default:
			GPIO_ResetBits(GPIOE,GPIO_Pin_0|GPIO_Pin_1);
			GPIO_ResetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_3);
	}
}
//0-255映射到0-400
//Speed大于0，正转；Speed小于0，反转
void Motor_Function(u8 wheel,int Speed)
{
	u16 arr = 400;
	u16 reload = arr - (arr/255) * Speed;
	if(Speed>0.2)
	{
			GPIO_ResetBits(GPIOE,GPIO_Pin_1);
			GPIO_SetBits(GPIOE,GPIO_Pin_0);
			GPIO_ResetBits(GPIOE,GPIO_Pin_3);
			GPIO_SetBits(GPIOE,GPIO_Pin_2);
			if(wheel == LEFT_WHEEL) TIM_SetCompare1(TIM8,reload);
			else if(wheel == RIGHT_WHEEL)	TIM_SetCompare2(TIM8,reload);
			else 
			{
				TIM_SetCompare1(TIM8,reload);
				TIM_SetCompare2(TIM8,reload);
			}
	}
	else if(Speed >= -0.2&&Speed <= 0.2)
	{
		Motor_Brake(ALL_WHEEL);
	}
	else
	{
			reload = arr - (arr/255) * (-Speed);
			GPIO_ResetBits(GPIOE,GPIO_Pin_0);
			GPIO_SetBits(GPIOE,GPIO_Pin_1);
			GPIO_ResetBits(GPIOE,GPIO_Pin_2);
			GPIO_SetBits(GPIOE,GPIO_Pin_3);
			if(wheel == LEFT_WHEEL) TIM_SetCompare1(TIM8,reload);
			else if(wheel == RIGHT_WHEEL)	TIM_SetCompare2(TIM8,reload);
			else 
			{
				TIM_SetCompare1(TIM8,reload);
				TIM_SetCompare2(TIM8,reload);
			}
		}
}

/*Initialize System*/
void Motor_IN_Pin_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	Encoder_Init_TIM2();
	Encoder_Init_TIM4();
	
	TIM8_PWM_Init(400,3599);//50Hz
}

/*
舵机调整角度
正数表示往右转，负数表示往左转
*/
void Servo_Set_Angle(int16_t angle)
{
	float increment=0;
		//TIM_SetCompare3(TIM8,400-25);//10-50//增加1代表增加6.75°
	//限制舵机转角
	if(angle>65) angle=65;
	else if(angle<-65) angle = -65;

	increment = angle*0.148;
	if(increment>15) increment = 15;
	else if(increment < -15) increment = -15;
	TIM_SetCompare3(TIM8,402-(25+(int)increment));
}

