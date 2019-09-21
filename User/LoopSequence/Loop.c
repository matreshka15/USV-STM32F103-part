#define LOOP 1
#include "..\CONFIGURATION.h"

/*优先度最高的系统时间片划分函数
说明：
为了系统协调性起见，每个时间片中任务调度所占用时间最大值（考虑到中断等突发事件）有以下限制：
200Hz: T5<=5ms
100Hz: T10<=T10-T5=5ms
50Hz:  T20<=T20-T10=15ms
10Hz:  T100<=T100-T20=80ms
1Hz:	 T1000<=T1000-T100=900ms
*/
void Loop(void)
{
	/*--------1Hz级任务---------*/
	if(Mission.Hz1)
	{
		feed_dog();//喂狗
		switch(current_state)
		{
			/*Limited State Machine:程序正常运行状态*/
			case STATE_NORMALLY_RUNNING:
			{
				if(RaspiOfflineIndicator>0) RaspiOfflineIndicator--;
			}
			case STATE_MANUAL_CTRL:	
			case STATE_COLLECTING_POINTS:
			{
				if(MadgwickGainAutoDecend >= MadgwickGainDecendTime) MadgwickGainAutoDecend = MadgwickGainDecendTime;
				else MadgwickGainAutoDecend ++;//用于使Madgwick增益自动缩小。
				LED_BRINK();	
				break;
			}				
			/*Limited State Machine:调试状态*/
			case STATE_DEBUGGING:
			{
				PrintAttitudeData();
				break;
			}
		}
		Mission.Hz1 = 0;
	}
	
	/*--------5Hz级任务---------*/
	if(Mission.Hz5)
	{
		
		switch(current_state)
		{
			/*Limited State Machine:程序正常运行状态*/
			case STATE_NORMALLY_RUNNING:
			{
				#if !USE_USART1_COMM_DMA_RX
					if(resolve_frame_from_Host_computer()) 
					{
						MISSION_LED_BRINK();
						RaspiOfflineIndicator = MotorLockTimeAfterRaspiOffline;
					}
				#endif
			}
			case STATE_MANUAL_CTRL:
			case STATE_COLLECTING_POINTS:
			{
				synthesis_frame();//向上位机发送当前姿态数据
				break;
			}
			/*Limited State Machine:调试状态*/
			case STATE_DEBUGGING:
			{
				TIM3_4channel_process();
				printf("\nTim3-CH1:%d\n",CHANNEL_HIGHOUTPUT[0]);
				//SendYawFromCompass();
				break;
			}
		}
		Mission.Hz5 = 0;
	}
	
	/*--------10Hz级任务---------*/
	if(Mission.Hz10)
	{
		TIM3_4channel_process();
		/*===有限状态机驱动器以10Hz的频率运行===*/
		/*===*/CHANGE_STATE_TO(Analyze_Controller_Msg());/*===*/
		/*======================================*/
		switch(current_state)
		{
			/*Limited State Machine:程序正常运行状态*/
			case STATE_NORMALLY_RUNNING:
			{
				if(RaspiOfflineIndicator && SignalFromGPS() && Motor_Enabled) 
				{
					Execute_Planned_Path(yaw,routeToGo.yaw,routeToGo.distance,routeToGo.control_status,10);
				}
				else 
				{
					Servo_Set_Angle(0);
					Motor_Brake(ALL_WHEEL);
				}		
			}
			case STATE_COLLECTING_POINTS:
			case STATE_MANUAL_CTRL:
			{
				#if !USE_USART3_GPS_DMA_RX
					pvtFromGPS();
				#endif
				receiveComData();
				dataFrom6050();
				filter_IMU_data(10);
				break;
			}
			/*Limited State Machine:调试状态*/
			case STATE_DEBUGGING:
			{
				print_IMU_data();
				/*左右轮PID调速 单位是每0.1秒轮子走过的距离(厘米)*/
				//Motor_Function(ALL_WHEEL,Minimize_Greatest_Error_Increment_PID(Read_Speed_left_wheel(),Read_Speed_right_wheel(),-4));
				//Motor_Function(LEFT_WHEEL,Minimize_Greatest_Error_Increment_PID(Read_Speed_left_wheel(),Read_Speed_left_wheel(),-3));
				//Motor_Function(RIGHT_WHEEL,Minimize_Greatest_Error_Increment_PID(Read_Speed_right_wheel(),Read_Speed_right_wheel(),3));			
				break;
			}
		}
		Mission.Hz10 = 0;
	}
	
	/*--------50Hz级任务---------*/
	if(Mission.Hz50)
	{
		switch(current_state)
		{
			case STATE_MANUAL_CTRL:
			/*Limited State Machine:程序正常运行状态*/
			case STATE_COLLECTING_POINTS:
			case STATE_NORMALLY_RUNNING:
			{
				MadgwickAHRSupdate(pAngleSpeed.channelX,pAngleSpeed.channelY,pAngleSpeed.channelZ, \
													pAcceleration.channelX,pAcceleration.channelY,pAcceleration.channelZ,\
													pCompass_Main.channelX,pCompass_Main.channelY,pCompass_Main.channelZ,\
													(MadgwickGainDecendTime-MadgwickGainAutoDecend)+BETA);
				Convert_Quaternion_To_Euler();		
				break;				 	
			}
			
			/*Limited State Machine:调试状态*/
			case STATE_DEBUGGING:
			{
				break;
			}
		}
		Mission.Hz50 = 0;
	}
	
	/*--------100Hz级任务---------*/
	if(Mission.Hz100)
	{
		Mission.Hz100 = 0;
	}
	
	/*--------200Hz级任务---------*/
	if(Mission.Hz200)
	{
		Mission.Hz200 = 0;
	}
}

/*任务驱动器初始化函数*/
void TIM5_STSTEM_PULSE_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 100;									//5ms定时器+1
	TIM_TimeBaseStructure.TIM_Prescaler = (7200)/APB1DIV-1;	//0.05ms中断一次
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	//中断优先级 NVIC 设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; //TIM5 中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //先占优先级 1 级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级 1 级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
	NVIC_Init(&NVIC_InitStructure); //初始化 NVIC 寄存器
	TIM_Cmd(TIM5, ENABLE); //使能 TIM5
	TIM_ARRPreloadConfig(TIM5,ENABLE);//自动重装载使能
	TIM_Cmd(TIM5,ENABLE);
	system_pulse = 1;
	Mission.Hz1 = 0;Mission.Hz10 = 0;Mission.Hz50 = 0;Mission.Hz100 = 0;Mission.Hz200 = 0;
}


/*
任务调度systick驱动器
*/
void TIM5_IRQHandler(void) //TIM5 中断
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //检查 TIM5 更新中断发生与否
	{
		/*一个高电平脉冲表示相应计数已溢出*/
		system_pulse++;
		if((system_pulse+1)%(1)==0)
		{
			if((system_pulse+1)%(2)==0)
			{
				if((system_pulse+1)%(4)==0)
				{
					if((system_pulse+1)%(20)==0)
					{
						if((system_pulse+1)%(40)==0)
						{
								if((system_pulse+1)%(200)==0)
								{
									Mission.Hz1 = 1;
									system_pulse = 0;
								}
								Mission.Hz5 = 1;
						}
						Mission.Hz10 = 1;
					}
					Mission.Hz50 = 1;
				}
				Mission.Hz100 = 1;
			}
			Mission.Hz200 = 1;
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除 TIM5 更新中断标志
	}
}

u8 Analyze_Controller_Msg(void)
{
	/*通道0为模式选择
	通道1为电机使能按钮
	通道2为左右转弯
	通道3为油门刹车*/
	u16 bottom_limit = 1200;
	u16 upper_limit = 1800;
	u8 Status = STATE_MANUAL_CTRL;
	int16_t angle  = -(int16_t)((((float)((int16_t)CHANNEL_HIGHOUTPUT[2]-1500))/(float)500)*(float)70);
	float speed = ((float)((int16_t)CHANNEL_HIGHOUTPUT[3]-1500)/(float)700);
	
		if(CHANNEL_HIGHOUTPUT[0]<=bottom_limit)
		{
			Status = STATE_COLLECTING_POINTS;
		}
		else if(CHANNEL_HIGHOUTPUT[0]<=upper_limit)
		{
			Status = STATE_NORMALLY_RUNNING;
		}
		else
		{
			Status = STATE_MANUAL_CTRL;
		}
		if(CHANNEL_HIGHOUTPUT[1]<=bottom_limit)
		{//通知上位机进行GPS坐标点转换
			Motor_Enabled = FALSE;
		}
		else if(CHANNEL_HIGHOUTPUT[1]<=upper_limit)
		{
			Motor_Enabled = FALSE;
		}
		else
		{
			Motor_Enabled = TRUE;
		}

	#if USE_BRUSHED_MOTOR
		if(Motor_Enabled)
		{
			if(Status==STATE_MANUAL_CTRL||Status==STATE_COLLECTING_POINTS)
			{
				MISSION_LED_BRINK();
				if(CHANNEL_HIGHOUTPUT[3]>=1700||CHANNEL_HIGHOUTPUT[3]<=1300)
					Motor_Function(ALL_WHEEL,Minimize_Greatest_Error_Increment_PID(Read_Speed_left_wheel(),Read_Speed_right_wheel(),(0.85*MOTOR_MAX_SPEED*speed)));
				else
					Motor_Brake(ALL_WHEEL);
				
				if(angle>=2||angle<=-2)
					Servo_Set_Angle(angle);
				else
					Servo_Set_Angle(0);				
			}
		}
		else
		{
			Servo_Set_Angle(0);
			Motor_Brake(ALL_WHEEL);			
		}
	#endif
		return Status;
}
