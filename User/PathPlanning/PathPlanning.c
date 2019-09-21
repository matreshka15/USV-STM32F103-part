#define PATHPLAN 1
#include "..\CONFIGURATION.h"

//Path planning

//给出当前Yaw与目标yaw，调整舵机与电机速度
//单位：YAW:度数，范围为0-360    distance:单位为厘米
void Execute_Planned_Path(u16 currentYAW,u16 nextYAW,u8 distance,u8 control_status,u8 freq)
{
	//printf("\ncurrentYaw=%d,nextyaw=%d,distance=%dcm,\n",currentYAW,nextYAW,distance);
	u8 spinDirection;	//0代表顺时针转；1代表逆时针转
	int degreesToSpin;
	volatile u8 tolerant_bias = 4; //允许的偏航角误差,单位为度
	/*舵机朝向控制延时，舵机角度不宜改变过快。*/
	static int16_t delay_counter = 0;
	
	if(delay_counter>=freq/5)//约每100ms执行一次
	{
		delay_counter = 0;
		/*舵机控制序列*/
		//计算旋转方向与旋转度数
		if(currentYAW>=nextYAW)
		{
			degreesToSpin = currentYAW - nextYAW;
			if(degreesToSpin>=180) spinDirection = CLOCKWISE;
			else spinDirection = COUNTER_CLOCKWISE;
		}
		else
		{
			degreesToSpin =  nextYAW - currentYAW;
			if(degreesToSpin>=180) spinDirection = COUNTER_CLOCKWISE;
			else spinDirection = CLOCKWISE;		
		}
		/*根据角度调整舵机*/
		/*需要转的角度大于可容忍的角度值*/
		if(degreesToSpin>=tolerant_bias)
		{
			if(degreesToSpin>65)
			{
				switch(spinDirection)
				{
					case CLOCKWISE:Servo_Set_Angle(65);break;
					case COUNTER_CLOCKWISE:Servo_Set_Angle(-65);break;
					default:Servo_Set_Angle(0);break;
				}
			}
			else
			{
				switch(spinDirection)
				{
					case CLOCKWISE:Servo_Set_Angle(degreesToSpin>30?50:1.5*degreesToSpin);break;//degreesToSpin
					case COUNTER_CLOCKWISE:Servo_Set_Angle(degreesToSpin>30?-50:-1.5*degreesToSpin);break;
					default:Servo_Set_Angle(0);break;
				}			
			}
		}
		/*需要转的角度小于可容忍的角度值*/
		else
		{
			Servo_Set_Angle(0);
		}
	}
	
	if(control_status&0x80)
	{
		Motor_Brake(ALL_WHEEL);
	}
	/*根据距离调整车速*/
	else if(distance>200)
	{
		Motor_Function(ALL_WHEEL,Minimize_Greatest_Error_Increment_PID(Read_Speed_left_wheel(),Read_Speed_right_wheel(),(0.8*MOTOR_MAX_SPEED)));
	}
	else
	{
		Motor_Function(ALL_WHEEL,Minimize_Greatest_Error_Increment_PID(Read_Speed_left_wheel(),Read_Speed_right_wheel(),(0.085*(distance/20)*MOTOR_MAX_SPEED)));
	}
	delay_counter ++;
}
