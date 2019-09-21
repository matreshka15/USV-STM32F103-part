#define PID 1
#include "..\CONFIGURATION.h"

/*
简单的增量PID
*/
int Increment_PID(float measurement,float Target)
{
	static float bias=11,bias_last1=0,outputPWM=130;
	bias = (measurement - Target);
	//printf("Bias = %f",bias);
	outputPWM += Velocity_KP*(bias-bias_last1)+Velocity_KI*bias;
	
	if(outputPWM>=255) outputPWM = 254;
	else if(outputPWM<=-255) outputPWM = -254;
	//outputPWM += Velocity_KP*bias_last1+Velocity_KI*bias;+Velocity_KD*bias_last2;
	//bias_last2 = bias_last1;
	bias_last1 = bias;
	//printf("\nOUTPUTPWM = %f\n",outputPWM);
	return (int)outputPWM;
}
/*
两个相同相同控制量，最小化误差最大的那个量。
类似于夹挤定理
0为正转，1为反转
*/
int Minimize_Greatest_Error_Increment_PID(float encoder1,float encoder2,float Target)
{
	static float bias = 5,bias_last1=0,outputPWM=0;
	static u8 last_direction = 0;
	float abs_bias1,abs_bias2;
	float bias1=11,bias2 = 11;
	u8 direction=0;
	last_direction = direction;
	direction = Target>0?1:0;
	bias1 = (encoder1 - Target);
	bias2 = (encoder2 - Target);
	if(bias1<0) abs_bias1 = -bias1;
	else abs_bias1 = bias1;
	if(bias2<0) abs_bias2 = -bias2;
	else abs_bias2 = bias2;
	
	
	if(last_direction != direction) 
	{
		outputPWM = 0;
		bias_last1 = 0;
		if(!direction)
		{
			bias = -10;			
		}
		else
		{
			bias = 10;	
		}
	}
	else if(abs_bias1>=abs_bias2) bias = bias1;
	else bias = bias2;
	//printf("\nPID Maximum Bias = %f\n",bias);
	outputPWM += Velocity_KP*(bias-bias_last1)+Velocity_KI*bias;
	
	if(outputPWM>=255) outputPWM = 254;
	else if(outputPWM<=-255) outputPWM = -254;
	bias_last1 = bias;
	return (int)outputPWM;
}
