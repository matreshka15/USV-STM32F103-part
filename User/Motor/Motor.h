#ifndef __Motor
#define __Motor
#include "stm32f10x.h"
	#define FORWARD 1
	#define BACKWARD 2
	#define ENCODER_WIRE_NUMBER 1560
	#define LEFT_WHEEL 2
	#define RIGHT_WHEEL 1
	#define ALL_WHEEL 3
void Motor_Brake(u8 numberOfMotor);
void Motor_IN_Pin_Init(void);
void Encoder_Init_TIM2(void);
int Read_Encoder(u8 wheel);
void TIM2_IRQHandler(void);
float Read_Speed_right_wheel(void);
float Read_Speed_left_wheel(void);
float Read_Speed_Average(void);
void Encoder_Init_TIM4(void);
void TIM4_IRQHandler(void);
void Motor_Function(u8 wheel,int Speed);
void Servo_Set_Angle(int16_t angle);
#endif
