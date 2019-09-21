#ifndef __PID
#define __PID
#include "stm32f10x.h"
int Increment_PID(float encoder,float Target);
int Minimize_Greatest_Error_Increment_PID(float encoder1,float encoder2,float Target);
#endif
