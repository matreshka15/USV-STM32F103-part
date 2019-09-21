#ifndef __FILTER
#define __FILTER
#include "stm32f10x.h"

float ch1_Float_LowPass_Filter(float data,float Increment,float dt,float tau);
void filter_IMU_data(u16 freq);

#endif
