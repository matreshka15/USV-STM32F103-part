#ifndef __DATAFUSION
#define __DATAFUSION
#include "stm32f10x.h"

extern float psi,theta,fhi;
void MadgwickUpdateAttitude(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, 
												int16_t m_x, int16_t m_y, int16_t m_z,float beta);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float beta);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float beta);

void Convert_Quaternion_To_Euler(void);
void PrintAttitudeData(void);
extern float SEq_1, SEq_2, SEq_3, SEq_4;
extern float yaw,pitch,roll;
#endif
