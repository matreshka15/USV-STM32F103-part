#define DATAFUSION 1
#include "..\CONFIGURATION.h"


float invSqrt(float x);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float beta);
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float beta) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az,beta);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-SEq_2 * gx - SEq_3 * gy - SEq_4 * gz);
	qDot2 = 0.5f * (SEq_1 * gx + SEq_3 * gz - SEq_4 * gy);
	qDot3 = 0.5f * (SEq_1 * gy - SEq_2 * gz + SEq_4 * gx);
	qDot4 = 0.5f * (SEq_1 * gz + SEq_2 * gy - SEq_3 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * SEq_1 * mx;
		_2q0my = 2.0f * SEq_1 * my;
		_2q0mz = 2.0f * SEq_1 * mz;
		_2q1mx = 2.0f * SEq_2 * mx;
		_2q0 = 2.0f * SEq_1;
		_2q1 = 2.0f * SEq_2;
		_2q2 = 2.0f * SEq_3;
		_2q3 = 2.0f * SEq_4;
		_2q0q2 = 2.0f * SEq_1 * SEq_3;
		_2q2q3 = 2.0f * SEq_3 * SEq_4;
		q0q0 = SEq_1 * SEq_1;
		q0q1 = SEq_1 * SEq_2;
		q0q2 = SEq_1 * SEq_3;
		q0q3 = SEq_1 * SEq_4;
		q1q1 = SEq_2 * SEq_2;
		q1q2 = SEq_2 * SEq_3;
		q1q3 = SEq_2 * SEq_4;
		q2q2 = SEq_3 * SEq_3;
		q2q3 = SEq_3 * SEq_4;
		q3q3 = SEq_4 * SEq_4;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * SEq_4 + _2q0mz * SEq_3 + mx * q1q1 + _2q1 * my * SEq_3 + _2q1 * mz * SEq_4 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * SEq_4 + my * q0q0 - _2q0mz * SEq_2 + _2q1mx * SEq_3 - my * q1q1 + my * q2q2 + _2q2 * mz * SEq_4 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * SEq_3 + _2q0my * SEq_2 + mz * q0q0 + _2q1mx * SEq_4 - mz * q1q1 + _2q2 * my * SEq_4 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * SEq_3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * SEq_4 + _2bz * SEq_2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * SEq_3 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * SEq_2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * SEq_4 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * SEq_3 + _2bz * SEq_1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * SEq_4 - _4bz * SEq_2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * SEq_3 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * SEq_3 - _2bz * SEq_1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * SEq_2 + _2bz * SEq_4) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * SEq_1 - _4bz * SEq_3) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * SEq_4 + _2bz * SEq_2) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * SEq_1 + _2bz * SEq_3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * SEq_2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	SEq_1 += qDot1 * (1.0f / sampleFreq);
	SEq_2 += qDot2 * (1.0f / sampleFreq);
	SEq_3 += qDot3 * (1.0f / sampleFreq);
	SEq_4 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 *= recipNorm;
	SEq_2 *= recipNorm;
	SEq_3 *= recipNorm;
	SEq_4 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float beta) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-SEq_2 * gx - SEq_3 * gy - SEq_4 * gz);
	qDot2 = 0.5f * (SEq_1 * gx + SEq_3 * gz - SEq_4 * gy);
	qDot3 = 0.5f * (SEq_1 * gy - SEq_2 * gz + SEq_4 * gx);
	qDot4 = 0.5f * (SEq_1 * gz + SEq_2 * gy - SEq_3 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * SEq_1;
		_2q1 = 2.0f * SEq_2;
		_2q2 = 2.0f * SEq_3;
		_2q3 = 2.0f * SEq_4;
		_4q0 = 4.0f * SEq_1;
		_4q1 = 4.0f * SEq_2;
		_4q2 = 4.0f * SEq_3;
		_8q1 = 8.0f * SEq_2;
		_8q2 = 8.0f * SEq_3;
		q0q0 = SEq_1 * SEq_1;
		q1q1 = SEq_2 * SEq_2;
		q2q2 = SEq_3 * SEq_3;
		q3q3 = SEq_4 * SEq_4;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * SEq_2 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * SEq_3 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * SEq_4 - _2q1 * ax + 4.0f * q2q2 * SEq_4 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	SEq_1 += qDot1 * (1.0f / sampleFreq);
	SEq_2 += qDot2 * (1.0f / sampleFreq);
	SEq_3 += qDot3 * (1.0f / sampleFreq);
	SEq_4 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 *= recipNorm;
	SEq_2 *= recipNorm;
	SEq_3 *= recipNorm;
	SEq_4 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//=================================================================================================
// END OF CODE FOR MADGWICK'S ALGORITHM
//====================================================================================================

void Convert_Quaternion_To_Euler(void)
{	
	fhi = atan2( 2.0f * SEq_4 * SEq_1 + 2.0f * SEq_2 * SEq_3, 1-2*(SEq_1*SEq_1+SEq_2*SEq_2));
	theta = asin( -2.0f*(SEq_1 * SEq_3-SEq_4 * SEq_2));
	psi = atan2( 2.0f * SEq_4 * SEq_3 + 2.0f * SEq_1 * SEq_2,1-2*(SEq_2*SEq_2+SEq_3*SEq_3));
	yaw = (-fhi)*180/PI - Magnatic_Static_Bias;
	if(yaw < 0 ) yaw += 2*180;
	roll = theta*180/PI;
	pitch = psi*180/PI;
}


void PrintAttitudeData(void)
{
	printf("=====GPS数据=====\n\
					Long:%d Lat:%d sumSV:%d"
					,GPS_PVTData.lat,GPS_PVTData.lon,GPS_PVTData.numSV);
//					printf("\n=====IMU数据=====\
//					\n姿态Psi:|%.2f|\
//	        \n姿态theta:|%.2f|\
//					\n姿态fhi:|%.2f|\
//					\nQ1=%.2fQ2=%.2fQ3=%.2fQ4=%.2f\n"
//					,psi*180/PI,(float)theta*180/PI,\
//					yaw,SEq_1,SEq_2,SEq_3,SEq_4);	
	
}
