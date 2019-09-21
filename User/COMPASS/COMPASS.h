#ifndef __COMPASS
#define __COMPASS
#include "stm32f10x.h"

u8 verifyCompass(void);
u8 checkDataReady(void);
void cfgCompassReg(void);
u8 receiveComData(void);
void CompassInit(void);
typedef struct{	
	volatile int16_t channelX;
	volatile int16_t channelY;
	volatile int16_t channelZ;
}Compass;
extern Compass eCompass;
extern int16_t Local_mag_bias;
#endif
