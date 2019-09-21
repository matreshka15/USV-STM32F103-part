#ifndef TIMESLICE
#define TIMESLICE

#include "stm32f10x.h"
extern volatile u16 system_pulse;
extern int16_t RaspiOfflineIndicator;
extern u8 MotorLockTimeAfterRaspiOffline;
extern	u8 current_state;//STATE_NORMALLY_RUNNING;
extern	u8 next_state;//STATE_NORMALLY_RUNNING;
void Loop(void);
u8 Analyze_Controller_Msg(void);
void TIM5_STSTEM_PULSE_Init(void);
#endif
