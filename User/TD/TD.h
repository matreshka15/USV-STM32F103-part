#ifndef __TD
#define __TD
#include "stm32f10x.h"
typedef struct{	
	u16 yaw;
	u16 distance;
	u8 control_status;
	}Route;

extern u16 FRAMELENGTH;
extern Route routeToGo;
extern	u8 DATAFRAME[22];
void synthesis_frame(void);
u8 resolve_frame_from_Host_computer(void);
#endif
