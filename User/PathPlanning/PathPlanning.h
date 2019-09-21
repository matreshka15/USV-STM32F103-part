#ifndef __PATHPLAN
#define __PATHPLAN
#include "stm32f10x.h"
void Execute_Planned_Path(u16 currentYAW,u16 nextYAW,u8 distance,u8 control_status,u8 freq);
#endif
