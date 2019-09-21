#define WATCHDOG 1
#include "..\CONFIGURATION.h"

//初始化看门狗
void WDG_Initialize(uint8_t prescaler,uint16_t reload)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//使能写权限
	//Tout=((4×2^prer) ×rlr) /40   ,溢出时间单位为ms
	IWDG_SetPrescaler(prescaler); //设置 IWDG 预分频值
	IWDG_SetReload(reload); //设置 IWDG 重装载值
}
//启动看门狗
//看门狗一旦被启动则无法关闭！
void WDG_Start(void)
{
	IWDG_Enable(); //使能 IWDG
}

//喂狗
void feed_dog(void)
{
	IWDG_ReloadCounter(); //按照 IWDG 重装载寄存器的值重装载 IWDG 计数器
}
