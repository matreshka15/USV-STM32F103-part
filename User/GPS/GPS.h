#ifndef __GPS
#define __GPS
#include "stm32f10x.h"
u8 UBX_CFG_USART(u32 baudrate);
u8 GPS_SENDDATA(u8 datain);
void USART3_IRQHandler(void);
void usart3_init(u32 baudrate);
void ringBuff_USART3_TEST(void);
u8 UBX_CFG_MODEL(void);
u8 UBX_CFG_DATAOUT(void);
u8 UBX_CFG_RATE(void);
u8 pvtFromGPS(void);
u8 SignalFromGPS(void);
void GPSDATA_Init(void);
void DisplayDataGPS(void);
typedef struct{	
	u8 fixtype;		//GPS锁定类型
	u8 fixFlags;	//GPS锁定类型标志位
	u8 numSV;			//用于导航解算的卫星数目
	int32_t lon;	//经度								 degree
	int32_t lat;	//纬度								 degree
	int32_t velN;	//北东地坐标系向北速度 mm/s
	int32_t velE;	//北东地坐标系向东速度 mm/s
	int32_t velD;	//北东地坐标系向地速度 mm/s
	u32 sAcc;			//预估速度精度				 mm/s
	u32 hAcc;			//预估水平精度				 mm
	u16 pdop;			//方位精度因子  pdop<4,精度很好;4<pdop<7，精度可接受;pdop>7,pdop较差
}pvtData;
extern pvtData GPS_PVTData;
#endif
