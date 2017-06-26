#ifndef __BALANCE__
#define __BALANCE__

#include "typedef.h"

extern u16 accZ_raw ;
extern u16 gyroP_raw ;
extern s16 balance_order ;

//--
extern s64 gyroP_offset_new;
//--
extern s32 calibrate_spd;
extern U8 calibrate_en;

void Balance_Init(void);
void Balance_Filter();
void Balance_Order(void);

extern s16 angle_int;
extern u16 gyro_offset_int;

#endif
