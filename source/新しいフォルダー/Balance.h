#ifndef __BALANCE__
#define __BALANCE__

#include "typedef.h"

extern s16 accZ,gyroP;

extern u32 accZ_raw ;
extern u32 gyroP_raw ;
extern u32 gyroP_offset;
extern s16 balance_order ;
extern s32 angle;
extern s16 palstance;

//--
extern s64 gyroP_offset_new;
//--
extern s32 calibrate_spd;
extern U8 calibrate_en;

void Balance_Init(void);
void Balance_Renew(void);
void Balance_Sample(s16 offset_error);
void Balance_Filter(s16 offset_error);
void Balance_KF(s16 offset_error);
void Balance_Order(void);

void Calibrate_Init();
void Balance_Calibrate();
void Balance_Calibrate50();

extern s16 accZ_raw_oled;
extern s16 gyroP_raw_oled;
extern s16 gyroP_offset_oled;
extern s16 angle_oled;


#endif
