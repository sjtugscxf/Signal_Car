#include "typedef.h"
#ifndef _CCD_H
#define _CCD_H
void DAC_Init();
void CCD_Init();
void CCD_SetThreshold(u16 data);
void CMOS_Control();
#define CMOS_POINT_PER_LINE 246
#define CMOS_POINT_ERROR 2   // max total point error permit in a line
#define TRACK_WIDTH 140      // the routine width
#define LINES_PER_FIELD 280  // the number of lines a field has
#define CMOS_INVALID_LEFT_POINT_COUNT 22  // when the cmp edge occured before this, it is invalid
#define CMOS_INVALID_RIGHT_POINT_COUNT 218
#define CMOS_INVALID_LINE_COUNT 20  // drop the first 20 lines because of the line interrupt signals

extern uint8 CMOS_Data[LINES_PER_FIELD][3];  // record blackL Midline blackR
extern uint16 CMOS_Data_Index;  // 2-D array line index

extern uint8 CMOS_Temp[16];   // record the blackL, midline and blackR a line
extern uint8 CMOS_Temp_Index;

extern uint8 CMOS_Last_MidLine;

extern uint8 CMOS_Sample_Flag;

extern uint8 CMOS_Invalid_Line_Count;
// only used to debug
extern int16 oled_debug;
extern int16 oled_debug2;

extern uint8 oecnt;
#endif