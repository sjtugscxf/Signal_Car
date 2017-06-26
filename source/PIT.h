#ifndef _PIT_H_
#define _PIT_H_

#include "typedef.h"

// ===== Settings =====

  // PIT1 is used for UI (Refreshing Loop)
  // Its period determined the refresh rate.
  // Suggest 20000 ~ 40000
#define PIT1_PERIOD_US (u32)20000

  // PIT0 is used for Control
  // Its period determined the control rate.
  // Suggest 2500
#define PIT0_PERIOD_US (u32)2500

  // PIT2 is used for Knowing time
  // No interrupt. Use PIT2_VAL() to get time.


// ===== Global Variables ====

extern U32 time_us ;
extern U32 pit0_time;   // time PIT0 ISR takes
extern U32 pit1_time;   // time PIT1 ISR takes
extern bool success_flag;


// ===== APIs =====

void PIT0_Init(u32 period_us);
void PIT1_Init(u32 period_us);
void PIT2_Init();

// =========== PID CONTROL =========== 
typedef struct {
	double kp, ki, kd;
	double lastErr, llastErr;
	double pid;
        double errSum;
} PIDInfo;

extern PIDInfo L, R;
void PID_Init(); 
void PWM(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R);      //前进的PID控制
void PWMne(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R);   //后退的PID控制，都是输入正数，输入负数有奇怪的bug

extern int16 speed_set;



#define PIT2_VAL() (PIT->CHANNEL[2].CVAL)


#endif