/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"


// ========= Variables =========

//--- global ---
U32 time_us = 0;
U32 pit0_time;
U32 pit1_time; 


 
//--- local ---
U32 pit1_time_tmp;

// =========== PID CONTROL =========== 
PIDInfo L, R;  //两个结构体指针，存与电机pid控制有关的量， 包括pid三个参数，lastErr
double L_err = 0;
double R_err = 0;
double L_pwm = 0;
double R_pwm = 0;



int16 speed_set = 0;

void PID_Init() 
{
  L.kp = 5;
  L.ki = 2;
  L.kd = 0;
  R.kp = 5;
  R.ki = 2;
  R.kd = 0;
  
  L.lastErr=0;
  L.errSum=0;
  R.lastErr=0;
  R.errSum=0;

  //临时测试用：
  debug_dir.kp=0;
  debug_dir.kd=0;
  
}

void PWM(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R)      //前进的PID控制
{  
  L_err=left_speed+tacho0;
  R_err=right_speed-tacho1;
  L->errSum+=L_err;
  if(L->errSum>300) L->errSum=300;
  if(L->errSum<-300) L->errSum=-300;
  R->errSum+=R_err;
  if(R->errSum>300) R->errSum=300;
  if(R->errSum<-300)R->errSum=-300;
  L_pwm=(L_err*L->kp + L->errSum*L->ki + (L_err-L->lastErr)*L->kd);
  R_pwm=(R_err*R->kp + R->errSum*R->ki + (R_err-R->lastErr)*R->kd);
  L->lastErr=L_err;
  R->lastErr=R_err;
  
  if(L_pwm>700)  L_pwm=700;
  if(R_pwm>700)  R_pwm=700;
  if(L_pwm<-700)  L_pwm=-700;
  if(R_pwm<-700)  R_pwm=-700;
  MotorL_Output((int)(-L_pwm)); 
  MotorR_Output((int)(-R_pwm));
}

void PWMne(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R)   //后退的PID控制，都是输入正数，输入负数有奇怪的bug
{
  L_err=left_speed-tacho0;
  R_err=right_speed+tacho1;
  L->errSum+=L_err;
  if(L->errSum>300) L->errSum=300;
  if(L->errSum<-300) L->errSum=-300;
  R->errSum+=R_err;
  if(R->errSum>300) R->errSum=300;
  if(R->errSum<-300)R->errSum=-300;
  L_pwm=(L_err*L->kp + L->errSum*L->ki + (L_err-L->lastErr)*L->kd);
  R_pwm=(R_err*R->kp + R->errSum*R->ki + (R_err-R->lastErr)*R->kd);
  L->lastErr=L_err;
  R->lastErr=R_err;
  
  if(L_pwm>700)  L_pwm=700;
  if(R_pwm>700)  R_pwm=700;
  if(L_pwm<-700)  L_pwm=-700;
  if(R_pwm<-700)  R_pwm=-700;
  MotorL_Output((int)(L_pwm)); 
  MotorR_Output((int)(R_pwm));
}



// =========== PIT 1 ISR =========== 
// ====  UI Refreshing Loop  ==== ( Low priority ) 

void PIT1_IRQHandler(){
  
  PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
  
  pit1_time_tmp = PIT2_VAL();
  
  //------------------------
  
  LED1_Tog();
  
  UI_Operation_Service();
  
  Bell_Service();
  
  UI_SystemInfo();
  
  
  //------------ Other -------------
  
  pit1_time_tmp = pit1_time_tmp - PIT2_VAL();
  pit1_time_tmp = pit1_time_tmp / (g_bus_clock/10000); //100us
  pit1_time = pit1_time_tmp;
  
}



//============ PIT 0 ISR  ==========
// ====  Control  ==== ( High priority )

void PIT0_IRQHandler(){
  PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
  
  time_us += PIT0_PERIOD_US;

  
  
  //-------- System info -----
  
  pit0_time = PIT2_VAL();
    
  battery = Battery();
  
  
  
  
  //-------- Get Sensers -----
  
  
  // Tacho
  Tacho0_Get();
  Tacho1_Get();
  
  // UI operation input
  ui_operation_cnt += tacho0;  // use tacho0 or tacho1
    

  
#if (CAR_TYPE==0)   // Magnet and Balance
  
  Mag_Sample();
  
  gyro1 = Gyro1();
  gyro2 = Gyro2();
  
  
  
#elif (CAR_TYPE==1)     // CCD
  
  CCD1_GetLine(ccd1_line);
  CCD2_GetLine(ccd2_line);
  
  
  
  
#else               // Camera
  
  // Results of camera are automatically put in cam_buffer[].
  
  
#endif
  
  
  
  // -------- Sensor Algorithm --------- ( Users need to realize this )
  
  // mag example : dir_error = Mag_Algorithm(mag_val);
  // ccd example : dir_error = CCD_Algorithm(ccd1_line,ccd2_line);
  // cam is complex. realize it in Cam_Algorithm() in Cam.c
  
  //-------- Controller --------



  
  // not balance example : dir_output = Dir_PIDController(dir_error);
  // example : get 'motorL_output' and  'motorR_output'
 
  //motorControl();
 
  // ------- Output -----
  
  
  // not balance example : Servo_Output(dir_output);  
  // example : MotorL_Output(motorL_output); MotorR_Output(motorR_output);
 //MotorL_Output(550); MotorR_Output(-550);

  
  
  // ------- UART ---------
  
  
  //UART_SendDataHead();
  //UART_SendData(battery);
  
  
  
  // ------- other --------
  
  pit0_time = pit0_time - PIT2_VAL();
  pit0_time = pit0_time / (g_bus_clock/1000000); //us
  
}




// ======= INIT ========

void PIT0_Init(u32 period_us)
{ 
                   
  SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
  
  PIT->MCR = 0x00;
 
  NVIC_EnableIRQ(PIT0_IRQn); 
  NVIC_SetPriority(PIT0_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));

  //period = (period_ns/bus_period_ns)-1
  PIT->CHANNEL[0].LDVAL |= period_us/100*(g_bus_clock/1000)/10-1; 
  
  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK |PIT_TCTRL_TEN_MASK;

};

void PIT1_Init(u32 period_us)
{ 
                   
  SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
  
  PIT->MCR = 0x00;
 
  NVIC_EnableIRQ(PIT1_IRQn); 
  NVIC_SetPriority(PIT1_IRQn, NVIC_EncodePriority(NVIC_GROUP, 3, 0));

  //period = (period_ns/bus_period_ns)-1
  PIT->CHANNEL[1].LDVAL |= period_us/100*(g_bus_clock/1000)/10-1; 
  
  PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TIE_MASK |PIT_TCTRL_TEN_MASK;

}

void PIT2_Init()
{ 
                   
  SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
  
  PIT->MCR = 0x00;

  //period = (period_ns/bus_period_ns)-1
  PIT->CHANNEL[2].LDVAL = 0xffffffff; 
  
  PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TEN_MASK;

}