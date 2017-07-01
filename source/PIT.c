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

extern char ch;

U32 trig_time=0;
bool success_flag=1;
bool dir_choice=0;
int  findl=0,findr=0;
s16 ccd1_linethre[128];
s16 linel=64,liner=64;
s16 linelS=64,linerS=64;
s16 middle=0;
s16 middle_rec=0;
bool nearflag=0;
int nearflag_cnt=0;
int i=0;

int cnttrig=0;

// =========== PIT 1 ISR =========== 
// ====  UI Refreshing Loop  ==== ( Low priority ) 

void PIT1_IRQHandler(){
  static int ct = 0;
  static char dir = 0;
  
  PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
  
  pit1_time_tmp = PIT2_VAL();
  
  //------------------------
 if (SW2())  UI_Operation_Service();
  
  //Bell_Service(); 

  //UI_SystemInfo();  //显示初始信息
  
  if (SW2())  AI_ShowInfo();    //一直显示AI信息，开关1坏了
  /*
  ++nearflag_cnt;
  if(nearflag_cnt>1000) nearflag_cnt=1000;
  */
  
  
 /////////////////CCD避障程序，可不用/////////////////////
 /*
  CCD1_GetLine(ccd1_line);
  findl=0;
  findr=0;

  for (int j=5;j<122;j++)  
    ccd1_linethre[j]=ccd1_line[j-3]+ccd1_line[j-2]+ccd1_line[j-1]+ccd1_line[j]
                     -ccd1_line[j+1]-ccd1_line[j+2]-ccd1_line[j+3]-ccd1_line[j+4];
  
  for (int i=5;i<122;i++) {
    if (ccd1_linethre[i]>CCDthre) {
      for(int j=i-10;j<i;++j) {    //从i-10到i-k改宽度
	    if(j<5) continue;
        if(ccd1_linethre[j]<-CCDthre) {
		  ++findr;
		  middle=(i+j)/2;
		}
      }
	}
  }
  */
  /*for (int i=64;i<122;i++) {
      if (ccd2_linethre[i]>60) {
          for(int j=i-8;j<i;++j) 
            if(ccd2_linethre[j]<-60)  {++findr;liner=(i+j)/2;}
      }
  }*/
  
 // middle=(linel+liner)/2;
  /*
  if(findr>0 && middle>36 && middle<90) {
	nearflag=1;
	nearflag_cnt=0;
	middle_rec=middle;
  }
  else {
    if(nearflag_cnt>10) nearflag=0;
  }
*/
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
  
  //CCD1_GetLine(ccd1_line);
  //CCD2_GetLine(ccd2_line);
  
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
 //MotorL_Output(530); 
 //MotorR_Output(530);
  
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