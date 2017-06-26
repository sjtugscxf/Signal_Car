/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/


#include "includes.h"


U8 ADC0_enabled = 0;
U8 ADC1_enabled = 0;//not used

void main (void)
{
  
  // --- System Initiate ---
  
  __disable_irq();
  
  HMI_Init();
  PIT0_Init(PIT0_PERIOD_US);
  PIT1_Init(PIT1_PERIOD_US);
  PIT2_Init();
  
  UART_Init(115200);
  
  Motor_Init();
  Tacho_Init();
  Servo_Init();
  PID_Init(); 
  Cam_B_Init();//≥ı ºªØCam_B
  
#if (CAR_TYPE==0)   // Magnet and Balance
  
  Mag_Init();
  LPLD_MMA8451_Init();
  Gyro_Init();
  
#elif (CAR_TYPE==1)     // CCD
  
  CCD_Init();
  
#else               // Camera
  
  Cam_Init();
  CCD_Init();
#endif
  
  //---  Press Key 1 to Continue ---
  Oled_Putstr(6,1,"Press Key1 to go on");
  while (Key1());
  Oled_Clear();

  __enable_irq();
  
  // --- System Initiated ---   
  while(1)
  {
    set_car_state();
    set_oled_menu();
    if(car_state!=0)
      Cam_B();
    Cam_Algorithm();
  }

}




// ===== System Interrupt Handler  ==== ( No Need to Edit )

void BusFault_Handler(){
  Oled_Clear();
  Oled_Putstr(1,5,"Bus Fault");
  Oled_Putstr(4,1,"press Key1 to goon");
  while(Key1());
  
  return;
}


void NMI_Handler(){
  Oled_Clear();
  Oled_Putstr(1,5,"NMI Fault");
  Oled_Putstr(4,1,"press Key1 to goon");
  while(Key1());
  
  return;
}

void HardFault_Handler(void)
{
  Oled_Clear();
  Oled_Putstr(1,5,"Hard Fault");
  Oled_Putstr(4,1,"press Key1 to goon");
  while(Key1());
  
  return;
}


void DefaultISR(void)
{
  Oled_Clear();
  Oled_Putstr(1,5,"Default ISR");
  Oled_Putstr(4,2,"press Key1 to goon");
  while(Key1());

  return;
}
