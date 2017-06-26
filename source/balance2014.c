
#include "common.h"
#include "includes.h"

void main (void)
{
  //PIT1_Init();
  GPIO_Init();
  
  FTM_Init();
  ADC_Init();
  Flash_Init();
  
  //CCD_Init();
  UART_Init();
  
  PIT0_Init(5);
  Balance_Init();
  while(1)
  {
    
    //LED1(1-Switch);
    /*accZ_raw = (accZ_raw*99 + ADC_accZ()*1000)/100;
    gyroP_raw = (gyroP_raw*59 + ADC_gyroP()*1000)/60;*/
  
    Operation();
    item_display();
    data_display(0);
    data_display(2);
    
  } 
}

