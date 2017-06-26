/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"



// ===== Variables ======
//---- GLOBAL ----

U8 ccd1_line[128];
U8 ccd2_line[128];

//---- LOCAL ---



// ===== Function Declaration ===== ( Local ) ( No need for users to use)

  // -- Basic Drivers --
u32 AD_Sample_CCD1();
u32 AD_Sample_CCD2();

  // --  Hardware Interface --
void CCD1_SI(u8 x);
void CCD2_SI(u8 x);
void CCD1_CLK(u8 x);
void CCD2_CLK(u8 x);



// =======  Function Realization ======

void CCD1_GetLine(U8 * ccd_line)
{
  u8 i;
 	
  //Collect pixels.
  //Sned SI
  CCD1_SI(1);  //SI = 1, t = 0
  
  asm("nop");asm("nop");
  
  CCD1_CLK(1); //CLK = 1, dt = 75ns
  CCD1_SI(0);  //SI = 0, dt = 50ns
  
  //First pixel.
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  
  ccd_line[0] = AD_Sample_CCD1();
  
  CCD1_CLK(0); //CLK = 0
 
  //2~128 CLK
  for(i=1; i<128; i++)
  {
    
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    
    CCD1_CLK(1);  //CLK = 1, dt = 125ns
    
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    
    ccd_line[i] = AD_Sample_CCD1();
    
    CCD1_CLK(0);  //CLK = 0.
  }
 
  //129 CLK
  
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  
  CCD1_CLK(1);  //CLK = 1.
  
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  
  CCD1_CLK(0);  //CLK = 0.
}


void CCD2_GetLine(U8 * ccd_line)
{
  u8 i;
 	
  //Collect pixels.
  //Sned SI
  CCD2_SI(1);  //SI = 1, t = 0
  
  asm("nop");asm("nop");
  
  CCD2_CLK(1); //CLK = 1, dt = 75ns
  CCD2_SI(0);  //SI = 0, dt = 50ns
  
  //First pixel.
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  
  ccd_line[0] = AD_Sample_CCD2();
  
  CCD2_CLK(0); //CLK = 0
 
  //2~128 CLK
  for(i=1; i<128; i++)
  {
    
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    
    CCD2_CLK(1);  //CLK = 1, dt = 125ns
    
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    
    ccd_line[i] = AD_Sample_CCD2();
    
    CCD2_CLK(0);  //CLK = 0.
  }
 
  //129 CLK
  
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  
  CCD2_CLK(1);  //CLK = 1.
  
  asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
  
  CCD2_CLK(0);  //CLK = 0.
}

  //  INIT 
void CCD_Init(){
  
  PORTB->PCR[20] |= PORT_PCR_MUX(1);    // 2 SI
  PORTB->PCR[21] |= PORT_PCR_MUX(1);    // 1 SI
  PORTB->PCR[22] |= PORT_PCR_MUX(1);    // 2 CLK
  PORTB->PCR[23] |= PORT_PCR_MUX(1);    // 1 CLK
  PTB->PDDR |= (0xf<<20);
  
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;//ADC0 Clock Enable
  /*
  ADC0->CFG1 |= 0
             //|ADC_CFG1_ADLPC_MASK
             | ADC_CFG1_ADICLK(1)
             | ADC_CFG1_MODE(0);
             //| ADC_CFG1_ADIV(0);
  ADC0->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_ADACKEN_MASK; 
  
  ADC0->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
  
  PORTC->PCR[0]|=PORT_PCR_MUX(0);//adc0-14
  PORTC->PCR[1]|=PORT_PCR_MUX(0);//adc0-15
  */
    if(!ADC1_enabled){
    SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;  //ADC1 Clock Enable
    ADC1->CFG1 |= 0
               //|ADC_CFG1_ADLPC_MASK
               | ADC_CFG1_ADICLK(1)
               | ADC_CFG1_MODE(0)
               | ADC_CFG1_ADIV(0);
    ADC1->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                  ADC_CFG2_ADACKEN_MASK;
    ADC1_enabled = 1;
  }
  
}


// ======= Basic Drivers ======

u32 AD_Sample_CCD1(){
  /*ADC0->SC1[0] = ADC_SC1_ADCH(15);
  while((ADC0->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC0->R[0];*/
  
  ADC1->SC1[0] = ADC_SC1_ADCH(15);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC1->R[0];
}
u32 AD_Sample_CCD2(){
  ADC0->SC1[0] = ADC_SC1_ADCH(14);
  while((ADC0->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC0->R[0];
}
   
// ===== Hardware Interface =====
void CCD1_SI(u8 x){
  if(x)
    PTB->PSOR |= 1<<21;
  else
    PTB->PCOR |= 1<<21;
}
void CCD2_SI(u8 x){
  if(x)
    PTB->PSOR |= 1<<20;
  else
    PTB->PCOR |= 1<<20;
}

void CCD1_CLK(u8 x){
  if(x)
    PTB->PSOR |= 1<<23;
  else
    PTB->PCOR |= 1<<23;
}
void CCD2_CLK(u8 x){
  if(x)
    PTB->PSOR |= 1<<22;
  else
    PTB->PCOR |= 1<<22;
}

