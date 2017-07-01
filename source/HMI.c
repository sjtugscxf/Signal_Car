/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"

// ===== Global Variables =====

U16 battery;

// --- Local Variables ---

u16 bell_request_tick = 0;



// ===== BELL SERVICE ====

void Bell_Request(u8 tick){
  bell_request_tick = tick;
}

void Bell_Service(){
  if(bell_request_tick>0){
    bell_request_tick--;
    BELL(1);
  }
  else
    BELL(0);
}




// ===== Basic APIs =====

//----- LED ------

void LED2(u8 x){
  if(x)
    PTA->PSOR |= 1<<7;
  else
    PTA->PCOR |= 1<<7;
}
void LED2_Tog(){
  PTA->PTOR |= 1<<7;
}
void LED1(u8 x){
  if(x)
    PTA->PSOR |= 1<<6;
  else
    PTA->PCOR |= 1<<6;
}
void LED1_Tog(){
  PTA->PTOR |= 1<<6;
}

//----- Bell ------

void BELL(u8 x){
  if(x)
    PTD->PSOR |= 1<<15;
  else
    PTD->PCOR |= 1<<15;
}


// --- battery  ---
u16 Battery(){
  u16 tmp;
  ADC1->SC1[0] = ADC_SC1_ADCH(0);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  tmp = ADC1->R[0];
  //
  return tmp;
}


// ===== INIT =====

void HMI_Init(void){
  
  //===== LED =====
  PORTA->PCR[6] |= PORT_PCR_MUX(1);
  PORTA->PCR[7] |= PORT_PCR_MUX(1);
  PTA->PDDR |= (3<<6);
  PTA->PDOR |= (3<<6);

  //===== KEY =====
  PORTA->PCR[8] |= PORT_PCR_MUX(1);
  PORTA->PCR[9] |= PORT_PCR_MUX(1);
  PORTA->PCR[10] |= PORT_PCR_MUX(1);
  PTA->PDDR &=~(7<<8);
  PORTA->PCR[8] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;	//PULLUP
  PORTA->PCR[9] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  PORTA->PCR[10] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  
  //===== SW =====
  PORTC->PCR[4] |= PORT_PCR_MUX(1);
  PORTC->PCR[5] |= PORT_PCR_MUX(1);
  PORTC->PCR[6] |= PORT_PCR_MUX(1);
  PORTC->PCR[7] |= PORT_PCR_MUX(1);
  PTC->PDDR &=~(0xf<<4);
  PORTC->PCR[4] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  PORTC->PCR[5] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  PORTC->PCR[6] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  PORTC->PCR[7] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;  
  
  //===== Bell =====
  PORTD->PCR[15] |= PORT_PCR_MUX(1);
  PTD->PDDR |= (1<<15);
  PTD->PDOR &=~ (1<<15);

  //==== battery ====
  
  if(!ADC1_enabled){
    SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;  //ADC1 Clock Enable
    ADC1->CFG1 |= 0
               //|ADC_CFG1_ADLPC_MASK
               | ADC_CFG1_ADICLK(1)
               | ADC_CFG1_MODE(1);
               //| ADC_CFG1_ADIV(0);
    ADC1->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                  ADC_CFG2_ADACKEN_MASK; 
    
    ADC1->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
    ADC1_enabled = 1;
  }
}



