/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"


// ===== Global Variables =====
U16 mag_val[6];


// ===== Function Realization =====

void Mag_Sample(){
  mag_val[0] = Mag1();
  mag_val[1] = Mag2();
  mag_val[2] = Mag3();
  mag_val[3] = Mag4();
  mag_val[5] = Mag6();
  mag_val[4] = mag_val[5] - Mag5();
}

u16 Mag1(){
  ADC1->SC1[0] = ADC_SC1_ADCH(4);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC1->R[0];
}
u16 Mag2(){
  ADC1->SC1[0] = ADC_SC1_ADCH(5);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC1->R[0];
}
u16 Mag3(){
  ADC1->SC1[0] = ADC_SC1_ADCH(6);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC1->R[0];
}
u16 Mag4(){
  ADC1->SC1[0] = ADC_SC1_ADCH(7);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC1->R[0];
}
s16 Mag5(){
  ADC1->SC1[0] = ADC_SC1_DIFF_MASK | ADC_SC1_ADCH(3);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC1->R[0];
  //ADC1->SC1[0] &= ~ADC_SC1_DIFF_MASK;
}
u16 Mag6(){
  ADC1->SC1[0] = ADC_SC1_ADCH(3);
  while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
  return ADC1->R[0];
}

void Mag_Init(){
  
  if(!ADC1_enabled){
    SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;  //ADC1 Clock Enable
    ADC1->CFG1 |= 0
               //|ADC_CFG1_ADLPC_MASK
               | ADC_CFG1_ADICLK(1)
               | ADC_CFG1_MODE(1)
               | ADC_CFG1_ADIV(0);
    ADC1->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                  ADC_CFG2_ADACKEN_MASK;
    
    ADC1->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
    ADC1_enabled = 1;
  }
  
  PORTE->PCR[0]|=PORT_PCR_MUX(0);//adc1-4a
  PORTE->PCR[1]|=PORT_PCR_MUX(0);//adc1-5a
  PORTE->PCR[2]|=PORT_PCR_MUX(0);//adc1-6a
  PORTE->PCR[3]|=PORT_PCR_MUX(0);//adc1-7a
}