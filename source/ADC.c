#include "includes.h"

void ADC_Init(void){
  SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;//ADC1 Clock Enable
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;//ADC0 Clock Enable
  
  ADC1->CFG1 |= 0
             //|ADC_CFG1_ADLPC_MASK
             | ADC_CFG1_ADICLK(1)
             | ADC_CFG1_MODE(1);
             //| ADC_CFG1_ADIV(0);
  ADC0->CFG1 |= 0
             //|ADC_CFG1_ADLPC_MASK
             | ADC_CFG1_ADICLK(1)
             | ADC_CFG1_MODE(1);
             //| ADC_CFG1_ADIV(0);
  
  
  //ADC1->CFG2 &=~ADC_CFG2_MUXSEL_MASK;
  ADC1->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_ADLSTS(2)|ADC_CFG2_MUXSEL_MASK; 
  ADC0->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_ADLSTS(2); 
  
  ADC1->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
  ADC0->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
  //ADC1->SC1[0] |= ADC_SC1_ADCH(11) | ADC_SC1_ADCH(12) | ADC_SC1_ADCH(13) | ADC_SC1_ADCH(14) | ADC_SC1_ADCH(15);
  
  ADC1->SC2&=0x00000000;//Clear all.
  ADC1->SC3 = 0 | ADC_SC3_AVGE_MASK | 0x10;
  
  PORTC->PCR[0]|=PORT_PCR_MUX(0);//adc0-14
  PORTC->PCR[1]|=PORT_PCR_MUX(0);//adc0-15
  PORTC->PCR[2]|=PORT_PCR_MUX(0);//adc0-4
  PORTC->PCR[9]|=PORT_PCR_MUX(0);//adc1-5
  PORTC->PCR[10]|=PORT_PCR_MUX(0);//adc1-6
}

U16 ADC_accX(void){
	ADC0->SC1[0] = ADC_SC1_ADCH(4);
	while((ADC0->SC1[0]&ADC_SC1_COCO_MASK)==0);
	return ADC0->R[0];
}

U16 ADC_accY(void){
	ADC0->SC1[0] = ADC_SC1_ADCH(15);
	while((ADC0->SC1[0]&ADC_SC1_COCO_MASK)==0);
	return ADC0->R[0];
}

U16 ADC_accZ(void){
	ADC0->SC1[0] = ADC_SC1_ADCH(14);
	while((ADC0->SC1[0]&ADC_SC1_COCO_MASK)==0);
	return ADC0->R[0];
}

U16 ADC_gyroY(void){
	ADC1->SC1[0] = ADC_SC1_ADCH(5);
	while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
	return ADC1->R[0];
}

U16 ADC_gyroP(void){
	ADC1->SC1[0] = ADC_SC1_ADCH(6);
	while((ADC1->SC1[0]&ADC_SC1_COCO_MASK)==0);
	return ADC1->R[0];
}
