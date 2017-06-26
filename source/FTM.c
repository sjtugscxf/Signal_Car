#include "includes.h"

S16 tachol,tachor;

void PWM_R(s16 x){
  if(x>1000) x=1000;
  if(x<-1000) x=-1000;
  if(x<0){
    FTM0->CONTROLS[4].CnV = (-x+PWM_DZR)*4/3;
    FTM0->CONTROLS[5].CnV = 0;
  }
  else if(x>0){
    FTM0->CONTROLS[4].CnV = 0;
    FTM0->CONTROLS[5].CnV = (x+PWM_DZR);//*2/3;
  }
  else{
    FTM0->CONTROLS[4].CnV = 0;
    FTM0->CONTROLS[5].CnV = 0;
  }
}

void PWM_L(s16 x){
  if(x>1000) x=1000;
  if(x<-1000) x=-1000;
  if(x>0){
    FTM0->CONTROLS[6].CnV = (x+PWM_DZL)*4/3; //*1/1
    FTM0->CONTROLS[7].CnV = 0;
  }
  else if(x<0){
    FTM0->CONTROLS[6].CnV = 0;
    FTM0->CONTROLS[7].CnV = (-x+PWM_DZL);
  }
  else{
    FTM0->CONTROLS[6].CnV = 0;
    FTM0->CONTROLS[7].CnV = 0;
  }
}

S16 Tacho_R(void){
  S16 tmp;
  tmp = -(S16)FTM1->CNT;
  FTM1->CNT=0;
  return tmp;
}

S16 Tacho_L(void){
  S16 tmp;
  tmp = -(S16)FTM2->CNT;
  FTM2->CNT=0;
  return tmp;
}


void FTM0_Init(void){
  SIM->SCGC6|=SIM_SCGC6_FTM0_MASK;
  FTM0->SC|=FTM_SC_CLKS(1)|FTM_SC_PS(0);//PS16,System Clock
  FTM0->MOD=1000;//Max Value
  FTM0->CONTROLS[4].CnSC|=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;
  FTM0->CONTROLS[5].CnSC|=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;
  FTM0->CONTROLS[6].CnSC|=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;
  FTM0->CONTROLS[7].CnSC|=FTM_CnSC_MSB_MASK|FTM_CnSC_ELSB_MASK;
  FTM0->CONTROLS[4].CnV=0;
  FTM0->CONTROLS[5].CnV=0;
  FTM0->CONTROLS[6].CnV=0;
  FTM0->CONTROLS[7].CnV=0;
  FTM0->POL = 0xff;
  PORTD->PCR[4]|=PORT_PCR_MUX(4);
  PORTD->PCR[5]|=PORT_PCR_MUX(4);
  PORTD->PCR[6]|=PORT_PCR_MUX(4);
  PORTD->PCR[7]|=PORT_PCR_MUX(4);
}

void FTM1_Init(void){
	PORTB->PCR[0]|=PORT_PCR_MUX(6);
	PORTB->PCR[1]|=PORT_PCR_MUX(6);
	SIM->SCGC6|=SIM_SCGC6_FTM1_MASK;
	FTM1->MODE|=FTM_MODE_WPDIS_MASK;//Write protection disable
	FTM1->QDCTRL|=FTM_QDCTRL_QUADMODE_MASK;
	FTM1->CNTIN=0;
	FTM1->MOD=0XFFFF;
	FTM1->QDCTRL|=FTM_QDCTRL_QUADEN_MASK;
	FTM1->MODE|=FTM_MODE_FTMEN_MASK;//let all registers available for use
	FTM1->CNT=0;
}

void FTM2_Init(void){
	PORTB->PCR[18]|=PORT_PCR_MUX(6);//FTM2->PTA10,CHA
	PORTB->PCR[19]|=PORT_PCR_MUX(6);//FTM2->PTA11,CHB
	SIM->SCGC3|=SIM_SCGC3_FTM2_MASK;
	FTM2->MODE|=FTM_MODE_WPDIS_MASK;//Write protection disable
	FTM2->QDCTRL|=FTM_QDCTRL_QUADMODE_MASK;
	FTM2->CNTIN=0;
	FTM2->MOD=0XFFFF;
	FTM2->QDCTRL|=FTM_QDCTRL_QUADEN_MASK;
	FTM2->MODE|=FTM_MODE_FTMEN_MASK;//let all registers available for use
	FTM2->CNT=0;
}

void FTM_Init(void){
  FTM0_Init();
  FTM1_Init();
  FTM2_Init();
}