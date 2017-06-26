/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"



// === Receive ISR ===
void UART3_IRQHandler(void){
   uint8 tmp = UART_GetChar();
   if (tmp == 'a'){
     remote_state = 2;
     UART_SendString("turn left");//
   }else if (tmp == 'd'){
     remote_state = 3;
     UART_SendString("turn right");//
   }else if (tmp == 'w'){
     remote_state = 1;
     UART_SendString("go ahead");
   }else if (tmp == 's'){
     remote_state = 0;
     UART_SendString("stop");
   }
}

// ======== APIs ======

// ---- Ayano's Format ----

void UART_SendDataHead(){
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = 127;
}
void UART_SendData(s16 data){
  uint8 neg=0;
  uint8 num;
  if(data<0){
    neg = 1;
    data = -data;
  }
  num = data%100;
  
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = neg?(49-data/100):(50+data/100);
  
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = num;
}

// ----- Basic Functions -----

// Read 
int8 UART_GetChar(){
  while(!(UART3->S1 & UART_S1_RDRF_MASK));
  return UART3->D;
}

// Send
void UART_SendChar(uint8 data){
  while(!(UART3->S1 & UART_S1_TDRE_MASK));
  UART3->D = data;
}

void UART_SendString(const char *p) {
  while (*p)
    UART_SendChar(*p++);
}

// Init
void UART_Init(u32 baud){
  //UART3
  
  uint16 sbr;
  uint32 busclock = g_bus_clock;

  SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
  
  PORTE->PCR[4] = PORT_PCR_MUX(3);
  PORTE->PCR[5] = PORT_PCR_MUX(3);
  UART3->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );  // DISABLE UART FIRST
  UART3->C1 = 0;  // NONE PARITY CHECK
  
  //UART baud rate = UART module clock / (16 ¡Á (SBR[12:0] + BRFD))
  // BRFD = BRFA/32; fraction
  sbr = (uint16)(busclock/(16*baud));
  UART3->BDH = (UART3->BDH & 0XC0)|(uint8)((sbr & 0X1F00)>>8);
  UART3->BDL = (uint8)(sbr & 0XFF);
  
  UART3->C4 = (UART3->C4 & 0XE0)|(uint8)((32*busclock)/(16*baud)-sbr*32); 

  UART3->C2 |=  UART_C2_RIE_MASK;
  
  UART3->C2 |= UART_C2_RE_MASK;
  
  UART3->C2 |= UART_C2_TE_MASK; 
  
  NVIC_EnableIRQ(UART3_RX_TX_IRQn); 
  NVIC_SetPriority(UART3_RX_TX_IRQn, NVIC_EncodePriority(NVIC_GROUP, 2, 2));
}