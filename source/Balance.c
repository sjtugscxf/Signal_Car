#include "includes.h"

u16 accZ_raw = 0;
u16 gyroP_raw = 0;

s16 angle_int;
u16 gyro_offset_int;

s16 balance_order;

float acc,gyro,angle,gyro_offset,palstance,acc_last;

#define Kgo 0.9998   // K gyro offset
//#define K 0.005

void Balance_Init(void){
  gyro_offset = 1680;
  angle = (float)accZ_raw - accZ_offset;
  acc_last = (float)accZ_raw;
}

void Balance_Filter(){
  float dt = (float)balance_dt / 10000 ;
  float K = (float)balance_K / 10000;
  
  acc = (float)accZ_raw - accZ_offset;
  
  gyro_offset = gyro_offset * Kgo + ((float)gyroP_raw + (acc - acc_last)/dt)*(1 - Kgo);
  
  gyro = (float)gyroP_raw - gyro_offset;
  
  angle = (angle - gyro * dt) * (1-K) + acc * (K);
  
  palstance = palstance * 0.6 + gyro * 0.4;
  
  acc_last = acc;
  
  //
  angle_int = (s16)angle;
  gyro_offset_int = (u16)gyro_offset;
}

void Balance_Order(void){
  
  balance_order = angle*balance_P /10
                  - palstance * balance_D /100;
  //balance_order = Deform(balance_order,deform,800);
  
  // protect
  if((angle>150 && palstance<-120)||
     (angle<-150 && palstance>120)) 
    Switch=0;
}
