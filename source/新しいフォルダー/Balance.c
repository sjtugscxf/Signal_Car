#include "includes.h"

u32 gyroP_offset = 0;      //10 times of real value

s32 angle = 0;  //1000 times
s16 palstance = 0;

u32 accZ_raw = 0;					//1000 times
u32 accZ_raw_last = 0;
u32 gyroP_raw = 0;

u64 gyroP_renew = 0;

s16 accZ;
s16 gyroP;

s16 balance_order;

u16 palstance_t=250;
s64 gyroP_offset_new;

//---
s16 calibrate_pos=0;
u16 calibrate_path=0;
s16 calibrate_speed[40];
u16 calibrate_cnt=0;
s32 calibrate_spd=0;
U8 calibrate_en;

const uint16 Qacc=50,Qgyro=2,Qoff=1;

void Balance_Init(void){
  gyroP_offset = 18000;//gyroP_raw /100;
  gyroP_renew = 1800000;//gyroP_raw ;
  
  angle = accZ_raw/1000 - accZ_offset; 
}

void Balance_Renew(){
  gyroP_offset_new = (s64)gyroP_raw +((s64)accZ_raw-(s64)accZ_raw_last)*10000/balance_dt;
  gyroP_renew = ( gyroP_renew * (800-1) + gyroP_offset_new ) / 800;
  gyroP_offset = gyroP_renew/100;
  gyroP_offset_oled = gyroP_offset/10;
}

void Balance_Sample(s16 offset_error){
  
  accZ = (u16)((s16)(accZ_raw/1000) - offset_error - (s16)accZ_offset);
  
  gyroP = gyroP_raw/1000 - gyroP_offset/10;
  
  accZ_raw_last = accZ_raw;
}

void Balance_Filter(s16 offset_error){
  Balance_Renew();
  Balance_Sample(offset_error);
  
  angle =( ( angle - gyroP * balance_dt/10)*(400-1) + accZ*1000 ) / 400;
  
  palstance = ((palstance_t-100)*palstance + 100 * gyroP )/palstance_t;
  
  angle_oled = angle/1000;
}
void Balance_KF(s16 offset_error){  
  s32 xp;  //angle predict *1000(accZ)
  static u32 off=1770000; //gyroP offset *1000
  u16 dt;  // *1000
  static s32 p11,p12,p21,p22;  //covariance of [angle,off] *10000
  s32 pp11,pp12,pp21,pp22;  //covariance of [xp,offp] *10000
  static s32 K1,K2;  //Kalman Gain of angle,off  *1000 000
  static s32 lastangle;
  
  dt = KF_dt;
  
  Balance_Sample(offset_error);
  lastangle = angle;
  xp = -angle + gyroP_raw*dt/1000 - off*dt/1000;
  
  pp11 = p11 + p22*dt*dt/1000000 
             + 1*dt*dt/100
             - p12*dt/1000 - p21*dt/1000;
  pp12 = p12 - p22*dt/1000;
  pp21 = p21 - p22*dt/1000;
  pp22 = p22 + 5000;
  
  K1 = (s32)((s64)pp11*1000000/(pp11+10000000));
  K2 = (s32)((s64)pp21*1000000/(pp11+10000000));
  angle = xp - K1*accZ/1000 - (s64)K1*xp/1000000;
  off = off - K2*accZ/1000 - (s64)K2*xp/1000000;  gyroP_offset=off/100;
  
  p11 = pp11 - K1*pp11/1000000;
  p12 = pp12 - K1*pp12/1000000;
  p21 = pp21 - K2*pp11/1000000;
  p22 = pp22 - K2*pp12/1000000;
  
  angle = -angle;
  palstance = (lastangle - angle)/1000;
}

void Balance_Order(void){
  u8 deform;
  if(calibrate_en)
    deform=0;
  else
    deform=balance_deform;
  
  balance_order = angle*balance_P /10000
                  - balance_D * palstance /100;
  balance_order = Deform(balance_order,deform,800);
  
  // protect
  if((angle>90000 && palstance<-80)||
     (angle<-90000 && palstance>80)) 
    Switch=0;
}

//-------- Calibrate -----------

void Balance_Calibrate50(){
  u8 i;
  calibrate_pos -= calibrate_speed[39];
  if(calibrate_speed[39]>0)  
    calibrate_path -= calibrate_speed[39];
  else
    calibrate_path -= (u16)(-calibrate_speed[39]);
  
  for(i=39;i>0;i--)
    calibrate_speed[i] = calibrate_speed[i-1];
  
  calibrate_speed[0] = tachol + tachor;
  
  calibrate_pos += calibrate_speed[0];
  if(calibrate_speed[0]>0)  
    calibrate_path += calibrate_speed[0];
  else
    calibrate_path += (u16)(-calibrate_speed[0]);
}

void Balance_Calibrate(){
  
  calibrate_spd += tachol+tachor;
  
  if(Switch==0){
    calibrate_en = 0;
  }
  else if(calibrate_path<350 &&
          calibrate_pos>-50 && calibrate_pos<50){
    
    if(calibrate_cnt==0 ){
      accZ_offset -= calibrate_spd*1 / balance_P /3;
      angle += calibrate_spd / balance_P * 40;
      calibrate_en = 0;
      Switch=0;
      Flash_Write(0);
    }
    else
      calibrate_cnt--;
  }
  else
    calibrate_cnt=1000;
}

void Calibrate_Init(){
  u8 i;
  
  for(i=0;i<40;i++) calibrate_speed[i]=0;
  calibrate_pos=0;
  calibrate_path=0;
  calibrate_cnt=1200;
  calibrate_spd=0;
  calibrate_en = 1;
  Switch = 1;
}
