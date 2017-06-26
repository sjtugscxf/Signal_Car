#include "includes.h"

void WheelR_Speed(s16 v){
  s16 e0;
  static s16 e1,e2;
  static s32 u1;
  s32 p,di,dd,u0,du,pwm;
  
  e0 = v-tachor;
  
  di = (s32)wheel_I * e0 *2;
  dd = (s32)wheel_D * (e0 + e2 - 2*e1) *2;
  du = di+dd;
  du = du * ABS(du)/100;
  u0 = u1+du;
  
  p = (s32)wheel_P * e0 * ABS(e0);
  
  pwm = (u0+p)/50;  
  if(pwm>700)pwm=700;
  if(pwm<-700)pwm=-700;
  PWM_R((s16)pwm);
  
  u1 = u0;
  e2 = e1; e1 = e0;
}
void WheelL_Speed(s16 v){
  s16 e0;
  static s16 e1,e2;
  static s32 u1;
  s32 p,di,dd,u0,du,pwm;
  
  e0 = v-tachol;
  
  di = (s32)wheel_I * e0 *2;
  dd = (s32)wheel_D * (e0 + e2 - 2*e1) *2;
  du = di+dd;
  du = du * ABS(du)/100;
  u0 = u1+du;
  
  p = (s32)wheel_P * e0 * ABS(e0);
  
  pwm = (u0+p)/50;  
  if(pwm>700)pwm=700;
  if(pwm<-700)pwm=-700;
  PWM_L((s16)pwm);
  
  u1 = u0;
  e2 = e1; e1 = e0;
}