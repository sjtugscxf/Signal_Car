#include "includes.h"

s16 speed_real_l,speed_real_r;
s16 speed_pwm;
s16 speed_order;
s16 speed_set = 0;

void Speed_Renew(){
  s16 set,p,i,d;
  static s16 u[10];
  u8 j;
  /*if((speed_real_l+speed_real_r)<speed_set){
     set = speed_set/2;
     p = (s16)P_speed;
     d = 0;
  }
  else{
    set = speed_set;
    p = (s16)P_speed/3;
    d = (s16)D_speed;
  }*/
  set = speed_set;
  p = (s16)P_speed;
  i = (s16)I_speed;
  d = (s16)D_speed;
  u[0] = set * 2 - speed_real_l - speed_real_r;
  speed_pwm = p * u[0]/10 + d * (u[0] - u[1])/10;
  for(j = 0;j<8;j++){
    speed_pwm + j * u[i] / 10;
  }
  
  for(j = 9;j > 0;j--){
    u[j] = u[j-1];
  }
  
  
  speed_real_l=0;
  speed_real_r=0;
  
}

void Speed_Order(){
  static s16 speed_pwm_last=0;
  speed_order = speed_pwm_last + (time_cnt%10+1)*(speed_pwm-speed_pwm_last)/10;
  if(time_cnt%10==9) speed_pwm_last=speed_pwm;
}
  
void WheelL_Order(s16 acc){
}