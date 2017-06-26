#include "includes.h"

u8 sqr[16]={
  0,3,8,15,
  24,35,48,63,
  80,99,120,143,
  168,195,224,255
};


u16 deform_poweru(u8 a,u16 x,u16 max){     //  (a/100)x^2 + ((100-a)/100)x
  u32 t,y,ysqr;
  u8 t1,b;
  
  b=a/100;
  a%=100;
  if(x>max)x=max;
  
  t=x;
  t*=25500;
  t = (t+max/2)/max;
  
  t1 = t/100/16;
  
  ysqr = (u32)(sqr[t1+1]-sqr[t1])*(t-1600*t1)/16 + sqr[t1]*100;
  
  y = ((u32)a*ysqr + (u32)(100-a)*t1*100 )/100;
  
  return ((y*max/255+50)/100);
}

s16 deform_powers(u8 a,s16 x,u16 max){     //  (a/100)x^2 + ((100-a)/100)x
  u32 t,y,ysqr;
  u8 t1,b;
  s8 s=1;
  s16 xout;
  
  b=a/100;
  a%=100;
  if(x<0){x=-x;s=-1;}
  if(x>max)x=max;
  
  t=x;
  t*=25500;
  t = (t+max/2)/max;
  
  t1 = t/100/16;
  
  ysqr = (u32)(sqr[t1+1]-sqr[t1])*(t-1600*t1)/16 + sqr[t1]*100;
  
  y = ((u32)a*ysqr + (u32)(100-a)*t )/100;
  
  xout = (s16)((y*max/255+50)/100)*s;
  while(b--){
    xout*=x;
  };
  return xout;
}

