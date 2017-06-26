#include "includes.h"


s16 Deform(s16 x,u8 a,s16 max){
  u8 sig,n;
  s32 y2,y;
  
  if(x<0){sig=1;x=-x;} 
  else sig=0;
  
  if(x>max)x=max;
  
  n = 16*x/max;
  y2 = (((s32)16*x-max*n)*(n+1)*(n+1)
        +((s32)n*max+max-16*x)*n*n)/256;
  y = (y2*a+(100-a)*x)/100;
  if(sig) y=-y;
  return (s16)y;
}