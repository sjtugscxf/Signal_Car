#ifndef SETTING_H
#define SETTING_H

  // NVIC GROUP NUM ( No understand no change =L= )
  // 7 : 0 PreemptPriorityBits & 4 SubPriorityBits
  // 3 : 4 PreemptPriorityBits & 0 SubPriorityBits
#define NVIC_GROUP 5  //  2 & 2


#define CAR_TYPE 2  // 0:Magnet & Balance    1:CCD   2:Camera

extern U8 ADC0_enabled;
extern U8 ADC1_enabled;

#endif