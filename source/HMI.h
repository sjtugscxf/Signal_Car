#ifndef GPIO_H
#define GPIO_H
#include "typedef.h"


// ===== Global Variable =====

extern U16 battery;


// ======== BELL Service ======

  // Request a time of beep lasting 'tick' ticks.
  // lasting time = 'tick' * period of UI Refreshing Loop
void Bell_Request(u8 tick);

  // Put this in UI Refreshing Loop
void Bell_Service();




// ======== Basic APIs ========

//----- LED ------
  // 0 is On , 1 is Off
void LED1(u8 x);
void LED1_Tog();
void LED2(u8 x);
void LED2_Tog();

//----- Bell ------
  // 1 is On , 0 is Off
void BELL(u8 x);

//=== key ===

  // 0 is Pushed , 1 is Not
#define Key1() ((PTA->PDIR>>8)&1)
#define Key2() ((PTA->PDIR>>9)&1)
#define Key3() ((PTA->PDIR>>10)&1)

#define SW1() ((PTC->PDIR>>4)&1)
#define SW2() ((PTC->PDIR>>5)&1)
#define SW3() ((PTC->PDIR>>6)&1)
#define SW4() ((PTC->PDIR>>7)&1)

//-- battery  --
u16 Battery();

//--- Init ---
void HMI_Init(void);

// ======================

#endif