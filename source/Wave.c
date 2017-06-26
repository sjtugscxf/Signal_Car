#include "includes.h"

// ===== Variables ======
//---- GLOBAL ----

U32 wavetimef;
U32 wavetime;
U32 wavetimeus;
U32 distance;
U32 distance_tmp;
U32 distance_last;

U32 trigtimef;
U32 trigtime;
U32 trigtimeus;
//---- LOCAL ---

// ===== Function Declaration ===== ( Local ) ( No need for users to use)

  // -- Basic Drivers --

  // --  Hardware Interface --

// =======  Function Realization ======


void GetDistance()
{
  /*  PTC->PSOR |= 1<<16;   //¸ßµçÎ»
    trigtimef=PIT2_VAL();
    for (int i=0;i<1700;++i)
    {
      
    }
    trigtime=trigtimef-PIT2_VAL();
    trigtimeus = trigtime / (g_bus_clock/1000000); //1us
    PTC->PCOR |= 1<<16;   //µÍµçÎ»*/
}

void Wave_Init()
{
  /*  
  PORTC->PCR[17] |= PORT_PCR_MUX(1) |PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(11);    // 1 SI     ³¬Éù²¨Ð¾Æ¬ÊäÈë  0
    PORTC->PCR[16] |= PORT_PCR_MUX(1);    // 1 CLK   ³¬Éù²¨Ð¾Æ¬Êä³ö  1
    PTC->PDDR |= (0x1<<16);
    NVIC_EnableIRQ(PORTC_IRQn);
    NVIC_SetPriority(PORTC_IRQn, NVIC_EncodePriority(NVIC_GROUP, 2, 1));
  */
}

// ======= Basic Drivers ======


   
// ===== Hardware Interface =====
