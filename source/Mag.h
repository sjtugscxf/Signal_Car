#ifndef MAG_H
#define MAG_H

// ======== Variables =======

  // Results of function : Mag_Sample()
extern U16 mag_val[6];


// ======= APIs =======

  // To excute a series of sampling of Magnet Signal.  
  // Results are put in   mag_val[6]  
  // Suggest using this fucntion in   PIT0_ISR_Handler()
void Mag_Sample();          


  // Init
void Mag_Init();



// ===== Basic Drivers ===== (No need for user to use)

u16 Mag1();
u16 Mag2();
u16 Mag3();
u16 Mag4();
s16 Mag5();
u16 Mag6();


#endif