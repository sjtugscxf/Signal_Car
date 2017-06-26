#ifndef WAVE_H
#define WAVE_H

// ===== Global Variables =====
extern U32 distance;   //  mm


// ======= APIs =======

void GetDistance();      //在PIT0中调用

  // Init
void Wave_Init();       //在main最前面初始化


#endif