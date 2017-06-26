#ifndef OPERATION_H
#define OPERATION_H
#include "typedef.h"



// ==== Global Variables ====
extern u16 ui_operation_shift, ui_operation_cnt;

// ==== APIs ====
  // --- Service ----
  // Put this in UI refreshing loop (PIT1_ISR)
void UI_Operation_Service();



#endif