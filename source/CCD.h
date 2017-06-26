#ifndef CCD_H
#define CCD_H

// ===== Global Variables =====
  // Results of CCDs  after excuting CCDx_GetLine(ccdx_line); x=1,2
extern U8 ccd1_line[128];
extern U8 ccd2_line[128];


// ======= APIs =======

  // Get results of CCDs and put it into  ccd_line 
  // Suggest using this function in  PIT0_ISR_Handler()
void CCD1_GetLine(U8 * ccd_line);
void CCD2_GetLine(U8 * ccd_line);

  // Init
void CCD_Init();


#endif