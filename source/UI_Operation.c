/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/04
License : MIT
*/


#include "includes.h"

char ch;
u8 uiphrase=0; //0--选哪一个量进行调试 1--通过key1 2 加减量 
                 //key 3切换phrase
u8 uivar=0; //0--redthre
                 //1--CCDthre
				 //2--straightspeed
				 //3--leftaround
				 //4--rightaround


// ====== Settings ======

  // Sensibility of operation
  // smaller, more sensitive
#define SENSIBILITY 8

  // Strength to debounce
  // depends on period of UI Refreshing Loop
#define DEBOUNCE_CNT 6 




// ====== Variables ======

// ---- Global ----
u16 ui_operation_shift, ui_operation_cnt;

// ---- Local ----
u8 Key3_down,Key1_down,Key2_down;   // flag : state of pushed down
u8 Key3_rise,Key1_rise,Key2_rise;   // flag : transient state of rising
u8 Key3_cnt,Key1_cnt,Key2_cnt;      // counter to debounce
u8 Key1_risemask,Key2_risemask,Key3_risemask;   // flag to mask Rise_Func after DownSpin_Func



// ====== Local Func Declaration ====

  // --- API ---
void Key1_Rise_Func();
void Key2_Rise_Func();
void Key3_Rise_Func();
void Key1_Downspin_Func();
void Key2_Downspin_Func();
void Key3_Downspin_Func();
void Spin_Func();

  // --- Baisc Drivers ---
void Key1_Read();
void Key2_Read();
void Key3_Read();




// ====== APIs =======
// write your codes in certain Func you wanna realize.


  // --- Keyn_Rise_Func ---
  // triggered when Keyn rises after push with no spin of tacho.

void Key1_Rise_Func(){
  if (SW3()==0 && SW1()==0) {
	  if (uiphrase==1) {
		  switch (uivar) {
			case 0 : redthre++; break;
			case 1 : CCDthre++; break;
			case 2 : straightspeed++; break;
			case 3 : leftaround++; break;
			case 4 : rightaround++; break;
                        case 5 :   if(theroute>6) theroute=0;
                                   else theroute++; break;
                        case 6 :   if(route[0]>6) route[0]=0;
                                   else route[0]++; break;
                        case 7 :   if(route[1]>6) route[1]=0;
                                   else route[1]++; break;
                        case 8 :   if(route[2]>6) route[2]=0;
                                   else route[2]++; break;
                        case 9 :   if(route[3]>6) route[3]=0;
                                   else route[3]++; break;
                        case 10 :   if(route[4]>6) route[4]=0;
                                   else route[4]++; break;
                        case 11 :   if(route[5]>6) route[5]=0;
                                   else route[5]++; break;
                        case 12 :   if(route[6]>6) route[6]=0;
                                   else route[6]++; break;
                        case 13 :   if(route[7]>6) route[7]=0;
                                   else route[7]++; break;
                        case 14 :   if(route[8]>6) route[8]=0;
                                   else route[8]++; break;
                        case 15 :   if(route[9]>6) route[9]=0;
                                   else route[9]++; break;
			default : break;
		  }
	  }
	  else {
		  if (uivar==15) uivar=0;
		  else uivar++;
	  }
  }
  //Bell_Request(3);
}


void Key2_Rise_Func(){
  if (SW3()==0 && SW1()==0) {
	  if (uiphrase==1) {
		  switch (uivar) {
			case 0 : redthre--; break;
			case 1 : CCDthre--; break;
			case 2 : straightspeed--; break;
			case 3 : leftaround--; break;
			case 4 : rightaround--; break;
                        case 5 :   if (theroute<1) theroute=6;
                                   else theroute--; break;
                        case 6 :   if (route[0]<1) route[0]=6;
                                   else route[0]--; break;
                        case 7 :   if (route[1]<1) route[1]=6;
                                   else route[1]--; break;
                        case 8 :   if (route[2]<1) route[2]=6;
                                   else route[2]--; break;
                        case 9 :   if (route[3]<1) route[3]=6;
                                   else route[3]--; break;
                        case 10 :   if (route[4]<1) route[4]=6;
                                   else route[4]--; break;
                        case 11 :   if (route[5]<1) route[5]=6;
                                   else route[5]--; break;
                        case 12 :   if (route[6]<1) route[6]=6;
                                   else route[6]--; break;
                        case 13 :   if (route[7]<1) route[7]=6;
                                   else route[7]--; break;
                        case 14 :   if (route[8]<1) route[8]=6;
                                   else route[8]--; break;
                        case 15 :   if (route[9]<1) route[9]=6;
                                   else route[9]--; break;
			default : break;
		  }
	  }
	  else {
		  if (uivar==0) uivar=15;
		  else uivar--;
	  }
  }
  /*__disable_irq();
  while(1)
    ;*/
}

void Key3_Rise_Func(){ 
  if (SW3()==0 && SW1()==0) uiphrase=1-uiphrase;
  /*item_renew_cnt = 8;  
  data_renew_cnt = 8;  
  if(menu==1)
    menu=10;
  else if(menu>9 && menu<20 && selected==0)  
    menu=1;  
  else if(menu>9 && menu<20 && selected==1)  
    selected=0;    
    
  curse_display();  */
}


  // --- Keyn_Downspin_Func ---
  // triggered when Keyn is pushed down and spinning tacho.
  // if triggered , Rise_Func won't be trigger when rises.

void Key1_Downspin_Func(){
  if (SW3()==0 && SW1()==0) straightspeed++;
  //Spin_Func();
}

void Key2_Downspin_Func(){
  if (SW3()==0 && SW1()==0) straightspeed--;
  //Spin_Func();
}

void Key3_Downspin_Func(){
  
  Spin_Func();
}


  // --- Spin_Func ---
  // triggered when spinning tacho with no Key pushed.
void Spin_Func(){
  Bell_Request(1);
  
}


// ========= Service ======= 

  // Put this in UI refreshing loop (PIT1_ISR)

void UI_Operation_Service(){
  Key3_Read();
  Key1_Read();
  Key2_Read();
  
  ui_operation_shift = ui_operation_cnt/SENSIBILITY;
  ui_operation_cnt %= SENSIBILITY;
  
  if(Key1_rise){
    Key1_rise=0;
    Key1_Rise_Func();
  }
  if(Key2_rise){
    Key2_rise=0;
    Key2_Rise_Func();
  }
  if(Key3_rise){
    Key3_rise=0;
    Key3_Rise_Func();
  }
  if(Key1_down && ui_operation_shift!=0 ){       // 
    Key1_risemask=1;
    Key1_Downspin_Func();
  }
  else if(Key2_down && ui_operation_shift!=0 ){       // 
    Key2_risemask=1;
    Key2_Downspin_Func();
  }
  else if(Key3_down && ui_operation_shift!=0 ){       // 
    Key3_risemask=1;
    Key3_Downspin_Func();
  }
  else if(ui_operation_shift!=0){
    Spin_Func();
  }
}




// ===== Basic Drivers ====

void Key1_Read(){
  if(Key1()==0){
    Key1_down=1;
    Key1_cnt=0;
  }
  else if(Key1_down && Key1_cnt<DEBOUNCE_CNT)
    Key1_cnt++;
  else if(Key1_down){
    Key1_down=0;
    Key1_rise=1;
  }
  else
    Key1_down=0;
}

void Key2_Read(){
  if(Key2()==0){
    Key2_down=1;
    Key2_cnt=0;
  }
  else if(Key2_down && Key2_cnt<DEBOUNCE_CNT)
    Key2_cnt++;
  else if(Key2_down){
    Key2_down=0;
    Key2_rise=1;
  }
  else
    Key2_down=0;
}

void Key3_Read(){
  if(Key3()==0){
    Key3_down=1;
    Key3_cnt=0;
  }
  else if(Key3_down && Key3_cnt<DEBOUNCE_CNT)
    Key3_cnt++;
  else if(Key3_down){
    Key3_down=0;
    if(Key3_risemask==0)
      Key3_rise=1;
    else
      Key3_risemask=0;
  }
  else
    Key3_down=0;
}