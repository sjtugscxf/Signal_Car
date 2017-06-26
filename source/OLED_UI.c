
#include "includes.h"


  /*
  UI_Page homepage;
  UI_Page subpage0;
  homepage.sub_type = (enum Item_Type *) malloc (4*2);
  homepage.sub_type[0] = Item_Type_Menu;
  homepage.sub = (void **)malloc (4*2); // (void **)(UI_Page **)
  *((UI_Page **)(homepage.sub)+0) = (UI_Page *) &subpage0;
  subpage0.parent = (void *) &homepage;
  
  subpage0.sub = (void **)123;
  Oled_Putnum(0,0,(s16)((*((UI_Page **)homepage.sub+0))->sub));
  */
  //free(homepage.sub);
  //free(homepage.sub_type);


enum Item_Type{
    Item_Type_Menu,
    Item_Type_Para,
    Item_Type_Show,
    Item_Type_Func,
};

typedef struct {
  void * parent;   // UI_Page *
  enum Item_Type * sub_type; 
  void ** sub;  // UI_Page **
}UI_Page;


void UI_SystemInfo(){
  Oled_Putstr(0,0,"Car Type"); Oled_Putnum(0,11,CAR_TYPE);
  Oled_Putstr(1,0,"battery"); Oled_Putnum(1,11,battery);
  Oled_Putstr(3,0,"pit0 time"); Oled_Putnum(3,11,(s16)pit0_time);
  Oled_Putstr(4,0,"pit1 time"); Oled_Putnum(4,11,(s16)pit1_time);
  Oled_Putstr(5,0,"tacho0"); Oled_Putnum(5,11,tacho0);
  Oled_Putstr(6,0,"tacho1"); Oled_Putnum(6,11,tacho1);
  //Oled_Putstr(7,0,"accx"); Oled_Putnum(7,11,accx);
}

void UI_Graph(u8* data){
  
}
















