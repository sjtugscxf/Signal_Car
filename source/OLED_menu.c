#include "includes.h"

u8 selected;

u16 menu=1;
u16 curse=0;
u16 curse1[MENU_1_ITEM_NUM],page1[MENU_1_ITEM_NUM];

u8 item_renew_cnt = 8;
u8 data_renew_cnt = 8;

U8 menu_1_item_num[MENU_1_ITEM_NUM]={  
  MENU_10_ITEM_NUM,        //10
  MENU_11_ITEM_NUM, 
  MENU_12_ITEM_NUM,      
  MENU_13_ITEM_NUM,          
  MENU_14_ITEM_NUM,
};

//------------------------
u8 Menu_0_item[8][8]={
	"       ",
	"       ",
	"       ",
	"       ",
	"       ",
	"       ",
	"       ",
	"       ",
};

u8 Menu_1_item[8][10]={
	"Observe  ",
	"Balance  ",
	"Speed    ",
	"Video    ",
	"Wheel    ",
	"         ",
	"         ",
	"         ",
};

u8 Menu_10_item[8*MENU_10_PAGE_NUM][10]={
	"accZ     ",
	"gyroP raw",
	"gyroP off",
	"angle    ",
	"Bal order",
	"tacho L  ",
	"tacho R  ",
        "spd order",
        "         ",
        "         ",
        "         ",
        "         ",
        "         ",
        "         ",
        "         ",
};

u8 Menu_11_item[8][10]={
    "Balance P",
	"Balance D",
	"dt       ",
	"accZ off ",
	"K        ",
	"         ",
	"         ",
	"         ",
};

u8 Menu_12_item[8][10]={
    "Speed    ",
	"Speed P  ",
    "Speed I  ",
	"Speed D  ",
    "DZ L     ",
	"DZ R     ",
    "         ",
	"         ",
};

u8 Menu_13_item[8][10]={
        "Threshold",
	"         ",
	"         ",
	"         ",
        "         ",
        "         ",
	"         ",
	"         ",
};

u8 Menu_14_item[8][10]={
        "P        ",
	"I        ",
	"D        ",
	"         ",
        "         ",
        "         ",
	"         ",
	"         ",
};

//------------------------------

U16* menu_10_pin[MENU_10_ITEM_NUM]={

};
U16* menu_11_pin[MENU_11_ITEM_NUM]={

};

U16* menu_12_pin[MENU_12_ITEM_NUM]={

};

U16* menu_13_pin[MENU_13_ITEM_NUM]={

};

U16* menu_14_pin[MENU_14_ITEM_NUM]={

};

/*
void (*function_pin[MENU_14_ITEM_NUM])(void)={
  Calibrate_Init,
  Flash_Data_Reset,
};*/

//------------------------------

void item_display(void){
  u8 cnt;
  if(item_renew_cnt>0){
    cnt = item_renew_cnt-1;
    
    if(menu==1)
      oled_putstr(cnt,1,Menu_1_item[cnt]);
    else if(menu==10)
      oled_putstr(cnt,1,Menu_10_item[cnt+8*page1[0]]);
    else if(menu==11)
      oled_putstr(cnt,1,Menu_11_item[cnt]);
    else if(menu==12)
      oled_putstr(cnt,1,Menu_12_item[cnt]);
    else if(menu==13)
      oled_putstr(cnt,1,Menu_13_item[cnt]);
    else if(menu==14)
      oled_putstr(cnt,1,Menu_14_item[cnt]);
    
    item_renew_cnt--;
  }
}

void data_display(u8 mode){
  u8 cnt,t;
  
  if(mode==0){ //mode 0  print all data	for once	
    if(data_renew_cnt>0){
      cnt = data_renew_cnt-1;
      
      if(menu==1)
        oled_putstr(cnt,11,Menu_0_item[cnt]);
      else if(menu==11){
        if(cnt<MENU_11_ITEM_NUM)
          oled_putnum(cnt,11,*menu_11_pin[cnt]);
        else
          oled_putstr(cnt,11,Menu_0_item[cnt]);
      }
      else if(menu==12){
        if(cnt<MENU_12_ITEM_NUM)
          oled_putnum(cnt,11,*menu_12_pin[cnt]);
        else
          oled_putstr(cnt,11,Menu_0_item[cnt]);
      }
      else if(menu==13){
        if(cnt<MENU_13_ITEM_NUM)
          oled_putnum(cnt,11,*menu_13_pin[cnt]);
        else
          oled_putstr(cnt,11,Menu_0_item[cnt]);
      }
      else if(menu==14){
        if(cnt<MENU_14_ITEM_NUM)
          oled_putnum(cnt,11,*menu_14_pin[cnt]);
        else
          oled_putstr(cnt,11,Menu_0_item[cnt]);
      }
      data_renew_cnt--;
    }
    
  }
  
  else if(mode == 1){				//mode 1  Update one data
    if(menu==11)
      oled_putnum(curse1[1],11,*menu_11_pin[curse1[1]]);
    else if(menu==12)
      oled_putnum(curse1[2],11,*menu_12_pin[curse1[2]]);
    else if(menu==13)
      oled_putnum(curse1[3],11,*menu_13_pin[curse1[3]]);
    else if(menu==14)
      oled_putnum(curse1[4],11,*menu_14_pin[curse1[4]]);
  }
  
  else if(mode == 2){				//mode 2 for continusly data updating
    if(menu==10){
      t=time_cnt-8*page1[0];
      if(t<(MENU_10_ITEM_NUM-8*page1[0])&&t<8)
        oled_putnum(t,11,*menu_10_pin[time_cnt]);
      else if(t<8)
        oled_putstr(t,11,Menu_0_item[t]);
    }
  }
  
}

void curse_display(void){
  u8 i;
  for(i=0;i<8;i++)
    oled_putstr(i,0," ");
  if(menu==1)
    oled_putstr(curse,0,">");
  else if(menu==10 && selected==0)
    oled_putstr(page1[0],0,">");
  else if(menu==10 && selected)
    oled_putstr(page1[0],0,"=");
  else if(menu==11 && selected==0)
    oled_putstr(curse1[1],0,">");
  else if(menu==11 && selected)
    oled_putstr(curse1[1],0,"=");
  else if(menu==12 && selected==0)
    oled_putstr(curse1[2],0,">");
  else if(menu==12 && selected)
    oled_putstr(curse1[2],0,"=");
  else if(menu==13 && selected==0)
    oled_putstr(curse1[3],0,">");
  else if(menu==13 && selected)
    oled_putstr(curse1[3],0,"=");
  else if(menu==14 && selected==0)
    oled_putstr(curse1[4],0,">");
  else if(menu==14 && selected)
    oled_putstr(curse1[4],0,"=");
}





