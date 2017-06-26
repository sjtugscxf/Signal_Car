#ifndef __OLED_MENU__
#define __OLED_MENU__

#include "typedef.h"


#define MENU_1_ITEM_NUM		5		//amount of items of Menu 1
#define MENU_10_ITEM_NUM	8		//amount of items of Menu 10
#define MENU_11_ITEM_NUM	5		//amount of items of Menu 11
#define MENU_12_ITEM_NUM	6		//amount of items of Menu 12
#define MENU_13_ITEM_NUM	1		//amount of items of Menu 13
#define MENU_14_ITEM_NUM	3		//amount of items of Menu 14
#define MENU_15_ITEM_NUM	0		//amount of items of Menu 15
#define MENU_16_ITEM_NUM	0		//amount of items of Menu 16
#define MENU_17_ITEM_NUM	0		//amount of items of Menu 17
#define MENU_10_PAGE_NUM        2

extern u8 selected;
extern u16 menu;
extern u16 curse;
extern u16 curse1[MENU_1_ITEM_NUM],page1[MENU_1_ITEM_NUM];
extern u8 item_renew_cnt;
extern u8 data_renew_cnt;

extern U8 menu_1_item_num[MENU_1_ITEM_NUM];
extern u8 Menu_0_item[8][8];
extern u8 Menu_1_item[8][10];
extern u8 Menu_10_item[8*MENU_10_PAGE_NUM][10];
extern u8 Menu_11_item[8][10];
extern u8 Menu_12_item[8][10];
extern u8 Menu_13_item[8][10];
extern u8 Menu_14_item[8][10];
extern U16* menu_10_pin[MENU_10_ITEM_NUM];
extern U16* menu_11_pin[MENU_11_ITEM_NUM];
extern U16* menu_12_pin[MENU_12_ITEM_NUM];
extern U16* menu_13_pin[MENU_13_ITEM_NUM];
extern U16* menu_14_pin[MENU_14_ITEM_NUM];
extern void (*menu_10_fpin[MENU_10_ITEM_NUM])(void);
extern void (*menu_11_fpin[MENU_11_ITEM_NUM])(void);

void item_display(void);
void data_display(u8);
void curse_display(void);



#endif
