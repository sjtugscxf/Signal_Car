#ifndef OLED_UI_H
#define OLED_UI_H

#define Rows 26
#define Pages 3

//=== Global variables===
extern u8 oled_menu;


void UI_SystemInfo();
void displayMenu();//menu==0
void displayParameters();//menu==1
void displayCamera();//menu==2
void displayDebug();//menu==3

void drawCam(bool(*isTarget)(u8 x));//��ֵ��
void drawCam2(bool(*isTarget)(u8 x));//͸�ӱ任
bool isWhite(u8 x);


#endif