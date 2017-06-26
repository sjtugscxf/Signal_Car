// main AI
#include "includes.h"
#include <stdio.h>
#include <string.h>

void drawCam(bool(*isTarget)(u8 x));
void drawPlot();

const char *stateTable[] = {
	"Nothing", "Turn Around",
	"Go To Red", "Go To Red Stop",
	"Avoid", "Return",
	"Go Back", "FINISH"
};
char OLED_Buffer[8][64];

int _protect_[10];  // 此处IAR有bug！！！全局变量定义都放着这里面，而且要定义在motorPID的前面
State state;
PIDInfo motorPID[3];
int _protect_[10];

extern int findl;
extern s16 middle_rec;
extern bool nearflag;
s16 cammax=0;


/*
 * 图像相关
 */
bool isRed(u8 x) {              //红色阈值，根据实际情况调，
  return x > 120;
}
bool isWhite(u8 x) {     //白色阈值，场地理想后好像没什么用
  return x < 100;
}

u8 plotBuffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];








