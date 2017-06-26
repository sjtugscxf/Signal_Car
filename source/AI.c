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

int _protect_[10];  // �˴�IAR��bug������ȫ�ֱ������嶼���������棬����Ҫ������motorPID��ǰ��
State state;
PIDInfo motorPID[3];
int _protect_[10];

extern int findl;
extern s16 middle_rec;
extern bool nearflag;
s16 cammax=0;


/*
 * ͼ�����
 */
bool isRed(u8 x) {              //��ɫ��ֵ������ʵ���������
  return x > 120;
}
bool isWhite(u8 x) {     //��ɫ��ֵ��������������ûʲô��
  return x < 100;
}

u8 plotBuffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];








