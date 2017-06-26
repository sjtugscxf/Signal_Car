#ifndef _AI_
#define _AI_

void AI_Run();
void AI_Init();
void motorControl(void);
void AI_ShowInfo();

extern s16 ccd1_linethre[128];
extern u8 uiphrase, uivar;
extern int theroute;
extern int route[10];
extern u8 redthre, CCDthre;
extern int straightspeed, leftaround, rightaround;

typedef enum {
	NOTHING, TURN_AROUND,
	GO_TO_RED, GO_TO_RED_STOP,
	AVOID, RETURN,
	GO_BACK, FINISH
} State;

typedef struct {   // ��ɫ�ϰ�����Ϣ�� ��Щ�ֶ�û�õ���������
	int number;
	int xCenter, left, right,last_xCenter,last_yCenter;
	int yCenter, top, bottom;
} BlockInfo;

typedef struct {
	double kp, ki, kd;
	double lastErr, llastErr;
	double pid;
    double errSum;
} PIDInfo;

#endif