#ifndef _AI_
#define _AI_

void AI_Run();
void AI_Init();
void motorControl(void);
void AI_ShowInfo();


typedef enum {
	NOTHING, TURN_AROUND,
	GO_TO_RED, GO_TO_RED_STOP,
	AVOID, RETURN,
	GO_BACK, FINISH
} State;

typedef struct {   // ��ɫ�ϰ�����Ϣ�� ��Щ�ֶ�û�õ���������
	int number;
	int xCenter, left, right,last_xCenter;
	int yCenter, top, bottom;
} BlockInfo;


#endif