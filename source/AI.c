// main AI
#include "includes.h"
#include <stdio.h>
#include <string.h>

//test

u8 redthre=120, CCDthre=60;
int straightspeed=30, leftaround=15, rightaround=22;   //L��R�����ϵĲ��ٱ���Ϊ3��5
int last_red_number=0, last_ycenter=0, last_xcenter=0;
int theroute=0;      //0��߹���ת��1��߹���ת��2�ұ߹���ת��3�ұ߹���ת
int route[10]={1,0,3,3,3,0,3,0,3,3};
int route_num=0;
bool far_flag=0;
int cnt_far=0;
bool go_near_flag=0;
void drawCam(bool(*isTarget)(u8 x));
void drawPlot();
void drawCCD();
void PWM(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R); //���������ã�left_speed right_speed Ϊ�����ٶ�����ֵ�����Ĳ�������������
                                                                 //��tacho0 Ϊ�����������ǰΪ����tacho1 Ϊ�ұ������� ��ǰΪ�����������ǰ��Ϊ�����ҵ�����ǰ��Ϊ����
                                                                 // eg. PWM(80, 90, &L, &R)   **L, R Ϊ�Ѿ������˵������ṹ��ָ�룬����PWMʱ���ùܣ� pid�����ĳ�ʼ����AI_Init()
                                                                 // AI_run() ���е��õ����ӣ���Ҫ�޸ģ��������Ͽ�����tacho0
void PWMne(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R);

const char *stateTable[] = {
	"Nothing", "Turn Around",
	"Go To Red", "Go To Red Stop",
	"Avoid", "Return",
	"Go Back", "FINISH"
};
char OLED_Buffer[8][64];
char ui_buffer[8][64]; // ����ֻ��5��

State state;
PIDInfo L, R;  //�����ṹ��ָ�룬������pid�����йص����� ����pid����������lastErr

extern int findl;
extern s16 middle_rec;
extern bool nearflag;

/////////////////////////////////////////////////////////////////////
//ǰ���ٶ�PID����
void PWM(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R)      //ǰ����PID����
{  
  double L_err=left_speed+tacho0;
  double R_err=right_speed-tacho1;
  L->errSum+=L_err;
  if(L->errSum>300) L->errSum=300;
  if(L->errSum<-300) L->errSum=-300;
  R->errSum+=R_err;
  if(R->errSum>300) R->errSum=300;
  if(R->errSum<-300)R->errSum=-300;
  double L_pwm=(L_err*L->kp + L->errSum*L->ki + (L_err-L->lastErr)*L->kd);
  double R_pwm=(R_err*R->kp + R->errSum*R->ki + (R_err-R->lastErr)*R->kd);
  L->lastErr=L_err;
  R->lastErr=R_err;
  
  if(L_pwm>750)  L_pwm=750;
  if(R_pwm>750)  R_pwm=750;
  if(L_pwm<-750)  L_pwm=-750;
  if(R_pwm<-750)  R_pwm=-750;
  MotorL_Output((int)(L_pwm)); 
  MotorR_Output((int)(-R_pwm));
}
////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
//�����ٶ�PID����
void PWMne(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R)  
{
  double L_err=left_speed-tacho0;
  double R_err=right_speed+tacho1;
  L->errSum+=L_err;
  if(L->errSum>300) L->errSum=300;
  if(L->errSum<-300) L->errSum=-300;
  R->errSum+=R_err;
  if(R->errSum>300) R->errSum=300;
  if(R->errSum<-300)R->errSum=-300;
  double L_pwm=(L_err*L->kp + L->errSum*L->ki + (L_err-L->lastErr)*L->kd);
  double R_pwm=(R_err*R->kp + R->errSum*R->ki + (R_err-R->lastErr)*R->kd);
  L->lastErr=L_err;
  R->lastErr=R_err;
  
  if(L_pwm>750)  L_pwm=750;
  if(R_pwm>750)  R_pwm=750;
  if(L_pwm<-750)  L_pwm=-750;
  if(R_pwm<-750)  R_pwm=-750;
  MotorL_Output((int)(-L_pwm)); 
  MotorR_Output((int)(R_pwm));
}
/////////////////////////////////////////////////////////////////////

/*
 * ͼ�����
 */
bool isRed(u8 x) {              //��ɫ��ֵ������ʵ���������
  return x > redthre;
}
bool isWhite(u8 x) {     //��ɫ��ֵ��������������ûʲô��
  return x < 100;
}
#define CAMERA_CENTER 40    //����ͷ��������ƫ��Ϊ��С���ӵ��Ա߲��������ü��٣�ʵ��Ч����û�в���

#define RED_NUMBER_NO 0
#define RED_NUMBER_NEAR 12

/*
 * �������
 */
#define DIR_MAX 230    //���ת�䣬����ʵ�������

///////////////////////////////////////
///�����ű�ƫ�Ƴ̶�ȷ�����ת�ǹ滮·�������Ҳ��ԳƷֿ�
void dirControlR(int row, int col) {
  int err = 0;
  
  if (row>53)   err = col - (87-row);
  else if (row>24)  err = col - (59-row*16/29) ;
  else err = col - 56;
  sprintf(OLED_Buffer[3], "err:    %6d  ", err);
  static int lastErr = 0;
  int pid;
  
  if(go_near_flag &&(err>15)|| (err<-15) ) pid = 5 * err;
  else pid = 3 * err;      //��������Ӧ�ù���
  
  if (pid > DIR_MAX)  pid = DIR_MAX;
  if (pid < -DIR_MAX) pid = -DIR_MAX;
  Servo_Output(pid);
  lastErr = err;
}

void dirControlL(int row, int col) {
  int err = 0;
  
  if (row>53)   err = col - (row+33);
  else if (row>24)  err = col - (row*16/29+61) ;
  else err = col - 64;
  sprintf(OLED_Buffer[3], "err:    %6d  ", err);
  static int lastErr = 0;
  int pid;
  
  if(go_near_flag && (err>15)||(err<-15)) pid = 5 * err;
  else pid = 3 * err;      //��������Ӧ�ù���
  
  if (pid > DIR_MAX)  pid = DIR_MAX;
  if (pid < -DIR_MAX) pid = -DIR_MAX;
  Servo_Output(pid);
  lastErr = err;
}
///////////////////////////////////////////////////////////

/*
 *   AI ���
 */
/////////////////////////////////////////////////////////////////////
// ɨ��ָ�������ɫ����Ϣ
void scan(s8 top, u8 left, u8 right, BlockInfo *info, bool(*isTarget)(u8 x)) {
  int number;
  int xSum, xMin, xMax; // ����ֻ���� xSum�� yMin�� yMax�����������š�
  int ySum, yMin, yMax;    //y�����Զ���йأ���ʵ���ò���

  number = xSum = ySum = 0;
  xMin = IMG_COLS;
  yMin = IMG_ROWS; yMax = -1;

  for (s8 i = IMG_ROWS - 1; i >= top; i--) {
    for (u8 j = left; j < right; j++) {    //ɨ���ú���ʱ����������
      if (isTarget(cam_buffer[i][j])) {
        number++;
        xSum += j;
        ySum += i;

        if (yMax == -1) yMax = i;
        yMin = i;
        if (j < xMin) xMin = j;
      }
    }
  }

  info->number = number;
  if (number) {                                  //�ҵ�
    info->xCenter = xSum / number;       //�������������ͷ���ĵĲ�����ת���С
    info->yCenter = ySum / number;
    info->left = xMin;
    info->top = yMin; info->bottom = yMax;
  }
}
//////////////////////////////////////////////////////////////

///////////////////PID������ʼ��////////////////////////
void AI_Init() 
{
  state =NOTHING ;

  L.kp = 5;
  L.ki = 2;
  L.kd = 0;
  R.kp = 5;
  R.ki = 2;
  R.kd = 0;
  
  L.lastErr=0;
  L.errSum=0;
  R.lastErr=0;
  R.errSum=0;
  
}

#define MOTORL_VAL (550)
#define MOTORR_VAL (-550)

void AI_Run() {
  BlockInfo red, white;
  extern int cnt_light;
  scan(BLACK_HEIGHT, 0, IMG_COLS, &red, isRed);    //ɨ���ɫ�ĵ���
 
  if(red.number==0) { red.yCenter=last_ycenter ;  red.xCenter=last_xcenter; }
  
  
  if(red.yCenter>34&&red.yCenter<50) go_near_flag=1;   //����Y���������ж�Զ��
  else go_near_flag=0;
  
  if((last_ycenter-red.yCenter)>20)   route_num=(route_num+1)%10;   //����Զ���л��ж��ű���
  
  if(red.number>0&&red.number<4&&red.yCenter<25)  {far_flag=1;cnt_far=0;}
  if(far_flag)
        {
          ++cnt_far;
          if(cnt_far>100) far_flag=0;
        }

  theroute=route[route_num];
  
  switch(state) // ״̬���л�,ֻ��������״̬�ͺ��ˣ��������ߣ�����������ת����ʼ״̬����
  {                         
    case NOTHING:
      if (red.number > RED_NUMBER_NO) {
		  state = GO_TO_RED;
		  cnt_light=0;
                   cnt_far=0;
      }
      break;
    
    case TURN_AROUND:   //LED1��������2��������
      LED1(0);
      if (SW4()) 
    { 
         if (theroute==0||theroute==2)
         {
            Servo_Output(-DIR_MAX);
            PWM(leftaround, rightaround, &L, &R);
            
         }
         else
         {
           Servo_Output(DIR_MAX);
             PWM(rightaround, leftaround, &L, &R);
         }

    }
      
      if (red.number > RED_NUMBER_NO) {
		state = GO_TO_RED;
		cnt_light=0;
                 cnt_far=0;
	  }
      break;
    
    case GO_TO_RED:
      LED1(1);
      if (SW4()) {
        // CCD����        
        if(nearflag && red.yCenter<31) {                              //�ɱ��Ͼ���
          LED2(0);
           if(middle_rec<64)  Servo_Output(5*(middle_rec-36));
           else Servo_Output(-5*(90-middle_rec));
        }
        else   
        {
          LED2(1);
          if(theroute==0||theroute==1) dirControlL(red.yCenter, red.xCenter);
          else dirControlR(red.yCenter, red.xCenter);
        }       
       PWM(straightspeed, straightspeed, &L, &R);
      }
      if (red.number <= RED_NUMBER_NO)
      {
        if(far_flag)
        {
          
        }
        else 
        {++cnt_light;
        if (cnt_light>4) state = TURN_AROUND;
        }
      }
      break;

    default:
      break;
  }
  last_red_number=red.number; 
  last_ycenter=red.yCenter;
  last_xcenter=red.xCenter;
  // UI��Ϣ��ʾ
if (SW2()) 
{  
  sprintf(OLED_Buffer[0], "redNumber: %6d  ", red.number);
  sprintf(OLED_Buffer[1], "rexCenter: %6d  ", red.xCenter);
  sprintf(OLED_Buffer[2], "reyCenter: %6d  ", red.yCenter);
 // sprintf(OLED_Buffer[3], "err:    %6d  ", err);
  sprintf(OLED_Buffer[4], "theroute:    %6d  ", theroute);
  sprintf(OLED_Buffer[5], "middle_rec: %6d  ", middle_rec);
  sprintf(OLED_Buffer[6], "near: %6d  ", go_near_flag);
  sprintf(OLED_Buffer[7], "route_num: %6d  ", route_num);

  if (uivar<6)
  {
    sprintf(ui_buffer[0], "redthre: %4d ", redthre);
    sprintf(ui_buffer[1], "CCDthre: %4d ", CCDthre);
    sprintf(ui_buffer[2], "straightSp: %4d ", straightspeed);
    sprintf(ui_buffer[3], "leftaround: %4d ", leftaround);
    sprintf(ui_buffer[4], "rightaround: %4d ", rightaround);
    sprintf(ui_buffer[5], "theroute: %4d ", theroute);
  }	
  else if(uivar<11)
  {
    sprintf(ui_buffer[0], "route0: %4d ", route[0]);
    sprintf(ui_buffer[1], "route1: %4d ", route[1]);
    sprintf(ui_buffer[2], "route2: %4d ", route[2]);
    sprintf(ui_buffer[3], "route3: %4d ", route[3]);
    sprintf(ui_buffer[4], "route4: %4d ", route[4]);
  }
  else
  {
    sprintf(ui_buffer[0], "route5: %4d ", route[5]);
    sprintf(ui_buffer[1], "route6: %4d ", route[6]);
    sprintf(ui_buffer[2], "route7: %4d ", route[7]);
    sprintf(ui_buffer[3], "route8: %4d ", route[8]);
    sprintf(ui_buffer[4], "route9: %4d ", route[9]);
  }
}

}

void AI_ShowInfo() {   // 40msһ�ε�UIˢ��ѭ�� , ��PIT.c�е���
  if (SW3()!=0 && SW1()!=0) { //����3 1û�򿪣���ʾ��������Ϣ����AI run���������
    for (int i = 0; i < 8; i++)
      Oled_Putstr(i, 0, OLED_Buffer[i]);     
  } 
  else if (SW3()==0 && SW1()!=0) { //����3�� 1û�� ������ͷͼ��
    drawCam(isRed);   
  }
  else if (SW3()!=0 && SW1()==0) { //����3û�� 1��
//	drawCCD();
  }
  else { //����3 1 �򿪣��˵��л�
      Oled_Clear();
      if (uivar<6)
      {
        if (uiphrase==0) {
            for (int i=0; i<6; i++) Oled_Putstr(i,2,ui_buffer[i]);
		  Oled_Putstr(uivar, 0, "o");
	  }
	  else {
		  Oled_Putstr(uivar,2,ui_buffer[uivar]);
	  }
      }
      else if(uivar<11)
      {
        if (uiphrase==0) {
            for (int i=0; i<5; i++) Oled_Putstr(i,2,ui_buffer[i]);
		  Oled_Putstr(uivar-6, 0, "o");
	  }
	  else {
		  Oled_Putstr(uivar-6,2,ui_buffer[uivar-6]);
	  }
      }
      else 
      {
        if (uiphrase==0) {
            for (int i=0; i<5; i++) Oled_Putstr(i,2,ui_buffer[i]);
		  Oled_Putstr(uivar-11, 0, "o");
	  }
	  else {
		  Oled_Putstr(uivar-11,2,ui_buffer[uivar-11]);
	  }
      }
  } 
  
  return;
} 

////////////������ͷͼ��//////////////////
void drawCam(bool(*isTarget)(u8 x)) {
  int row, col, i;
  u8 buf[1024];
  u8 *p = buf;
    
  for (row = IMG_ROWS-1; row >= 0; row -= 8) {
    for (col = IMG_COLS-1; col >= 0 ; col--) {
      u8 tmp = 0;
      for (i = 0; i < 8; i++) {
        tmp <<= 1;
        if (isTarget(cam_buffer[row-i][col]))
          tmp |= 0x01;
      }
      *p++ = tmp;
    }
  }
  
  Oled_DrawBMP(0, 0, 128, 64, buf);
}
/*
void drawCCD() {
  u8 tmp=0x11111111;
  u8 buf[1024]={0};
  
  int i;
  for (i=0;i<128;i++) {
    if (ccd1_linethre[i]<0) ;
    else {
  	  buf[128*(ccd1_linethre[i]/10)+i]=tmp;
    }
	
	buf[128*(CCDthre/10)+i]=tmp;
  }
  
  Oled_DrawBMP(0, 0, 128, 64, buf);
}
*/