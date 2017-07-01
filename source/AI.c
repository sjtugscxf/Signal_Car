// main AI
#include "includes.h"
#include <stdio.h>
#include <string.h>

//test

u8 redthre=120, CCDthre=60;
int straightspeed=30, leftaround=15, rightaround=22;   //L和R理论上的差速比例为3：5
int last_red_number=0, last_ycenter=0, last_xcenter=0;
int theroute=0;      //0左边过左转，1左边过右转，2右边过左转，3右边过右转
int route[10]={1,0,3,3,3,0,3,0,3,3};
int route_num=0;
bool far_flag=0;
int cnt_far=0;
bool go_near_flag=0;
void drawCam(bool(*isTarget)(u8 x));
void drawPlot();
void drawCCD();
void PWM(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R); //电机输出设置，left_speed right_speed 为左右速度期望值（给的参数符号正常）
                                                                 //（tacho0 为左编码器，向前为负，tacho1 为右编码器， 向前为正，左电机输出前进为正，右电机输出前进为负）
                                                                 // eg. PWM(80, 90, &L, &R)   **L, R 为已经定义了的两个结构体指针，调用PWM时不用管， pid参数的初始化在AI_Init()
                                                                 // AI_run() 中有调用的例子（需要修改），板子上靠边是tacho0
void PWMne(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R);

const char *stateTable[] = {
	"Nothing", "Turn Around",
	"Go To Red", "Go To Red Stop",
	"Avoid", "Return",
	"Go Back", "FINISH"
};
char OLED_Buffer[8][64];
char ui_buffer[8][64]; // 现在只用5个

State state;
PIDInfo L, R;  //两个结构体指针，存与电机pid控制有关的量， 包括pid三个参数，lastErr

extern int findl;
extern s16 middle_rec;
extern bool nearflag;

/////////////////////////////////////////////////////////////////////
//前进速度PID控制
void PWM(u8 left_speed, u8 right_speed, PIDInfo *L, PIDInfo *R)      //前进的PID控制
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
//后退速度PID控制
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
 * 图像相关
 */
bool isRed(u8 x) {              //红色阈值，根据实际情况调，
  return x > redthre;
}
bool isWhite(u8 x) {     //白色阈值，场地理想后好像没什么用
  return x < 100;
}
#define CAMERA_CENTER 40    //摄像头中心设在偏左，为了小车从灯旁边擦过而不用减速，实际效果还没有测试

#define RED_NUMBER_NO 0
#define RED_NUMBER_NEAR 12

/*
 * 控制相关
 */
#define DIR_MAX 230    //最大转弯，根据实际情况调

///////////////////////////////////////
///根据信标偏移程度确定舵机转角规划路径，左右不对称分开
void dirControlR(int row, int col) {
  int err = 0;
  
  if (row>53)   err = col - (87-row);
  else if (row>24)  err = col - (59-row*16/29) ;
  else err = col - 56;
  sprintf(OLED_Buffer[3], "err:    %6d  ", err);
  static int lastErr = 0;
  int pid;
  
  if(go_near_flag &&(err>15)|| (err<-15) ) pid = 5 * err;
  else pid = 3 * err;      //比例调节应该够了
  
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
  else pid = 3 * err;      //比例调节应该够了
  
  if (pid > DIR_MAX)  pid = DIR_MAX;
  if (pid < -DIR_MAX) pid = -DIR_MAX;
  Servo_Output(pid);
  lastErr = err;
}
///////////////////////////////////////////////////////////

/*
 *   AI 相关
 */
/////////////////////////////////////////////////////////////////////
// 扫描指定区域的色块信息
void scan(s8 top, u8 left, u8 right, BlockInfo *info, bool(*isTarget)(u8 x)) {
  int number;
  int xSum, xMin, xMax; // 现在只算了 xSum， yMin， yMax，其他先留着。
  int ySum, yMin, yMax;    //y方向跟远近有关，其实作用不大

  number = xSum = ySum = 0;
  xMin = IMG_COLS;
  yMin = IMG_ROWS; yMax = -1;

  for (s8 i = IMG_ROWS - 1; i >= top; i--) {
    for (u8 j = left; j < right; j++) {    //扫调用函数时给定的区域
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
  if (number) {                                  //找到
    info->xCenter = xSum / number;       //这个中心与摄像头中心的差距决定转向大小
    info->yCenter = ySum / number;
    info->left = xMin;
    info->top = yMin; info->bottom = yMax;
  }
}
//////////////////////////////////////////////////////////////

///////////////////PID参数初始化////////////////////////
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
  scan(BLACK_HEIGHT, 0, IMG_COLS, &red, isRed);    //扫描红色的点数
 
  if(red.number==0) { red.yCenter=last_ycenter ;  red.xCenter=last_xcenter; }
  
  
  if(red.yCenter>34&&red.yCenter<50) go_near_flag=1;   //根据Y中心坐标判断远近
  else go_near_flag=0;
  
  if((last_ycenter-red.yCenter)>20)   route_num=(route_num+1)%10;   //根据远近切换判断信标数
  
  if(red.number>0&&red.number<4&&red.yCenter<25)  {far_flag=1;cnt_far=0;}
  if(far_flag)
        {
          ++cnt_far;
          if(cnt_far>100) far_flag=0;
        }

  theroute=route[route_num];
  
  switch(state) // 状态机切换,只留下三种状态就好了，看到了走，看不到倒后转，起始状态不动
  {                         
    case NOTHING:
      if (red.number > RED_NUMBER_NO) {
		  state = GO_TO_RED;
		  cnt_light=0;
                   cnt_far=0;
      }
      break;
    
    case TURN_AROUND:   //LED1亮，开关2拨下启动
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
        // CCD避障        
        if(nearflag && red.yCenter<31) {                              //可避障距离
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
  // UI信息显示
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

void AI_ShowInfo() {   // 40ms一次的UI刷新循环 , 在PIT.c中调用
  if (SW3()!=0 && SW1()!=0) { //开关3 1没打开，显示数字类信息，在AI run的最后设置
    for (int i = 0; i < 8; i++)
      Oled_Putstr(i, 0, OLED_Buffer[i]);     
  } 
  else if (SW3()==0 && SW1()!=0) { //开关3打开 1没打开 画摄像头图像
    drawCam(isRed);   
  }
  else if (SW3()!=0 && SW1()==0) { //开关3没打开 1打开
//	drawCCD();
  }
  else { //开关3 1 打开，菜单切换
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

////////////画摄像头图像//////////////////
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