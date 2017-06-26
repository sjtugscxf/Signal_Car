/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"


// ====== Variables ======
// ---- Global ----
u8 cam_buffer_safe[BLACK_WIDTH*2];
u8 cam_buffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];   //64*155，把黑的部分舍去是59*128
Road road_B[ROAD_SIZE];//由近及远存放
float mid_ave;//road中点加权后的值
//float  weight[10] = {1,1,1.118, 1.454, 2.296, 3.744, 5.304, 6.000, 5.304, 3.744}; //2.296};//, 1.454};//上一次的权值
//float weight[10] = {1.04,1.14,1.41,2.01,3.03,4.35,5.52,6,5.52,4.35};//待测试
float weight[4][10] ={ {0,0,0,0,0,0,0,0,0,0},
                        {1,1,1,1,1,1,1,1,1,1},
                        {1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29},
                        {1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454}};
int valid_row=0;//与有效行相关，未有效识别
int valid_row_thr=10;//有效行阈值
u8 car_state=0;//智能车状态标志 0：停止  1：测试舵机  2：正常巡线
u8 remote_state = 0;//远程控制
u8 road_state = 0;//前方道路状态 1、直道   2、弯道  3、环岛  4、障碍
                  //3 4 状态下权重拉近
                  //2 状态下减速

float motor_L=MIN_SPEED;
float motor_R=MIN_SPEED;

//OLED调参
int debug_speed=0;
PIDInfo debug_dir;
int margin=30;
circle C;
int c1=15, c2=10, c3=5;

//=====================
int CAM_HOLE_ROW=27; //用来向两边扫描检测黑洞·环岛的cam_buffer行位置
int ROAD_OBST_ROW=10; //用来检测障碍物的road_B行位置//不能太远，也不能太近
int OBSTACLE_THR=40;  //有障碍物时赛道宽度阈值


// ---- Local ----
u8 cam_row = 0, img_row = 0;
/*
//——————透视变换·变量——————
double matrix[4][4];
//int buffer[64][128];=cam_buffer
int former[8200][4];
int later[8200][4];
u8 cam_buffer2[64][128];//int buffer2[64][128];
int visited[64][128];

//——————透视变换·函数——————

void getMatrix(double fov, double aspect, double zn, double zf){
    matrix[0][0] = 1 / (tan(fov * 0.5) *aspect) ;
    matrix[1][1] = 1 / tan(fov * 0.5) ;
    matrix[2][2] = zf / (zf - zn) ;
    matrix[2][3] = 1.0;
    matrix[3][2] = (zn * zf) / (zn - zf);
    return;
}

void linearization(){
    int cnt = 0;
    for (int i=0;i<64;i++){
        for (int j=0;j<128;j++){
            former[cnt][0] = i;
            former[cnt][1] = j;
            former[cnt][2] = cam_buffer[i][j];//buffer[i][j];
            former[cnt][3] = 1;
            cnt++;
        }
    }
    return;
}

void multiply(int k){
    for (int i=0;i<4;i++){
        later[k][i] = former[k][0]*matrix[0][i]+former[k][1]*matrix[1][i]+former[k][2]*matrix[2][i]+former[k][3]+matrix[3][i];
    }
    return;
}

void matrixMultiply(){
    for (int i=0;i<8192;i++){
        multiply(i);
    }
    return;
}

void getNewBuffer(){
    for (int i=0;i<64;i++){
        for (int j=0;j<128;j++){
            cam_buffer2[i][j] = 0;
        }
    }
    for (int i=0;i<8192;i++){
        cam_buffer2[later[i][0]][later[i][1]] = later[i][2];
    }
    return;
}
*/

// ====== 

void Cam_Algorithm(){
  static u8 img_row_used = 0;
  
  while(img_row_used ==  img_row%IMG_ROWS); // wait for a new row received
  
  // -- Handle the row --  
  
  if (img_row_used >= BLACK_HEIGHT) {     //前5行黑的不要
    for (int col = 0; col < IMG_COLS; col++) {
      u8 tmp = cam_buffer[img_row_used][col];
      if(!SW1()) UART_SendChar(tmp < 0xfe ? tmp : 0xfd);
    }
   if(!SW1()) UART_SendChar(0xfe);//0xfe->纯参数读取溢出
  }
  
  //  -- The row is used --
  img_row_used++;
  if (img_row_used == IMG_ROWS) {    //一帧图像完行归零，控制算法启动，进入AI_Run
    img_row_used = 0;

    if(!SW1()) UART_SendChar(0xff);//0xff->异常结束
  }//以上原来是SW1()
}

float constrain(float lowerBoundary, float upperBoundary, float input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

int constrainInt(int lowerBoundary, int upperBoundary, int input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
  double a,b,c,d,e,f;
  double r,x,y;
	
  a=2*(x2-x1);
  b=2*(y2-y1);
  c=x2*x2+y2*y2-x1*x1-y1*y1;
  d=2*(x3-x2);
  e=2*(y3-y2);
  f=x3*x3+y3*y3-x2*x2-y2*y2;
  x=(b*f-e*c)/(b*d-e*a);
  y=(d*c-a*f)/(b*d-e*a);
  r=sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
  x=constrain(-1000.0,1000.0,x);
  y=constrain(-1000.0,1000.0,y);
  r=constrain(1.0,500.0,r);
  bool sign = (x>0)?1:0;
  circle tmp = {r,sign};
  return tmp;
}

bool is_stop_line(int target_line)//目测并不有效……
{
  if((road_B[target_line].right-road_B[target_line].left)<ROAD_WID)
    return 1;
  else return 0;
}

double getSlope_(int x1, int y1, int x2, int y2)
{
  double dx = x2-x1;
  double dy = y2-y1;
  if(dy==0) return dx*100;
  else return (double)dx/dy;
}

void Cam_B_Init()//初始化Cam_B
{
  int i=0;
  for(i=0;i<ROAD_SIZE;i++)
  {
    road_B[i].left=CAM_WID/2;
    road_B[i].right=CAM_WID/2+2;
    road_B[i].mid=CAM_WID/2+1;
  }
  mid_ave=CAM_WID/2+1;
  //以下为road->mid加权值weight的初始化，由近到远
  //方案一：分段函数
  /*for(i=0;i<3;i++)
  {  
    weight[i]=1;
  }
  for(i=3;i<7;i++)
  {  
    weight[i]=2;
  }
  for(i=7;i<10;i++)
  {
    weight[i]=1;
  }*/
  
  //方案二：遵从正态分布，最高值在weight[MaxWeight_index]，在头文件定义相关参数//但是……无效……不知为何
/*  for(int i=0;i<10;i++)
  {
    weight[i]=1.0 + MaxWeight * exp(-(double)exp_k*pow((double)(i-MaxWeight_index),2.0)/2.0); //目前最高下标为常量
  }*/
  
  // design 3 ——>声明与定义放一块，global
//  weight = {1.118, 1.454, 2.296, 3.744, 5.304, 6.000, 5.304, 3.744, 2.296, 1.454};
  
}

    //出界保护  //无效……
  /*  static bool flag_protect=0;
    if((cam_buffer[56][60]+cam_buffer[57][60]+cam_buffer[58][60])<3*thr||flag_protect==1){                       //保护机制，貌似不太好用cam_buffer[45][64]<70 && cam_buffer[50][64]<70 &&
      Servo_Output(0);
      PWM(0, 0, &L, &R); 
      flag_protect=1;
    }*/

//test
double theta,theta_d,slope,test;
double x,y;

  //第一次进化版巡线程序
void Cam_B(){
  
    //===================变量定义====================
  
    float max_speed=MAX_SPEED+debug_speed;//最大速度
    static int dir;//舵机输出
    
    //================================透视变化
    //getMatrix(0.785398,1.0,1.0,1000);
   // linearization();
   // matrixMultiply();
   // getNewBuffer();
 
    //==================================获取road_B的left right mid 坐标和slope_
    //斜率方案
    /*
    for(int j=0;j<ROAD_SIZE;j++)//从下向上扫描
    {
      //double x,y;
      slope=road_B[j].slope_;
      theta=atan(road_B[j].slope_);//double theta=atan(road_B[j].slope_);
      test=sin(theta);
      test=cos(theta);
      test=tan(theta);
      theta_d=theta*180/PI;
      test=sin(theta_d);//以上用来检测atan函数以及sin函数使用的是否正确，debug模式下
      //left
      x=road_B[j].mid[0];
      y=road_B[j].mid[1];
      while(x>=0 && x<=CAM_WID && y<=CAM_NEAR && y>=CAM_FAR)
      {
        if(cam_buffer[(int)y][(int)x]<thr)
          break;
        else
        {
          x-=cos(theta);
          y-=sin(theta);
        }
      }
      road_B[j].left[0]=constrain(0,CAM_WID,x);
      road_B[j].left[1]=constrain(CAM_FAR,CAM_NEAR,y);

      //right
      x=road_B[j].mid[0];
      y=road_B[j].mid[1];
      while(x>=0 && x<=CAM_WID && y<=CAM_NEAR && y>=CAM_FAR)
      {
        if(cam_buffer[(int)y][(int)x]<thr)
          break;
        else
        {
          x+=cos(theta);
          y+=sin(theta);
        }
      }
      road_B[j].right[0]=constrain(0,CAM_WID,x);
      road_B[j].right[1]=constrain(CAM_FAR,CAM_NEAR,y);
      //mid
      road_B[j].mid[0] = (road_B[j].left[0] + road_B[j].right[0])/2;
      road_B[j].mid[1] = (road_B[j].left[1] + road_B[j].right[1])/2;//分别计算并存储25行的mid
      //slope
      if(j>0)
      {
        road_B[j].slope_=getSlope_(road_B[j-1].mid[0],-road_B[j-1].mid[1],road_B[j].mid[0],-road_B[j].mid[1]);
        road_B[j+1].slope_=road_B[j].slope_;//用以预测下一个mid
      }
      //mid of the next road_B[]
      if(j<(ROAD_SIZE-1))
      {
        road_B[j+1].mid[0]=road_B[j].mid[0]+CAM_STEP*sin(theta);
        road_B[j+1].mid[1]=road_B[j].mid[1]-CAM_STEP*cos(theta);
      }
    }
    */
    //横向扫描方案
    for(int j=0;j<ROAD_SIZE;j++)//从下向上扫描
    {
      int i;
      //left
      for (i = road_B[j].mid; i > 0; i--){
        if (cam_buffer[60-CAM_STEP*j][i] < thr)
          break;
        }
      road_B[j].left = i;
      //right
      for (i = road_B[j].mid; i < CAM_WID; i++){
        if (cam_buffer[60-CAM_STEP*j][i] < thr)
          break;
        }
      road_B[j].right = i;
      //mid
      road_B[j].mid = (road_B[j].left + road_B[j].right)/2;//分别计算并存储25行的mid
      //store
      if(j<(ROAD_SIZE-1))
        road_B[j+1].mid=road_B[j].mid;//后一行从前一行中点开始扫描
    }
      
    //===========================区分前方道路类型//需要设置一个优先级！！！
    static int mid_ave3;
    bool flag_valid_row=0;
    for(int i_valid=0;i_valid<(ROAD_SIZE-3) && flag_valid_row==0;i_valid++)
    {
      mid_ave3 = (road_B[i_valid].mid + road_B[i_valid+1].mid + road_B[i_valid+2].mid)/3;
      if(mid_ave3<margin||mid_ave3>(CAM_WID-margin))
      {
        flag_valid_row=1;
        valid_row=i_valid;
      }
      else valid_row=ROAD_SIZE-3;
    }
    if(valid_row<valid_row_thr)
      road_state=2;//弯道
    else road_state=1;//直道
    //detect the black hole————————————————————
    int left=0,right=0;
    if(cam_buffer[CAM_HOLE_ROW][CAM_WID/2]<thr)
    {
      //left
      int i=CAM_WID/2-1;
      while(i>0){
        if(left==0 && cam_buffer[CAM_HOLE_ROW][i]>thr){//是否考虑取平均防跳变？
          left++;
        }
        else if(left==1 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          left++;
        }
        i--;
      }
      //right
      i=CAM_WID/2+1;
      while(i<CAM_WID){
        if(right==0 && cam_buffer[CAM_HOLE_ROW][i]>thr){//是否考虑取平均防跳变？
          right++;
        }
        else if(right==1 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          right++;
        }
        i++;
      }
    }
    if(left>=1 && right>=1)
      road_state=3;//前方环岛
    //detect the obstacle————————————————————
  /*  if((road_B[ROAD_OBST_ROW].right-road_B[ROAD_OBST_ROW].left)<OBSTACLE_THR)
    {
      int i=road_B[ROAD_OBST_ROW].mid;
      left=0;
      right=0;
      //left
      while(i>0){
        if(left==0 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          left++;
        }
        else if(left==1 && cam_buffer[CAM_HOLE_ROW][i]>thr){
          left++;
        }
        else if(left==2 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          left++;
        }
        i--;
      }
      //right
      while(i<CAM_WID){
        if(right==0 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          right++;
        }
        else if(right==1 && cam_buffer[CAM_HOLE_ROW][i]>thr){
          right++;
        }
        else if(right==2 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          right++;
        }
        i++;
      }
      if(left>=3 || right>=3)
        road_state=4;
    }*/
    
  /*  //=============================根据前方道路类型，选择不同的权值weight
     switch(road_state)
    {
      case 1: 
        for(int i=0;i<10;i++)weight[i]=1;//均匀分布的权值
        break;
      case 2:
        max_speed=MAX_SPEED-5;//减多少未定，取决于弯道最高速度
        float weight2[10] = {1.00,1.03,1.14,1.54,2.56,4.29,6.16,7.00,6.16,4.29};
        for(int i;i<10;i++) weight[i] = weight2[i];//正态分布的权值
        break;
      case 3:
        max_speed=MAX_SPEED-5;
        float  weight3[10] = {1.118, 1.454, 2.296, 3.744, 5.304, 6.000, 5.304, 3.744, 2.296, 1.454};//未确定
        for(int i;i<10;i++) weight[i] = weight2[i];
        break;
      case 4:
        break;
      default:break;
    }*/
    
    //================================对十行mid加权：
    float weight_sum=0;
    for(int j=0;j<10;j++)
    {
      mid_ave += road_B[j].mid * weight[road_state][j];
      weight_sum += weight[road_state][j];
    }
    mid_ave/=weight_sum;
    
    //=================================舵机的PD控制
    static float err;
    static float last_err;
    err = mid_ave  - CAM_WID / 2;

    dir = (Dir_Kp+debug_dir.kp) * err + (Dir_Kd+debug_dir.kd) * (err-last_err);     //舵机转向  //参数: (7,3)->(8,3.5)
    if(dir>0)
      dir*=1.2;//修正舵机左右不对称的问题//不可删
    last_err = err;
    
    dir=constrainInt(-230,230,dir);
    if(car_state!=0)
      Servo_Output(dir);
    else   
      Servo_Output(0);
    
    
    
    //==============速度控制=================
    //PWM以dir为参考，前期分级控制弯道速度；中期分段线性控速；后期找到合适参数的时候，再进行拟合——PWM关于dir的函数
    float range=max_speed-MIN_SPEED;//速度范围大小 
    if(car_state==2 ){
      //分段线性控速
      if(abs(dir)<50 ){//&& valid_row>valid_row_thr
        motor_L=motor_R=max_speed;
      }
      else if(abs(dir)<95){
        motor_L=motor_R=max_speed-0.33*range*(abs(dir)-50)/45;
        if(dir>0) motor_R=constrain(MIN_SPEED,motor_R,motor_R*0.9);//右转
        else motor_L=constrain(MIN_SPEED,motor_L,motor_L*0.9);//0.9
      }
      else if(abs(dir)<185){    
        motor_L=motor_R=max_speed-0.33*range-0.33*range*(abs(dir)-95)/90;
        if(dir>0) motor_R=constrain(MIN_SPEED,motor_R,motor_R*0.8);//右转
        else motor_L=constrain(MIN_SPEED,motor_L,motor_L*0.8);//0/75
      }
      else if(abs(dir)<230){
        motor_L=motor_R=max_speed-0.66*range-0.33*range*(abs(dir)-185)/45;
        if(dir>0) motor_R=constrain(MIN_SPEED,motor_R,motor_R*0.7);//右转
        else motor_L=constrain(MIN_SPEED,motor_L,motor_L*0.7);//0.5
      }//以上的差速控制参数未确定，调参时以车辆稳定行驶为目标
      else{
        motor_L=motor_R=MIN_SPEED;
      }
      PWM(motor_L, motor_R, &L, &R);               //后轮速度
    }
   else
     PWM(0, 0, &L, &R);
    
    //方案二//暂时放弃
    //C=getR(road_B[c1].mid,20-c1,road_B[c2].mid,20-c2,road_B[c3].mid,20-c3);
    
}




// ====== Basic Drivers ======

void PORTC_IRQHandler(){
  if((PORTC->ISFR)&PORT_ISFR_ISF(1 << 8)){  //CS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 8);
    if(img_row < IMG_ROWS && cam_row % IMG_STEP == 0 ){
      DMA0->TCD[0].DADDR = (u32)&cam_buffer[img_row][-BLACK_WIDTH];
      DMA0->ERQ |= DMA_ERQ_ERQ0_MASK; //Enable DMA0
      ADC0->SC1[0] |= ADC_SC1_ADCH(4); //Restart ADC
      DMA0->TCD[0].CSR |= DMA_CSR_START_MASK; //Start
    }
    cam_row++;
  }
  else if(PORTC->ISFR&PORT_ISFR_ISF(1 << 9)){   //VS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 9);
    cam_row = img_row = 0;
  }
}

void DMA0_IRQHandler(){
  DMA0->CINT &= ~DMA_CINT_CINT(7); //Clear DMA0 Interrupt Flag
  
  img_row++; 
}

void Cam_Init(){
  // --- IO ---
  
  PORTC->PCR[8] |= PORT_PCR_MUX(1); //cs
  PORTC->PCR[9] |= PORT_PCR_MUX(1); //vs
  PORTC->PCR[11] |= PORT_PCR_MUX(1);    //oe
  PTC->PDDR &=~(3<<8);
  PTC->PDDR &=~(1<<11);
  PORTC->PCR[8] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(10);	//PULLUP | falling edge
  PORTC->PCR[9] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(9);  // rising edge
  PORTC->PCR[11] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  
  NVIC_EnableIRQ(PORTC_IRQn);
  NVIC_SetPriority(PORTC_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
  
  // --- AD ---
  
  /*
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  //ADC1 Clock Enable
  ADC0->CFG1 |= 0
             //|ADC_CFG1_ADLPC_MASK
             | ADC_CFG1_ADICLK(1)
             | ADC_CFG1_MODE(0);     // 8 bits
             //| ADC_CFG1_ADIV(0);
  ADC0->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_MUXSEL_MASK |  // b
                ADC_CFG2_ADACKEN_MASK; 
  
  ADC0->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
  
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA
  
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; // continuous
  
  //PORTC->PCR[2]|=PORT_PCR_MUX(0);//adc1-4a
  
  ADC0->SC1[0] |= ADC_SC1_ADCH(4);
  */
  
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; //ADC1 Clock Enable
  ADC0->SC1[0] &= ~ADC_SC1_AIEN_MASK; //ADC1A
  ADC0->SC1[0] = 0x00000000; //Clear
  ADC0->SC1[0] |= ADC_SC1_ADCH(4); //ADC1_5->Input, Single Pin, No interrupt
  ADC0->SC1[1] &= ~ADC_SC1_AIEN_MASK; //ADC1B
  ADC0->SC1[1] |= ADC_SC1_ADCH(4); //ADC1_5b
  ADC0->SC2 &= 0x00000000; //Clear all.
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA, SoftWare
  ADC0->SC3 &= (~ADC_SC3_AVGE_MASK&~ADC_SC3_AVGS_MASK); //hardware average disabled
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; //Continuous conversion enable
  ADC0->CFG1|=ADC_CFG1_ADICLK(1)|ADC_CFG1_MODE(0)|ADC_CFG1_ADIV(0);//InputClk, ShortTime, 8bits, Bus
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK; //ADC1  b
  ADC0->CFG2 |= ADC_CFG2_ADACKEN_MASK; //OutputClock
    
  // --- DMA ---
  
  SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK; //DMAMUX Clock Enable
  SIM->SCGC7 |= SIM_SCGC7_DMA_MASK; //DMA Clock Enable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(40); //DMA0->No.40 request, ADC0
  DMA0->TCD[0].SADDR = (uint32_t) & (ADC0->R[0]); //Source Address 0x400B_B010h
  DMA0->TCD[0].SOFF = 0; //Source Fixed
  DMA0->TCD[0].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0); //Source 8 bits, Aim 8 bits
  DMA0->TCD[0].NBYTES_MLNO = DMA_NBYTES_MLNO_NBYTES(1); //one byte each
  DMA0->TCD[0].SLAST = 0; //Last Source fixed
  DMA0->TCD[0].DADDR = (u32)cam_buffer;
  DMA0->TCD[0].DOFF = 1;
  DMA0->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(IMG_COLS+BLACK_WIDTH);
  DMA0->TCD[0].DLAST_SGA = 0;
  DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(IMG_COLS+BLACK_WIDTH);
  DMA0->TCD[0].CSR = 0x00000000; //Clear
  DMA0->TCD[0].CSR |= DMA_CSR_DREQ_MASK; //Auto Clear
  DMA0->TCD[0].CSR |= DMA_CSR_INTMAJOR_MASK; //Enable Major Loop Int
  DMA0->INT |= DMA_INT_INT0_MASK; //Open Interrupt
  //DMA->ERQ&=~DMA_ERQ_ERQ0_MASK;//Clear Disable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK; //Enable
  
  NVIC_EnableIRQ(DMA0_IRQn);
  NVIC_SetPriority(DMA0_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
}