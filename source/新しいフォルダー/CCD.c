#include "includes.h"
u8 oecnt=0;

uint32 PIT1_Basic_Value,last_pit1_value;

uint8 CMOS_Data[LINES_PER_FIELD][3];
uint16 CMOS_Data_Index;

uint8 CMOS_Invalid_Line_Count;

uint8 CMOS_Temp[16];
uint8 CMOS_Temp_Index;

uint8 CMOS_Last_MidLine;

uint8 CMOS_Sample_Flag;

int16 oled_debug = 0;

uint8 cscnt=0;
int16 oled_debug2=0;
void DAC_Init(){
  //Vout= Vin* (1 + DACDAT0[11:0])/4096
  SIM->SCGC2 |= SIM_SCGC2_DAC0_MASK;   //开启DAC0时钟
  DAC0->DAT[0].DATH = 0X03;  // output 3.3/(DATH:DATL)V
  DAC0->DAT[0].DATL = 0XFF;  // 
  
  DAC0->C1 = 0X00; //DMA禁止，缓冲区禁止
  DAC0->C2 = 0X00; //缓冲区读指针指向第一个数据，缓冲区大小为1word
  DAC0->C0 = 0XF0; //模块使能，选择VDDA为参考电压，软件触发
   
}

void CCD_Init(){
  int i=0;
  DAC_Init();
  SIM->SCGC4 |= SIM_SCGC4_CMP_MASK;
  PORTC->PCR[7] = PORT_PCR_MUX(0);  //COMPARATOR MUX
  PTC->PDDR &= ~0X07;
  
  CMP0->CR1 = 0X10;  // 先关闭模块及其使能
  CMP0->CR0 |= CMP_CR0_FILTER_CNT(4); // 先将滤波器count置零
  
  CMP0->FPR = 0X02; // 滤波周期（待定）= TPD + (CR0[FILTER_CNT] *TSAMPLE) + Tper
  CMP0->SCR = CMP_SCR_IER_MASK|CMP_SCR_IEF_MASK|CMP_SCR_CFR_MASK|CMP_SCR_CFF_MASK;  // 开启上升下降沿中断，禁止DMA方式
  CMP0->MUXCR = 0XCD; // 正反向复用使能，正向输入IN1(VEDIO)，反向输入IN5（DAC)
  CMP0->CR0 |= 0X40; // 四次连续采样相同则输出
  CMP0->CR1 = 0X11; // ,SE WE禁止模块使能，高速比较模式，结果不翻转，输出为COUT，输出引脚禁止
  CMOS_Temp_Index=0;
  CMOS_Data_Index=0;
  CMOS_Invalid_Line_Count=0;
  CMOS_Last_MidLine = CMOS_POINT_PER_LINE/2;
  CMOS_Data[0][1]=3200/2;  // intialize the first midline
  CMOS_Sample_Flag = 0;
  enable_irq(59);
  for(i;i<LINES_PER_FIELD;i++){
     CMOS_Data[i][0]=0; 
     CMOS_Data[i][1]=0;
     CMOS_Data[i][2]=0;
  }
  
}

void CCD_SetThreshold(u16 data){
  DAC0->DAT[0].DATH = (u8)(data>>8);  // output 3.3/4096V
  DAC0->DAT[0].DATL = (u8)(data&0xff);  // output 3.3/4096V
}



void CMP0_IRQHandler(){
  uint8 temp;
  //if(CMP0->SCR & CMP_SCR_CFR_MASK){  // rising edge interrupt
  // do something here
  CMP0->SCR |= CMP_SCR_CFR_MASK;  // clear the flag
  //else if(CMP0->SCR & CMP_SCR_CFF_MASK){ // falling edge interrupt
  //do something here
  CMP0->SCR &= ~CMP_SCR_CFF_MASK;  // clear the flag
  if(CMOS_Sample_Flag){
    temp = (uint8)((PIT1_Basic_Value-PIT1_Value())/13);
    
    // get the raw data from cmp
    if(temp<CMOS_Last_MidLine) {    
      CMOS_Data[CMOS_Data_Index][0] = temp; // blackL
    }
    else if(temp>=CMOS_Last_MidLine){
      CMOS_Data[CMOS_Data_Index][2] = temp; // blackR 
      CMOS_Sample_Flag = 0;      // get the blackR position, stop sampling this line
    }
  }
  
  // only used to debug
  /*temp = (uint8)((PIT1_Basic_Value-PIT1_Value())/13);
  CMOS_Data[CMOS_Data_Index][CMOS_Temp_Index]=temp;
  CMOS_Temp_Index=2;
  LED2_Tog();*/
  
}


void PORTC_IRQHandler(){
  int16 temp;
  uint16 blackL,blackR;
  if(PORTC->ISFR&(1<<6)){
    PORTC->ISFR |=(1<<6);
    //VOE_ISR();
    oecnt++; 
    if(oecnt==50){
      oecnt=0;
      LED1_Tog();
    }
    CMOS_Data_Index = 0;
    CMOS_Invalid_Line_Count=0;
    CMOS_Control();
    oled_debug = cscnt;
    cscnt = 0;
  }
  
  
  if(PORTC->ISFR&(1<<5)){
    PORTC->ISFR |=(1<<5);
    //VCS_ISR();
    if(CMOS_Invalid_Line_Count<CMOS_INVALID_LINE_COUNT){  // ignore the invalid lines
      CMOS_Invalid_Line_Count++; 
      last_pit1_value = PIT1_Value();
    }
    else if(CMOS_Data_Index<(LINES_PER_FIELD-1)){
      PIT1_Basic_Value = PIT1_Value();
      temp = (int16)((last_pit1_value-PIT1_Basic_Value)/13)-CMOS_POINT_PER_LINE;
      last_pit1_value = PIT1_Basic_Value;
      if(temp<CMOS_POINT_ERROR && temp>-CMOS_POINT_ERROR){   // valid line
         blackL = CMOS_Data[CMOS_Data_Index][0];
         blackR = CMOS_Data[CMOS_Data_Index][2];
         
         //oled_debug = blackR;
         // both lines are in sight
         if(blackL>CMOS_INVALID_LEFT_POINT_COUNT && blackR<CMOS_INVALID_RIGHT_POINT_COUNT){  
            CMOS_Data[CMOS_Data_Index][1] = (blackL+blackR)/2;
         }
         // left line are out of sight
         else if(blackL<CMOS_INVALID_LEFT_POINT_COUNT && blackR<CMOS_INVALID_RIGHT_POINT_COUNT){
           CMOS_Data[CMOS_Data_Index][0] = (blackR-TRACK_WIDTH)>0?(blackR-TRACK_WIDTH):0;
           CMOS_Data[CMOS_Data_Index][1] = (blackR+CMOS_Data[CMOS_Data_Index][0])/2;
         }
         // right line are out of sight
         else if(blackL>CMOS_INVALID_LEFT_POINT_COUNT && blackR>CMOS_INVALID_RIGHT_POINT_COUNT){
           CMOS_Data[CMOS_Data_Index][2] = (blackL+TRACK_WIDTH)<CMOS_POINT_PER_LINE?(blackL+TRACK_WIDTH):CMOS_POINT_PER_LINE;
           CMOS_Data[CMOS_Data_Index][1] = (blackR+CMOS_Data[CMOS_Data_Index][0])/2;
         }
         // both lines are out of sight
         else{
           CMOS_Data[CMOS_Data_Index][1] = CMOS_POINT_PER_LINE/2;
         }
         
         CMOS_Last_MidLine = (CMOS_Data[CMOS_Data_Index][0]+CMOS_Data[CMOS_Data_Index][2])/2;
         CMOS_Data_Index++;
         
       }
      
    }
    CMOS_Sample_Flag = 1;  // enable cmp0 to sample
    // only used to debug
   /* CMOS_Temp_Index=0;
    CMOS_Data_Index++;
    oled_debug=CMOS_Data[100][2];*/
  
  }
}

void CMOS_Control(){
  
}


