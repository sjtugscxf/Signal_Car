#include "Flash.h"
#include "typedef.h"

U16 data[DATA_NUM];

U16 data_initial[DATA_NUM]={
  1,	        //data flag
  0,	        //KF_switch
  0,	        //blance_deform
  40,	        //balance_K
  100,	        //balance P
  85,	        //balance D
  95,	        //balance dt
  1635,	        //acc offset
  40,            //P speed
  10,            //D speed
  75,           //pwm deadzone left
  70,           //pwm deadzone right
  1,            //wheel p
  1,            //wheel i
  0,            //wheel d
  15,           // I speed
};


//----------------------
void Flash_Write(U16 sector){
  U32 addr = ADDR + sector*SECTOR_SIZE;
  NVIC_DisableIRQ(PIT0_IRQn);
  NVIC_DisableIRQ(PORTC_IRQn);
  NVIC_DisableIRQ(UART3_RX_TX_IRQn);
  Flash_Erase(sector);
  Flash_Program(sector,DATA_NUM,data);
  NVIC_EnableIRQ(PIT0_IRQn);
  NVIC_EnableIRQ(PORTC_IRQn);
  NVIC_EnableIRQ(UART3_RX_TX_IRQn);
}

void Flash_Data_Update(void){
  U8 i;
  for(i=0;i<DATA_NUM;i++){
    data[i] = Flash_Read(0,i);
  }
}

void Flash_Data_Reset(void){
  U16 i;
  for(i=0;i<DATA_NUM;i++){
    data[i] = data_initial[i];
  }
  Flash_Write(0);
}
//-----------------------

void Flash_Init(void){
  FMC->PFB0CR|=FMC_PFB0CR_S_B_INV_MASK;
  FMC->PFB1CR|=FMC_PFB0CR_S_B_INV_MASK;
  while(!(FTFL->FSTAT&FTFL_FSTAT_CCIF_MASK));
  FTFL->FSTAT=FTFL_FSTAT_ACCERR_MASK|FTFL_FSTAT_FPVIOL_MASK;
  
  if(Flash_Read(0,0)!=1) Flash_Data_Reset();
  else Flash_Data_Update();
}

U8 Flash_Erase(U16 num){
  union{
    U32 Word;
    U8  Byte[4];
  }FlashDestination;
  FlashDestination.Word=ADDR+num*SECTOR_SIZE;
  FTFL->FCCOB0=FTFL_FCCOB0_CCOBn(ERSSCR);
  FTFL->FCCOB1=FlashDestination.Byte[2];
  FTFL->FCCOB2=FlashDestination.Byte[1];
  FTFL->FCCOB3=FlashDestination.Byte[0];
  if(FlashCMD()==1){return 1;}//Error
  return 0;//Success
}

U8 Flash_Program(U16 num, U16 WriteCounter, U16 *DataSource){
  U32 size;
  U32 destaddr;
  union{
    U32 Word;
    U8 Byte[4];
  }FlashDestination; 
  FTFL->FCCOB0=PGM4;
  destaddr=(ADDR+num*SECTOR_SIZE);
  FlashDestination.Word=destaddr;
  for(size=0;size<WriteCounter;size+=2,FlashDestination.Word+=4,DataSource+=2){
    FTFL->FCCOB1=FlashDestination.Byte[2];
    FTFL->FCCOB2=FlashDestination.Byte[1];
    FTFL->FCCOB3=FlashDestination.Byte[0];
    FTFL->FCCOB4=DataSource[1]>>8;
    FTFL->FCCOB5=DataSource[1]&0xFF;
    FTFL->FCCOB6=DataSource[0]>>8;
    FTFL->FCCOB7=DataSource[0]&0xFF;    
    if(FlashCMD()==1)return 2;
  }
  return 0;
}


U16 Flash_Read(U16 num,U16 data_num){
  U16* addr = (U16*)(ADDR + num*SECTOR_SIZE + data_num * 0x02);
  return *addr;
}

static U8 FlashCMD(void){
  FTFL->FSTAT=FTFL_FSTAT_ACCERR_MASK|FTFL_FSTAT_FPVIOL_MASK;
  FTFL->FSTAT=FTFL_FSTAT_CCIF_MASK;
  while(!(FTFL->FSTAT&FTFL_FSTAT_CCIF_MASK));
  if(FTFL->FSTAT&(FTFL_FSTAT_ACCERR_MASK|FTFL_FSTAT_FPVIOL_MASK|FTFL_FSTAT_MGSTAT0_MASK)){return 1;}//Failed
  return 0;//Success
}
