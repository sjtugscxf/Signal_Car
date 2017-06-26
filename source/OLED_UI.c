
#include "includes.h"

// ====== Variables ======
// ---- Global ----
u8 oled_menu=0;
// ---- Local ----
bool flag_down=0;



  /*
  UI_Page homepage;
  UI_Page subpage0;
  homepage.sub_type = (enum Item_Type *) malloc (4*2);
  homepage.sub_type[0] = Item_Type_Menu;
  homepage.sub = (void **)malloc (4*2); // (void **)(UI_Page **)
  *((UI_Page **)(homepage.sub)+0) = (UI_Page *) &subpage0;
  subpage0.parent = (void *) &homepage;
  
  subpage0.sub = (void **)123;
  Oled_Putnum(0,0,(s16)((*((UI_Page **)homepage.sub+0))->sub));
  */
  //free(homepage.sub);
  //free(homepage.sub_type);


enum Item_Type{
    Item_Type_Menu,
    Item_Type_Para,
    Item_Type_Show,
    Item_Type_Func,
};

typedef struct {
  void * parent;   // UI_Page *
  enum Item_Type * sub_type; 
  void ** sub;  // UI_Page **
}UI_Page;

typedef struct {
  u8 * category;
  s16  data;
}Putboth;


void UI_SystemInfo(){
  
  Oled_Clear();//删掉什么后果？
  
  switch(oled_menu)
  {
  case 0:displayMenu();break;
  case 1:displayParameters();break;
  case 2:displayCamera();break;
  case 3:displayDebug();break;
  default:Oled_Clear();
  }
  
  
 // if(SW1()){
  //Oled_Putstr(0,0,"Car Type"); Oled_Putnum(0,11,CAR_TYPE);
  //Oled_Putstr(1,0,"battery"); Oled_Putnum(1,11,battery);
  //Oled_Putstr(2,0,"mid[6]"); Oled_Putnum(2,11,road_B[6].mid);
  //Oled_Putstr(3,0,"mid[7]"); Oled_Putnum(3,11,road_B[7].mid);
  //Oled_Putstr(4,0,"mid[8]"); Oled_Putnum(4,11,road_B[8].mid);
  //Oled_Putstr(5,0,"mid[9]"); Oled_Putnum(5,11,road_B[9].mid);
  //Oled_Putstr(6,0,"servo"); Oled_Putnum(6,11,ServoOut);
  //Oled_Putstr(7,0,"safe_point"); Oled_Putnum(7,11,safe_point);
  //Oled_Putstr(6,0,"40"); Oled_Putnum(6,11,cam_buffer[40][10]);
  //Oled_Putstr(3,0,"pit0 time"); Oled_Putnum(3,11,(s16)pit0_time);
 // Oled_Putstr(4,0,"pit1 time"); Oled_Putnum(4,11,(s16)pit1_time);
  //Oled_Putstr(5,0,"speeds"); Oled_Putnum(5,11,speed_set);
  //Oled_Putstr(6,0,"tacho0"); Oled_Putnum(6,11,tacho0);
  //Oled_Putstr(7,0,"tacho1"); Oled_Putnum(7,11,tacho1);
  //Oled_Putstr(5,0,"10"); Oled_Putnum(5,11,cam_buffer[10][10]);
  //Oled_Putstr(6,0,"40"); Oled_Putnum(6,11,cam_buffer[40][10]);
  //Oled_Putstr(7,0,"70"); Oled_Putnum(7,11,cam_buffer[70][10]);
}

void displayMenu()//menu==0
{
  Oled_Putstr(0,1,"<Start-Menu>");
  Oled_Putstr(1,1,"car type"); Oled_Putnum(1,11,CAR_TYPE); 
 // Oled_Putstr(1,1,"Press Key1: Display Parameters");
 // Oled_Putstr(2,1,"Press Key2: Display Camera");
 // Oled_Putstr(3,1,"Press Key3: Debug Parameters");
  Oled_Putstr(6,1,"battery"); Oled_Putnum(6,11,battery);
  Oled_Putstr(7,1,"car_state"); Oled_Putnum(7,11,car_state);
}

void displayParameters()//menu==1
{
   //按键下拉上拉功能
  //最大行数需要在头文件中修改
  static int start=0;
  static int page=0;
  Putboth outpair[Pages][Rows]={
    {
      {"road_state",road_state},
      {"valid_row",valid_row}, 
      {"battery",battery},      {"pit0 time",pit0_time},  {"pit1 time",pit1_time},  
      {"servo",ServoOut},       {"mid_ave",mid_ave},       
      {"car_state",car_state},
      {"tacho0",tacho0},{"tacho1",tacho1},
      {"motor_L",motor_L},{"motor_R",motor_R}

      //{"C.r",C.r},{"C.sign",C.sign}
    },
    { 
      {"mid_ave_x",mid_ave},
      {"mid[0]",road_B[0].mid},     {"mid[1]",road_B[1].mid},         {"mid[2]",road_B[2].mid}, 
      {"mid[3]",road_B[3].mid},     {"mid[4]",road_B[4].mid},         {"mid[5]",road_B[5].mid}, 
      {"mid[6]",road_B[6].mid},     {"mid[7]",road_B[7].mid},         {"mid[8]",road_B[8].mid},
      {"mid[9]",road_B[9].mid},     {"mid[10]",road_B[10].mid},       {"mid[11]",road_B[11].mid},
      {"mid[12]",road_B[12].mid},   {"mid[13]",road_B[13].mid},       {"mid[14]",road_B[14].mid},
      {"mid[15]",road_B[15].mid},   {"mid[16]",road_B[16].mid},       {"mid[17]",road_B[17].mid},
      {"mid[18]",road_B[18].mid},   {"mid[19]",road_B[19].mid},       {"mid[20]",road_B[20].mid},
      {"mid[21]",road_B[21].mid},   {"mid[22]",road_B[22].mid},       {"mid[23]",road_B[23].mid}, 
      {"mid[24]",road_B[24].mid}
    },
    {
          
      {"weight[road_state][0]",weight[road_state][0]*10},      {"weight[road_state][1]",weight[road_state][1]*10},        {"weight[road_state][2]",weight[road_state][2]*10},
      {"weight[road_state][3]",weight[road_state][3]*10},      {"weight[road_state][4]",weight[road_state][4]*10},        {"weight[road_state][5]",weight[road_state][5]*10},
      {"weight[road_state][6]",weight[road_state][6]*10},      {"weight[road_state][7]",weight[road_state][6]*10},        {"weight[road_state][8]",weight[road_state][8]*10},
      {"weight[road_state][9]",weight[road_state][9]*10},          {"",0}
    }
   /* {
      {"cam_buf[60]",cam_buffer[60][60]},
      {"cam_buf[55]",cam_buffer[55][60]},
      {"cam_buf[50]",cam_buffer[50][60]},  
      {"cam_buf[45]",cam_buffer[45][60]},
      {"cam_buf[40]",cam_buffer[40][60]},
      {"cam_buf[35]",cam_buffer[35][60]},  
      {"cam_buf[30]",cam_buffer[30][60]},
      {"cam_buf[25]",cam_buffer[25][60]},
      {"cam_buf[20]",cam_buffer[20][60]},
      {"cam_buf[15]",cam_buffer[15][60]},
      {"cam_buf[10]",cam_buffer[10][60]}
    }*/
  };
  
  for (int i=1;i<=7;i++)                           
  { 
    Oled_Putstr(i,0,outpair[page][start+i-1].category); Oled_Putnum(i,11,outpair[page][start+i-1].data);
  }
  
  if(Key2() && Key3() && Key1())  flag_down=0;//检测是否按下key2 key3
  //改变OLED显示内容
  //上下拉
  if (!Key3() && Key1() && start<Rows-7 ){ start++; flag_down=1; }//下拉
  if (!Key2() && Key1() && start>0 ){ start--; flag_down=1; }//上拉
  //下一页
  if(!Key1() && flag_down==0)
  {
    page++;
    page%=Pages;
    start=0;
    flag_down=1;
  }
}

void displayCamera()//menu==2
{
//  if(!Key1())
    drawCam(isWhite);
 // else
  //  drawCam2(isWhite);
}


void displayDebug()//menu==3
{
  static int para_state=0;
  if(Key1() && Key2() && Key3()) flag_down=0;
  else if(!Key1() && flag_down==0)
  {
    para_state++;
    para_state%=4;
    flag_down=1;
  }
  switch(para_state)
  {
  /*  case 0:
    Oled_Putstr(6,0,"Debugging CAM_HOLE_ROW"); Oled_Putnum(7,11,CAM_HOLE_ROW);
    if(!Key2() && flag_down==0) {CAM_HOLE_ROW+=1;flag_down=1;}
    if(!Key3() && flag_down==0) {CAM_HOLE_ROW-=1;flag_down=1;}
    break;
  case 1:
    Oled_Putstr(6,0,"Debugging ROAD_OBST_ROW"); Oled_Putnum(7,11,ROAD_OBST_ROW);
    if(!Key2() && flag_down==0) {ROAD_OBST_ROW+=1;flag_down=1;}
    if(!Key3() && flag_down==0) {ROAD_OBST_ROW-=1;flag_down=1;}
    break;
  case 2:
    Oled_Putstr(6,0,"Debugging OBSTACLE_THR"); Oled_Putnum(7,11,OBSTACLE_THR);
    if(!Key2() && flag_down==0) {OBSTACLE_THR+=1;flag_down=1;}
    if(!Key3() && flag_down==0) {OBSTACLE_THR-=1;flag_down=1;}
    break;*/
  case 0:
    Oled_Putstr(6,0,"Debugging dir.kp"); Oled_Putnum(7,11,debug_dir.kp*10);
    if(!Key2() && flag_down==0) {debug_dir.kp+=0.1;flag_down=1;}
    if(!Key3() && flag_down==0) {debug_dir.kp-=0.1;flag_down=1;}
    break;
  case 1:
    Oled_Putstr(6,0,"Debugging dir.kd"); Oled_Putnum(7,11,debug_dir.kd*10);
    if(!Key2() && flag_down==0) {debug_dir.kd+=0.1;flag_down=1;}
    if(!Key3() && flag_down==0) {debug_dir.kd-=0.1;flag_down=1;}
    break;
  case 2:
    Oled_Putstr(6,0,"Debugging MaxSpeed"); Oled_Putnum(7,11,debug_speed);
    if(!Key2() && flag_down==0) {debug_speed+=1;flag_down=1;}
    if(!Key3() && flag_down==0 && MAX_SPEED+debug_speed>MIN_SPEED+2) {debug_speed-=1;flag_down=1;}
    break;
  case 3:
    Oled_Putstr(6,0,"Debugging Margin"); Oled_Putnum(7,11,margin);
    if(!Key2() && flag_down==0) {margin+=1;flag_down=1;}
    if(!Key3() && flag_down==0 && margin>0) {margin-=1;flag_down=1;}
    break;
/*  case 4:
    Oled_Putstr(6,0,"Debugging c1"); Oled_Putnum(7,11,c1);
    if(!Key2() && flag_down==0) {c1+=1;flag_down=1;}
    if(!Key3() && flag_down==0) {c1-=1;flag_down=1;}
    break;
  case 5:
    Oled_Putstr(6,0,"Debugging c2"); Oled_Putnum(7,11,c2);
    if(!Key2() && flag_down==0) {c2+=1;flag_down=1;}
    if(!Key3() && flag_down==0) {c2-=1;flag_down=1;}
    break;
  case 6:
    Oled_Putstr(6,0,"Debugging c3"); Oled_Putnum(7,11,c3);
    if(!Key2() && flag_down==0) {c3+=1;flag_down=1;}
    if(!Key3() && flag_down==0) {c3-=1;flag_down=1;}
    break;*/
  default:break;
  }
  
  Oled_Putstr(0,0,"dir.kp*10"); Oled_Putnum(0,11,(Dir_Kp+debug_dir.kp)*10);
  Oled_Putstr(1,0,"dir.kd*10"); Oled_Putnum(1,11,(Dir_Kd+debug_dir.kd)*10);
  Oled_Putstr(2,0,"MaxSpeed"); Oled_Putnum(2,11,MAX_SPEED+debug_speed);
    
}

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
void drawCam2(bool(*isTarget)(u8 x)) {
  int row, col, i;
  u8 buf[1024];
  u8 *p = buf;
    
  for (row = IMG_ROWS-1; row >= 0; row -= 8) {
    for (col = IMG_COLS-1; col >= 0 ; col--) {
      u8 tmp = 0;
      for (i = 0; i < 8; i++) {
        tmp <<= 1;
        if (isTarget(cam_buffer2[row-i][col]))
          tmp |= 0x01;
      }
      *p++ = tmp;
    }
  }
  Oled_DrawBMP(0, 0, 128, 64, buf);
}
*/
bool isWhite(u8 x) {     //白色阈值，场地理想后好像没什么用
  return x > 70;
}

void UI_Graph(u8* data){
  
}
















