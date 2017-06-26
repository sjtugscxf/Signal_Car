#ifndef CAM_H
#define CAM_H

// ===== Settings =====
// the camera has about 300 rows and 400 cols ,
// but we can not handle that much ,
// so we get 1 row of image for every IMG_STEP rows of camera ,
// and totally get IMG_ROWS rows.

#define IMG_ROWS 64
#define IMG_COLS 128
#define IMG_STEP 2      // TODO: 远多近少

#define CAM_STEP 2 //cam数据处理间距
#define CAM_FAR 10//最远对应cam行数
#define CAM_NEAR 60//最近对应cam行数

#define BLACK_HEIGHT 5
#define BLACK_WIDTH  27

#define PI 3.141593

// ====== Global Variables ======

extern u8 cam_buffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];


// ===== APIs ======

  // write your algorithm in this func
void Cam_Algorithm();

  // Init
void Cam_Init();

//==============CAM_B===========
#define CAM_WID 132//摄像头有效宽度//与摄像头安放位置有关//120//132
#define thr 70//黑白阈值，目前无需调
#define ROAD_WID 30//道路宽度，未知，需要在透视变换后使用、、、、、、、、、、
#define Dir_Kp 4    //舵机比例控制参数
#define Dir_Kd 3  //舵机微分控制参数
#define MAX_SPEED 20 //直道最大速度/////////////////////////26为现在的极限
#define MIN_SPEED 12 //弯道最大速度////////////////////////不确定
#define ROAD_SIZE 25 //利用的摄像头数据行数
#define WEIGHT_SIZE 10 //实际加权并控制舵机的行数
#define MaxWeight_index 7 //最大weight的下标，范围是0-9
#define MaxWeight 6.0 //最大weight权值
#define exp_k 0.3 //给正态分布的指数项乘一个因子，使曲线扁平
/*#define CAM_HOLE_ROW 15 //用来向两边扫描检测黑洞·环岛的cam_buffer行位置
#define ROAD_OBST_ROW 10 //用来检测障碍物的road_B行位置//不能太远，也不能太近
#define OBSTACLE_THR 10  //有障碍物时赛道宽度阈值*/
extern int CAM_HOLE_ROW; //用来向两边扫描检测黑洞·环岛的cam_buffer行位置
extern int ROAD_OBST_ROW; //用来检测障碍物的road_B行位置//不能太远，也不能太近
extern int OBSTACLE_THR;  //有障碍物时赛道宽度阈值
typedef struct {
  int left;
  int right;
  int mid;
 // double slope_;//slpoe_=dx/dy
}Road;//由近及远存放
typedef struct {
  float r;
  bool sign;
}circle;
extern Road road_B[ROAD_SIZE];
extern float mid_ave;//road中点加权后的值
extern float weight[4][10];//road中点权值
extern int valid_row;//有效行位置，大小为road_B[]下标
extern int valid_row_thr;//有效行阈值
extern u8 car_state;//智能车状态
extern u8 remote_state;//远程控制
extern u8 road_state;//前方道路状态
extern float motor_L;
extern float motor_R;
//OLED调参
extern int debug_speed;
extern PIDInfo debug_dir;
extern int margin;
extern circle C;
extern int c1, c2, c3;

void Cam_B_Init();//初始化Cam_B
float constrain(float lowerBoundary, float upperBoundary, float input);//控制上下限的函数
int constrainInt(int lowerBoundary, int upperBoundary, int input);//控制上下限的函数(integer专用)
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);//得到前方曲率圆
bool is_stop_line(int target_line);//判断是否为终止行/起点行
double getSlope_(int x1, int y1, int x2, int y2);//得到斜率的倒数
//——————透视变换
/*
extern u8 cam_buffer2[64][128];
void getMatrix(double fov, double aspect, double zn, double zf);
void linearization();
void multiply(int k);
void matrixMultiply();
void getNewBuffer();*/
//——————

//test
extern double theta;
extern double x,y;

#endif