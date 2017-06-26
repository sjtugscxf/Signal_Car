#ifndef MOTOR_H
#define MOTOR_H

// ===== Glabal Variables =====

extern S16 tacho0, tacho1;
extern s16 ServoOut;



// ===== APIs =====

  // Control Servo . 0 is straight forward
void Servo_Output(s16 x);

  // Control Motors . -1000 to 1000
void MotorL_Output(s16);
void MotorR_Output(s16);

  // Enable Motors . 1 is On. ( Automatically On when Initiated )
  // There are some differences between 
  // ( MotorL_Output(0);MotorL_Enable(1); ) and  MotorL_Enable(0);
  // You can try it out.
void MotorL_Enable(u8 x);
void MotorR_Enable(u8 x);

  // Get the postion of Tacho
void Tacho0_Get();
u16 Tacho0_Renew();
void Tacho1_Get();

  // Init
void Motor_Init(void);
void Tacho_Init();
void Servo_Init();


// ===== Basic Drivers ===== 

u8 Tacho0_Dir(void);
u8 Tacho1_Dir(void);

#endif