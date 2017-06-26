#ifndef FTM_H
#define FTM_H

extern S16 tachol,tachor;

void FTM_Init(void);

void PWM_L(s16);
void PWM_R(s16);
S16 Tacho_L(void);
S16 Tacho_R(void);

#endif