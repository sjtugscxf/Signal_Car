#ifndef SPEED_H
#define SPEED_H

extern s16 speed_real_l,speed_real_r;
extern s16 speed_pwm;
extern s16 speed_order;
extern s16 speed_set;

void Speed_Renew();
void Speed_Order();

#endif