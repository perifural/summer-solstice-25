#ifndef __MOTOR_H
#define __MOTOR_H

#include "AllHeader.h"

#define L_PWMA   TIM8->CCR1  //PC6
#define L_PWMB   TIM8->CCR2  //PC7

#define R_PWMA   TIM8->CCR3  //PC8
#define R_PWMB   TIM8->CCR4  //PC9

#define MOTOR_IGNORE_PULSE (1300)//死区  亚博智能平衡小车电机的死区 1450 25Khz   Dead zone      The dead zone of Yabo smart balancing car motor 1450 25Khz

void Balance_PWM_Init(u16 arr,u16 psc);
void Balance_Motor_Init(void);


int PWM_Ignore(int pulse);
int PWM_Limit(int IN,int max,int min);

void Set_Pwm(int motor_left,int motor_right);

#endif
