#ifndef __BSP_ULTRASONIC_H_
#define __BSP_ULTRASONIC_H_

#include "AllHeader.h"

#define ULTRASONIC_RCC   RCC_APB2Periph_GPIOA

#define TRIG_PORT  GPIOA
#define TRIG_PIN   GPIO_Pin_0

#define ECHO_PORT  GPIOA
#define ECHO_PIN   GPIO_Pin_1

#define TRIG_SIG 	PAout(0)

void ultrasonic_init(void);
void TIM2_Cap_Init(u16 arr,u16 psc);	
int get_distance(void);

#endif

