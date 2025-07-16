#ifndef _pwm_h
#define _pwm_h
#include "stm32f4xx.h"
#include "common.h"

void TIM3_PWM_Init(u32 arr,u32 psc, float q);

#endif
