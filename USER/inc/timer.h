/*********************************************************************************
**********************************************************************************
* �ļ�����: timer.h                                                         	     *
* �ļ���������ʱ������ADC��DAC								                       							 *
* �������ڣ�2024.05.06                                                          	 *
* ˵    ����																												   						 *
**********************************************************************************
*********************************************************************************/

#ifndef _TIMER_H
#define _TIMER_H
#include "common.h"

//////////////////////////////////////////////////////////////////////////////////
extern u8 pidEN;
void TIM2_Init(u16 auto_data,u16 fractional);
void TIM6_Init(u16 auto_data,u16 fractional);
static void GENERAL_TIM4_NVIC_Config(void);
void TIM4_Init(u16 auto_data, u16 fractional);
void TIM4_IRQHandler(void);
void TIM1_ETR_Init(void);
void TIM7_Init(void);

#endif
