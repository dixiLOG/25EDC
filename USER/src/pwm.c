#include "pwm.h"
#include "led.h"
#include "uart.h"
/*stm32f4֧�ֵ�PWM����������£�
TIM1_CH1, PA8,    PE9,
    TIM1_CH2, PA9,    PE11
    TIM1_CH3, PA10,    PE13
    TIM1_CH4, PA11,    PE14

    TIM2_CH1, PA15 (����429��439) 407û�д˽�
    TIM2_CH2, PA1,  PB3
    TIM2_CH3, PA2,  PB10
    TIM2_CH4, PA3,  PB11

    TIM3_CH1, PA6,  PB4, PC6
    TIM3_CH2, PA7,  PB5, PC7
    TIM3_CH3, PB0,  PC8
    TIM3_CH4, PB1,  PC9

    TIM4_CH1, PB6,  PD12
    TIM4_CH2, PB7,  PD13
    TIM4_CH3, PB8,  PD14
    TIM4_CH4, PB9,  PD15

    TIM5_CH1, PA0,  PH10
    TIM5_CH2, PA1,  PH11
    TIM5_CH3, PA2,  PH12
    TIM5_CH4, PA3,  PI10

    TIM8_CH1, PC6,  PI5
    TIM8_CH2, PC7,  PI6
    TIM8_CH3, PC8,  PI7
    TIM8_CH4, PC9,  PI2

    TIM9_CH1, PA2,  PE5
    TIM9_CH2, PA3,  PE6

    TIM10_CH1, PB8,  PF6

    TIM11_CH1, PB9,  PF7

    TIM12_CH1, PB14,  PH6
    TIM12_CH2, PB15,  PH9

    TIM13_CH1, PA6,  PF8
    TIM14_CH1, PA7,  PF9

*/

//TIM14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ,Ϊ��Ҫ��Ƶ�ʼ�1
//psc��ʱ��Ԥ��Ƶ����84Mһ��ȡ84
//q��ռ�ձȣ���ʾ1ռ�������ڵİٷֱ�

void TIM3_PWM_Init(u32 arr, u32 psc,float q)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   // TIM3 ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // ʹ�� PORTC ʱ��

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3); // GPIOC6 ����Ϊ��ʱ��3

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           // GPIOC6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // ���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // �ٶ� 100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // ���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);              // ��ʼ�� GPIOC6

    TIM_TimeBaseStructure.TIM_Prescaler = psc;  // ��ʱ����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr;   // �Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); // ��ʼ����ʱ��3

    // ��ʼ�� TIM3 Channel1 PWM ģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ѡ��ʱ��ģʽ:TIM �����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; // �������:TIM ����Ƚϼ��Ե�
    TIM_OCInitStructure.TIM_Pulse = (arr + 1) * (1.0f-q) - 1; // ����ռ�ձ�Ϊ40%

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  // ����ָ���Ĳ�����ʼ������ TIM3 OC1

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  // ʹ�� TIM3 �� CCR1 �ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM3, ENABLE); // ARPE ʹ�� 

    TIM_Cmd(TIM3, ENABLE);  // ʹ�� TIM3
    TIM_CtrlPWMOutputs(TIM3, ENABLE);  // ʹ�� PWM ���
}
 


