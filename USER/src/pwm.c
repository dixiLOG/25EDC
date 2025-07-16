#include "pwm.h"
#include "led.h"
#include "uart.h"
/*stm32f4支持的PWM输出引脚如下：
TIM1_CH1, PA8,    PE9,
    TIM1_CH2, PA9,    PE11
    TIM1_CH3, PA10,    PE13
    TIM1_CH4, PA11,    PE14

    TIM2_CH1, PA15 (仅限429，439) 407没有此脚
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

//TIM14 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值,为需要的频率减1
//psc：时钟预分频数，84M一般取84
//q：占空比，表示1占整个周期的百分比

void TIM3_PWM_Init(u32 arr, u32 psc,float q)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);   // TIM3 时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // 使能 PORTC 时钟

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3); // GPIOC6 复用为定时器3

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           // GPIOC6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 速度 100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);              // 初始化 GPIOC6

    TIM_TimeBaseStructure.TIM_Prescaler = psc;  // 定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseStructure.TIM_Period = arr;   // 自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); // 初始化定时器3

    // 初始化 TIM3 Channel1 PWM 模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 选择定时器模式:TIM 脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; // 输出极性:TIM 输出比较极性低
    TIM_OCInitStructure.TIM_Pulse = (arr + 1) * (1.0f-q) - 1; // 设置占空比为40%

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  // 根据指定的参数初始化外设 TIM3 OC1

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  // 使能 TIM3 在 CCR1 上的预装载寄存器

    TIM_ARRPreloadConfig(TIM3, ENABLE); // ARPE 使能 

    TIM_Cmd(TIM3, ENABLE);  // 使能 TIM3
    TIM_CtrlPWMOutputs(TIM3, ENABLE);  // 使能 PWM 输出
}
 


