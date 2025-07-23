/*********************************************************************************
**********************************************************************************
* 文件名称: timer.c                                                         	     *
* 文件简述：定时器触发ADC和DAC								                       							 *
* 创建日期：2024.05.06                                                          	 *
* 说    明：																												   						 *
**********************************************************************************
*********************************************************************************/

#include "timer.h"
#include "pid.h"
#include "lcd.h"
#include "freqmeas.h"
/****************************************************************************
* 名    称: TIM2_Init(u16 auto_data,u16 fractional)
* 功    能：定时器2初始化
* 入口参数：auto_data: 自动重装值
*           fractional: 时钟预分频数
* 返回参数：无
* 说    明：定时器溢出时间计算方法:Tout(us)=auto_data*fractional/Ft		Ft为定时器时钟(MHz)
****************************************************************************/

float kp= 0.00001;
float ki= 0.00002;
float kd=-0.00009;
//
//

void TIM2_Init(u16 auto_data,u16 fractional)
{
	/* 使能时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	//使能TIM2时钟 84MHz

	/* TIM2初始化设置 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_TimeBaseInitStructure.TIM_Period = auto_data-1;	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=fractional-1;	//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV4;	//无需修改
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);	//初始化TIM2

	/* PWM初始化设置 */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState =	TIM_OutputState_Enable;	//比较输出使能
	TIM_OCInitStructure.TIM_Pulse = auto_data/2;	//TIM_Pulse/(TIM_Period+1) 为PWM的占空比
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	//输出极性:TIM输出比较极性低
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);	//初始化外设TIM2_CH2
	
	/* 使能 */
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);	//使能TIM2预装载寄存器
	TIM_ARRPreloadConfig(TIM2,ENABLE);	//使能ARPE
	TIM_CtrlPWMOutputs(TIM2, ENABLE);	// 使能PWM
	TIM_Cmd(TIM2,ENABLE);	//使能定时器2
}

/****************************************************************************
* 名    称: TIM6_Init(u16 auto_data,u16 fractional)
* 功    能：定时器6初始化
* 入口参数：auto_data: 自动重装值
*           fractional: 时钟预分频数
* 返回参数：无
* 说    明：定时器溢出时间计算方法:Tout(us)=auto_data*fractional/Ft		Ft为定时器时钟(MHz)
****************************************************************************/
void TIM6_Init(u16 auto_data,u16 fractional)
{
	/* 使能时钟 */
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//使能TIM6时钟 84MHz
	
	/* TIM6初始化设置 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_TimeBaseInitStructure.TIM_Period = auto_data-1;	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=fractional-1;	//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV4;	//无需修改
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);//设置TIME输出触发为更新模式

	/* 使能 */
//	TIM_Cmd(TIM6, ENABLE);
}


void TIM4_Init(u16 period, u16 prescaler)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);		//使能TIM4时钟
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				//TIM4中断
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//先占优先级0级
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);								//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	TIM_TimeBaseStructure.TIM_Period = period-1;				//总的值设置为0xFFFF，设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler-1;			//预分频器
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;				//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);				//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM_Cmd(TIM4, ENABLE);										//开启定时器
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);					//开启定时器更新中断
}

void kp_up(){
	kp+=0.00001f;
}
void kp_down(){
	kp-=0.00001f;
}
void kd_up(){
	kd+=0.0000005f;
}
void kd_down(){
	kd-=0.0000005f;
}

void TIM4_IRQHandler(void)
{
	//static u16 tmr200ms = 0;
//	static u16 tmr1s =0;
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) 			//溢出中断
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);				//清除中断标志位
		//输入你的中断代码//
	}
}


///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// 频率计定时器配置 ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/****************************************************************************
* 名    称: void TIM7_Init(void)
* 功    能：定时器7 初始化，作为频率计的基准时钟
* 入口参数：无
* 返回参数：无
* 说    明：请不要随意更改
****************************************************************************/
void TIM7_Init(void)	
{
	/* TIM7 Init (Base TIM) --> GATE */
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStrcture;
	
	/* Freq of APB1: 168/4 = 42MHz; TIM7 Freq = 2*42 = 84MHz */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);	
	
	/* 	[Gate] Period of TIM7  Reload = (arr + 1)(psc + 1) / 84M  
	 *	[f standard] After prescaler = 84M / (psc + 1)   
	 *	Error smaller -> Gate*fs larger, Gate*fs = arr + 1
	 *	Let Gate Time = 0.1s
	 *	arr = 50000 - 1
	 *	psc = 168 - 1
	 */
	
	TIM_InitStructure.TIM_Period = 50000 - 1;				// 计数周期 -- 50000
	TIM_InitStructure.TIM_Prescaler = 168 - 1;				// 计数频率 -- 84e6 / 168 = 500 kHz
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 不分频
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// 向上计数
	TIM_InitStructure.TIM_RepetitionCounter = DISABLE;		// 不重复
	TIM_TimeBaseInit(TIM7, &TIM_InitStructure);

	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);	// 清中断
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);	// 开中断

	NVIC_InitStrcture.NVIC_IRQChannel = TIM7_IRQn;				// 中断
	NVIC_InitStrcture.NVIC_IRQChannelPreemptionPriority = 0;	// 抢占优先级 0
	NVIC_InitStrcture.NVIC_IRQChannelSubPriority = 0;			// 响应优先级 0
	NVIC_InitStrcture.NVIC_IRQChannelCmd = ENABLE;				// 开中断
	NVIC_Init(&NVIC_InitStrcture);
	
	TIM_Cmd(TIM7, DISABLE);	// 停止计数
}

/****************************************************************************
* 名    称: void TIM1_ETR_Init(void)
* 功    能：开启 定时器1 的 ETR 功能，实现引脚 PA12 对外部脉冲计数
* 入口参数：无
* 返回参数：无
* 说    明：请不要随意更改
****************************************************************************/
void TIM1_ETR_Init(void)
{
	/* TIM1 ETR Init (Advanced TIM)  ETR Count -- > PA12 */
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStrcture;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);		// TIM1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// GPIOA

	// GPIOA (PA12) 初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;			// PA12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  		// 复用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  		// 上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	// 100 MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_TIM1);

	// TIM1 初始化
	TIM_InitStructure.TIM_Period = 0xFFFF;  				// 计数周期
	TIM_InitStructure.TIM_Prescaler = 0;   					// 分配系数
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 不分频
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// 向上计数
	TIM_TimeBaseInit(TIM1, &TIM_InitStructure);
	TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);  // 外部时钟源模式
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);	// 清中断
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);	// 开中断
	
	// 中断配置
	NVIC_InitStrcture.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;		// 中断
	NVIC_InitStrcture.NVIC_IRQChannelPreemptionPriority = 0;	// 抢占优先级 0 
	NVIC_InitStrcture.NVIC_IRQChannelSubPriority = 1;		  	// 响应优先级 1
	NVIC_InitStrcture.NVIC_IRQChannelCmd = ENABLE;			  	// 开中断
	NVIC_Init(&NVIC_InitStrcture);

	TIM_Cmd(TIM1, DISABLE);	// 停止计数
}

/****************************************************************************
* 名    称: void TIM7_IRQHandler(void)
* 功    能：定时器7 中断服务程序，用于指示频率计闸门和异常超时
* 入口参数：无
* 返回参数：无
* 说    明：请不要随意更改
****************************************************************************/
void TIM7_IRQHandler(void)
{
	if(TIM7->SR & (1<<0))	  // TIM_GetITStatus(TIM7,TIM_IT_Update) == SET
	{
		timesTIM7_UpOverLoad ++;							// 基准信号计数溢出次数
		if(timesTIM7_UpOverLoad == 11) overtimeFlag = 0;	// 1.1s 外部脉冲还没有来, 判定为超时异常
		TIM7->SR &= ~(1<<0);  								// TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

/****************************************************************************
* 名    称: void TIM1_UP_TIM10_IRQHandler(void)
* 功    能：定时器1 中断服务程序，用于指示外部脉冲计数溢出
* 入口参数：无
* 返回参数：无
* 说    明：请不要随意更改
****************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM1->SR & (1<<0))
	{
		timesTIM1_UpOverLoad++;		// 外部脉冲计数溢出次数
		TIM1->SR &= ~(1<<0);  		// 清中断
	}
}