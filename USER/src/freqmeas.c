/********************************************************************************
*********************************************************************************
* 文件名称: freqmeas.c															*
* 文件简述：单片机模拟等精度频率计												*
* 创建日期：2024.07.21															*
* 说    明：运行时尽量避免触发其他中断	
只能测试方波，误差为0.1%，频率范围为0-40MHz
*********************************************************************************
********************************************************************************/

#include "freqmeas.h"

// 外部脉冲计数值: 65536 * timesTIM1_UpOverLoad + cntTIM1_ETR
uint32_t cntTIM1_ETR = 0;  			// TIM1_ETR(PA12) 外部计数值
uint32_t timesTIM1_UpOverLoad = 0;	// TIM1 计数器溢出次数

// 标准信号计数值: (arr + 1) * timesTIM7_UpOverLoad + cntTIM7_Std 
uint32_t cntTIM7_Std = 0;			// TIM7 标准信号计数值
uint32_t timesTIM7_UpOverLoad = 0;	// TIM7 定时器(对基准信号计数)溢出次数

// 异常超时标志
uint32_t overtimeFlag = 1;

// 频率值: 标准信号频率 * 外部脉冲计数值 / 标准信号计数值

/****************************************************************************
* 名    称: freqMeasurement(void)
* 功    能：进行一次频率测量
* 入口参数：无
* 返回参数：>0 为测得的频率, <0 表示测量超时
* 说    明：运行时尽量避免触发其他中断
****************************************************************************/
float freqMeasurement(void)
{
	/* 初始化 */
	TIM7_Init(); 
	TIM1_ETR_Init();
	overtimeFlag = 1;
	
	/* 清空 */
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	cntTIM7_Std = cntTIM1_ETR = 0;
	timesTIM1_UpOverLoad  = timesTIM7_UpOverLoad  = 0;
	TIM1->CNT = TIM7->CNT = 0; 	// TIM_SetCounter(TIM1, 0); TIM_SetCounter(TIM7, 0);
	
	/* Step1 等待上升沿 */
	TIM7->CR1 |= 0x01;		// TIM_Cmd(TIM7, ENABLE)
	while(freqInPortRead == 1 && overtimeFlag);	// 等低电平
	while(freqInPortRead == 0 && overtimeFlag);	// 等高电平
	
	/* Step2 开闸门 */
	timesTIM7_UpOverLoad = TIM7->CNT = 0;	// 清计数
	TIM1->CR1 |= 0x01;		// TIM_Cmd(TIM1, ENABLE);
	
	/* Step3 关闸门, 等上升沿 */
	while(timesTIM7_UpOverLoad == 0 && overtimeFlag);
	while(freqInPortRead == 1 && overtimeFlag);		// 等低电平
	while(freqInPortRead == 0 && overtimeFlag);		// 等高电平
	
	/* Step4 关计数器 */
	TIM1->CR1 &= 0xFE;		// TIM_Cmd(TIM1, DISABLE);
	TIM7->CR1 &= 0xFE;		// TIM_Cmd(TIM7, DISABLE);	
	
	/* 超时处理 */ 
	if(overtimeFlag == 0) return -1.0f;

	/* 计算并返回频率 */
	cntTIM7_Std = timesTIM7_UpOverLoad * 50000 + TIM7->CNT;
	cntTIM1_ETR = timesTIM1_UpOverLoad * 65536 + TIM1->CNT;
	return  500000.0f * cntTIM1_ETR / cntTIM7_Std;
}
