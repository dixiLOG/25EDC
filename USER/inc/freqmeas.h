#ifndef _FREQMEAS_H_
#define _FREQMEAS_H_

#include "common.h"
#include "timer.h"

//////////////////////////////////////////////////////////////////////////////////	 

// 快速读取 PA12 端口的电平
#define freqInPortRead  PAin(12)

// 外部脉冲计数值: 65536 * timesTIM1_UpOverLoad + cntTIM1_ETR
extern uint32_t cntTIM1_ETR;  			// TIM1_ETR(PA12) 外部计数值
extern uint32_t timesTIM1_UpOverLoad;	// TIM1 计数器溢出次数

// 标准信号计数值: (arr + 1) * timesTIM7_UpOverLoad + cntTIM7_Std 
extern uint32_t cntTIM7_Std;			// TIM7 标准信号计数值
extern uint32_t timesTIM7_UpOverLoad;	// TIM7 定时器(对基准信号计数)溢出次数

// 异常超时标志
extern uint32_t overtimeFlag;

/* 进行一次频率测量, 运行时尽量避免触发其他中断
 * 输入参数: 无
 * 返回值: >0 为测得的频率, <0 表示测量超时
 */
float freqMeasurement(void);

#endif
