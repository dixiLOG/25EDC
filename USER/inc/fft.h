/*********************************************************************************
**********************************************************************************
* 文件名称: fft.h                                                         	     	 *
* 文件简述：DSP								                     		 													 *
* 创建日期：2024.07.11                                                          	 *
* 说    明：FFT函数，幅度谱																						   				 *
**********************************************************************************
*********************************************************************************/

#ifndef __FFT_H
#define __FFT_H
#include "common.h"
#include "arm_math.h"		
#include "adc.h"

//////////////////////////////////////////////////////////////////////////////////  

#define FFT_N 4096  //FFT点数

// --- 外部变量声明 (注意类型的变化) ---
extern float32_t FFT_Data[FFT_N << 1];
extern float32_t Mag[FFT_N];
extern float32_t Mag_Single[FFT_N >> 1];

// 幅值变量保持不变
extern float32_t Mag_max;
extern float32_t Mag_secmax;
extern float32_t Mag_thirmax;

// 索引变量类型已更改，以存储精确频率！
// 我们也可以选择将它们重命名为 Freq_max, Freq_secmax 等，但为了保持变量名不变，仅改变类型。
extern float32_t Mag_max_index;    // <-- 类型从 u32 变为 float32_t
extern float32_t Mag_secmax_index; // <-- 类型从 u32 变为 float32_t
extern float32_t Mag_thirmax_index;// <-- 类型从 u32 变为 float32_t
void DSP_FFT(const uint16_t* p_adc_data);	//FFT+频谱

#endif
