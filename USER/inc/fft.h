/*********************************************************************************
**********************************************************************************
* �ļ�����: fft.h                                                         	     	 *
* �ļ�������DSP								                     		 													 *
* �������ڣ�2024.07.11                                                          	 *
* ˵    ����FFT������������																						   				 *
**********************************************************************************
*********************************************************************************/

#ifndef __FFT_H
#define __FFT_H
#include "common.h"
#include "arm_math.h"		
#include "adc.h"

//////////////////////////////////////////////////////////////////////////////////  

#define FFT_N 4096  //FFT����

// --- �ⲿ�������� (ע�����͵ı仯) ---
extern float32_t FFT_Data[FFT_N << 1];
extern float32_t Mag[FFT_N];
extern float32_t Mag_Single[FFT_N >> 1];

// ��ֵ�������ֲ���
extern float32_t Mag_max;
extern float32_t Mag_secmax;
extern float32_t Mag_thirmax;

// �������������Ѹ��ģ��Դ洢��ȷƵ�ʣ�
// ����Ҳ����ѡ������������Ϊ Freq_max, Freq_secmax �ȣ���Ϊ�˱��ֱ��������䣬���ı����͡�
extern float32_t Mag_max_index;    // <-- ���ʹ� u32 ��Ϊ float32_t
extern float32_t Mag_secmax_index; // <-- ���ʹ� u32 ��Ϊ float32_t
extern float32_t Mag_thirmax_index;// <-- ���ʹ� u32 ��Ϊ float32_t
void DSP_FFT(const uint16_t* p_adc_data);	//FFT+Ƶ��

#endif
