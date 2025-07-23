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

#define FFT_N 4096	//FFT����
extern float32_t FFT_Data[FFT_N<<1];
//extern float32_t Mag[FFT_N];
//extern float32_t Mag_Single[FFT_N>>1];
//extern float32_t Mag_max;
//extern float32_t Mag_secmax;
//extern float32_t Mag_thirmax;
//extern u32 Mag_max_index;
//extern u32 Mag_secmax_index;
//extern u32 Mag_thirmax_index;
float32_t DSP_FFT(float32_t *FFT_Data, float32_t *fft_buffer);	//FFT+Ƶ��
float32_t find_max_value_500(float32_t* data);


// ================= LUT ����ģ�� ====================
#define FREQ_TABLE_SIZE (7)
#define AMP_TABLE_SIZE  (7)

extern const float freq_table_kHz[FREQ_TABLE_SIZE];
extern const float amp_table_V[AMP_TABLE_SIZE];
extern const float meas_table[FREQ_TABLE_SIZE * AMP_TABLE_SIZE];

float estimate_theoretical_amp(float f_query_kHz, float V_meas);
int Amp2Num(float amp_query);

// =====================================


#endif
