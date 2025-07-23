#ifndef __AD9833_H_
#define __AD9833_H_

#include "common.h"
#include "spi.h"

//////////////////////////////////////////////////////////////////////////////////	 

/* ���α�ʶ */
#define SQU_WAVE    1	// ����
#define TRI_WAVE 	2	// ���ǲ�
#define SIN_WAVE 	3	// ���Ҳ�

/* �Ĵ������� */
#define F_mclk 25000000.0f	// 9833���Ƶ��25Mhz(�޸�)
#define M_mclk 268435456	// 2��28�η�
#define P_mclk 4096			// 2��12�η�

// AD9833 ��ʼ��
void AD9833_Init(void);

// AD9833 ���ò���
void AD9833_WaveOut(u8 mode, u32 Freq, u16 Phase, uint8_t channel);

#endif
