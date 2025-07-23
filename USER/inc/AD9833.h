#ifndef __AD9833_H_
#define __AD9833_H_

#include "common.h"
#include "spi.h"

//////////////////////////////////////////////////////////////////////////////////	 

/* 波形标识 */
#define SQU_WAVE    1	// 方波
#define TRI_WAVE 	2	// 三角波
#define SIN_WAVE 	3	// 正弦波

/* 寄存器常量 */
#define F_mclk 25000000.0f	// 9833外接频率25Mhz(修改)
#define M_mclk 268435456	// 2的28次方
#define P_mclk 4096			// 2的12次方

// AD9833 初始化
void AD9833_Init(void);

// AD9833 设置波形
void AD9833_WaveOut(u8 mode, u32 Freq, u16 Phase, uint8_t channel);

#endif
