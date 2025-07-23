/*********************************************************************************
**********************************************************************************
* 文件名称: dma.h                                                         	     	 *
* 文件简述：当前用于ADC和DAC				                       							 					 *
* 创建日期：2024.07.07                                                          	 *
* 说    明：ADC采样1024点执行FFT													   							 				 *
**********************************************************************************
*********************************************************************************/

#ifndef __DMA_H
#define	__DMA_H
#include "common.h"
#include "adc.h"

//////////////////////////////////////////////////////////////////////////////////

extern u8 DMA_FLAG;	//采样完成标志
extern volatile int8_t ADC_Data_Ready_Buffer; //DMA状态标志
void USER_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 trdir,u32 par,u32 mar,u16 ndtr);//配置DMAx_CHx
void USER_DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);	//使能DMA传输

#endif
