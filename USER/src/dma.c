/*********************************************************************************
**********************************************************************************
* 文件名称: dma.c                                                         	     	 *
* 文件简述：当前用于ADC和DAC				                       							 					 *
* 创建日期：2024.07.07                                                          	 *
**********************************************************************************
*********************************************************************************/

#include "dma.h"

u8 DMA_FLAG;	//1024点采样完成标志

/****************************************************************************
* 名    称: void USER_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 trdir,u32 par,u32 mar,u16 ndtr)
* 功    能：DMAx的各通道配置
* 入口参数：DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
            chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
						trdir:传输方向
					  par:外设地址
					  mar:存储器地址
					  ndtr:数据传输量
* 返回参数：无
* 说    明：这边传输形式是固定的,这点要根据不同的情况来修改
****************************************************************************/ 
void USER_DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 trdir,u32 par,u32 mar,u16 ndtr)
{ 
	/* 使能DMAy_Streamx时钟 */
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 168MHz
	else
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 168MHz

  /* 配置DMA_Stream */
  DMA_DeInit(DMA_Streamx);
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
	DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = chx;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器地址
  DMA_InitStructure.DMA_DIR = trdir;//传输方向
  DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//存储器数据长度
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用扫描模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
	
	/////////////////////////////专为ADC1配置的中断，与其他外设无关//////////////////////////////////

	/* 配置DMA2_Stream0中断 */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
}

/****************************************************************************
* 名    称: void USER_DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
* 功    能：开启一次DMA传输
* 入口参数：DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
            ndtr:数据传输量
* 返回参数：无
* 说    明：
****************************************************************************/
void USER_DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
	DMA_Cmd(DMA_Streamx, DISABLE);	//关闭DMA传输

	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置

	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);	//数据传输量

	DMA_Cmd(DMA_Streamx, ENABLE);	//开启DMA传输
}	  

//DMA2_Stream0中断处理函数
void  DMA2_Stream0_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0)!=RESET){
		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);	//清除DMA中断标志
		DMA_FLAG = 1;	//FFT完成标志
	}
}
