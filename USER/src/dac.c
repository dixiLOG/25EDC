/*********************************************************************************
**********************************************************************************
* 文件名称: dac.c                                                         	    	 *
* 文件简述：DAC1定时器6触发，输出可更改，DMA搬运								             	     *
* 创建日期：2024.07.07                                                          	 *
* 说    明：DAC_Data_Tx为输出固定值，DAC1_Data_Tx为连续输出波形			 							 *
**********************************************************************************
*********************************************************************************/

#include "dac.h"

u16 DAC_Data_Tx = 2/3.3*4095;	//DAC输出固定值，可随时在主函数while(1)中更改，并需要在初始化中取消注释，和注释连续波

//DAC输出连续波形
u16 DAC1_Data_Tx[DataSize] = {1650,1684,1719,1753,1787,1821,1855,1889,1923,1957,1990,2023,2056,2089,2122,2154,2186,
	2217,2249,2279,2310,2340,2370,2399,2428,2456,2484,2511,2538,2564,2590,2615,2640,2664,2687,2710,2732,2754,2774,2795,2814,2833,2851,2868,
	2885,2901,2916,2930,2943,2956,2968,2979,2990,2999,3008,3016,3023,3029,3035,3039,3043,3046,3048,3050,3050,3050,3048,3046,3043,3039,3035,
	3029,3023,3016,3008,2999,2990,2979,2968,2956,2943,2930,2916,2901,2885,2868,2851,2833,2814,2795,2774,2754,2732,2710,2687,2664,2640,2615,
	2590,2564,2538,2511,2484,2456,2428,2399,2370,2340,2310,2279,2249,2217,2186,2154,2122,2089,2056,2023,1990,1957,1923,1889,1855,1821,1787,
	1753,1719,1684,1650,1616,1581,1547,1513,1479,1445,1411,1377,1343,1310,1277,1244,1211,1178,1146,1114,1083,1051,1021,990,960,930,901,872,
	844,816,789,762,736,710,685,660,636,613,590,568,546,526,505,486,467,449,432,415,399,384,370,357,344,332,321,310,301,292,284,277,271,265,
	261,257,254,252,250,250,250,252,254,257,261,265,271,277,284,292,301,310,321,332,344,357,370,384,399,415,432,449,467,486,505,526,546,568,
	590,613,636,660,685,710,736,762,789,816,844,872,901,930,960,990,1021,1051,1083,1114,1146,1178,1211,1244,1277,1310,1343,1377,1411,1445,1479,
1513,1547,1581,1616};

	
//DAC输出初始化
void DAC1_Init(void)
{  
	/* 使能时钟 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能GPIOA时钟 168MHz
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);	//使能DAC时钟 42MHz

	/* DAC端口配置 */
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//PA4
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;	//模拟输入，连接到DAC时同时表示模拟输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;	//内部下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);	//初始化

	/* DAC初始化设置 */
	DAC_InitTypeDef DAC_InitType;
	DAC_InitType.DAC_Trigger=DAC_Trigger_T6_TRGO;	//定时器6触发
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;	//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;	//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Enable;	//DAC1输出缓存
  DAC_Init(DAC_Channel_1,&DAC_InitType);	//初始化DAC1
	
	/* DAC触发频率 */
//	TIM6_Init(100,840);	//触发频率 84MHz/100/840 = 1kHz
//	TIM6_Init(100,420);	//触发频率 84MHz/100/420 = 2kHz
//	TIM6_Init(10,840);	//触发频率 84MHz/10/840 = 10kHz
	TIM6_Init(100,42);	//触发频率 84MHz/100/42 = 20kHz
//	TIM6_Init(2,3281);	//波形频率 84MHz/2/3281/DataSize ≈ 50Hz，输出连续波形时除以DataSize

	/* 配置DMA */
	USER_DMA_Config(DMA1_Stream5,DMA_Channel_7,DMA_DIR_MemoryToPeripheral,(u32)&DAC->DHR12R1,(u32)&DAC_Data_Tx,DataSize);	//DAC1输出固定值
//	USER_DMA_Config(DMA1_Stream5,DMA_Channel_7,DMA_DIR_MemoryToPeripheral,(u32)&DAC->DHR12R1,(u32)DAC1_Data_Tx,DataSize);	//DAC1连续输出正弦波（波形可改）
	
	/* 配置ADC中的DMA */
	DAC_DMACmd(DAC_Channel_1,ENABLE);	//使能DAC1的DMA
	
	/* 使能DMA */
	USER_DMA_Enable(DMA1_Stream5,DataSize);	//使能DMA1的Stream5
	
	/* 使能DAC */
	DAC_Cmd(DAC_Channel_1, ENABLE);	//使能DAC1
}
