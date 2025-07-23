/*********************************************************************************
**********************************************************************************
* 文件名称: adc.c                                                         	 	     *
* 文件简述：定时器2触发ADC1通道0/1采样，DMA搬运									                   *
* 创建日期：2024.07.07                                                          	 *
* 说    明：采样率可调																			   						 				 *
**********************************************************************************
*********************************************************************************/

#include "adc.h"

u16 ADC1_Data_Rx[2*Sampl_Times];	//采样数据存储, <-- 缓冲区大小加倍
u16 ADC3_Data_Rx[ChannelSize*Sampl_Times];	
//初始化ADC
void  ADC1_Init(void)
{
	/* 使能时钟 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能GPIOA时钟 168MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟 84MHz

  /* 初始化ADC1通道0/1 IO口 */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA0/PA1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束

	/* 配置ADC1采样 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; //DMA使能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;	//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
  ADC_CommonInit(&ADC_CommonInitStructure);	//初始化

	ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	//12位模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//非连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;	//TIM2_CH2触发ADC1
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;	//外部上升沿触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//右对齐
  ADC_InitStructure.ADC_NbrOfConversion = 1;	//ChannelSize个转换在规则序列中
  ADC_Init(ADC1, &ADC_InitStructure);	//ADC初始化
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);	//Cycles大，采样更准确
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);
	
	/* 配置ADC中的DMA */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);	//使能ADC的DMA
	ADC_Cmd(ADC1, ENABLE);	//使能ADC
	
	/* 配置DMA */
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC, ENABLE);	//使能DMA2传输完成中断
	USER_DMA_Config(DMA2_Stream0,DMA_Channel_0,DMA_DIR_PeripheralToMemory,(u32)&ADC1->DR,(u32)ADC1_Data_Rx,2*Sampl_Times);	//配置DMA2,// <-- 使用双倍大小
	USER_DMA_Enable(DMA2_Stream0, 2*Sampl_Times);	//使能DMA2 <-- 使用双倍大小
	
	/* 设置采样率 */
	//定时器2的CH2 PWM触发（双通道最大600k）
//	TIM2_Init(2,420000);	//采样率 = 84MHz/2/42000 = 1kHz
//	TIM2_Init(2,21000);	//采样率 = 84MHz/2/21000 = 2kHz
//	TIM2_Init(2,4200);	//采样率 = 84MHz/2/4200 = 10kHz
//	TIM2_Init(2,2100);	//采样率 = 84MHz/2/2100 = 20kHz
//	TIM2_Init(2,820);	//采样率 = 84MHz/2/820 = 50kHz
//	TIM2_Init(2,420);	//采样率 = 84MHz/2/420 = 100kHz
//	TIM2_Init(2,410);	//采样率 = 84MHz/2/410 = 102.44kHz
//	TIM2_Init(2,210);	//采样率 = 84MHz/2/210 = 200kHz
//	TIM2_Init(2,140);	//采样率 = 84MHz/2/140 = 300kHz
		//采样率 = 84MHz/2/70 = 600kHz（21MHz/(最少3+12+5+3+12=35时钟周期)=600kHz）

	//单通道可达1.4M
//	TIM2_Init(2,42);	//采样率 = 84MHz/2/42 = 1MHz
//	TIM2_Init(2,41);	//采样率 = 84MHz/2/41 = 1.0244MHz
//	TIM2_Init(2,30);	//采样率 = 84MHz/2/30 = 1.4MHz（21MHz/(最少3+12=15时钟周期)=1.4MHz）
}
void ADC3_Init(void){
	/* 使能时钟 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能GPIOA时钟 168MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); //使能ADC3时钟 84MHz

  /* 初始化ADC3通道0/1 IO口 */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//PA0/PA1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,ENABLE);	  //ADC3复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,DISABLE);	//复位结束

	/* 配置ADC3采样 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; //DMA使能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;	//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
  ADC_CommonInit(&ADC_CommonInitStructure);	//初始化

	ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	//12位模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//非连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;	//TIM2_CH2触发ADC3
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;	//外部上升沿触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//右对齐
  ADC_InitStructure.ADC_NbrOfConversion = ChannelSize;	//ChannelSize个转换在规则序列中
  ADC_Init(ADC3, &ADC_InitStructure);	//ADC初始化
	
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);	//Cycles大，采样更准确
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 2, ADC_SampleTime_3Cycles);

	/*
	*            @arg ADC_SampleTime_3Cycles: Sample time equal to 3 cycles
  *            @arg ADC_SampleTime_15Cycles: Sample time equal to 15 cycles
  *            @arg ADC_SampleTime_28Cycles: Sample time equal to 28 cycles
  *            @arg ADC_SampleTime_56Cycles: Sample time equal to 56 cycles	
  *            @arg ADC_SampleTime_84Cycles: Sample time equal to 84 cycles	
  *            @arg ADC_SampleTime_112Cycles: Sample time equal to 112 cycles	
  *            @arg ADC_SampleTime_144Cycles: Sample time equal to 144 cycles	
  *            @arg ADC_SampleTime_480Cycles: Sample time equal to 480 cycles	
	*/
	/* 配置ADC中的DMA */
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);	//使能ADC的DMA
	ADC_Cmd(ADC3, ENABLE);	//使能ADC

	/* 配置DMA */
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC, ENABLE);	//使能DMA2传输完成中断
	USER_DMA_Config(DMA2_Stream1,DMA_Channel_2,DMA_DIR_PeripheralToMemory,(u32)&ADC3->DR,(u32)ADC3_Data_Rx,ChannelSize*Sampl_Times);	//配置DMA2
	USER_DMA_Enable(DMA2_Stream1, ChannelSize*Sampl_Times);	//使能DMA2
	
	/* 设置采样率 */
	//定时器2的CH2 PWM触发（双通道最大600k）
//	TIM2_Init(2,42000);	//采样率 = 84MHz/2/42000 = 1kHz
//	TIM2_Init(2,21000);	//采样率 = 84MHz/2/21000 = 2kHz
//	TIM2_Init(2,4200);	//采样率 = 84MHz/2/4200 = 10kHz
//	TIM2_Init(2,2100);	//采样率 = 84MHz/2/2100 = 20kHz
//	TIM2_Init(2,820);	//采样率 = 84MHz/2/820 = 50kHz
//	TIM2_Init(2,420);	//采样率 = 84MHz/2/420 = 100kHz
//	TIM2_Init(2,410);	//采样率 = 84MHz/2/410 = 102.44kHz
//	TIM2_Init(2,210);	//采样率 = 84MHz/2/210 = 200kHz
//	TIM2_Init(2,140);	//采样率 = 84MHz/2/140 = 300kHz
	;	//采样率 = 84MHz/2/70 = 600kHz（21MHz/(最少3+12+5+3+12=35时钟周期)=600kHz）

	//单通道可达1.4M
//	TIM2_Init(2,42);	//采样率 = 84MHz/2/42 = 1MHz
//	TIM2_Init(2,41);	//采样率 = 84MHz/2/41 = 1.0244MHz
//	TIM2_Init(2,30);	//采样率 = 84MHz/2/3
}

