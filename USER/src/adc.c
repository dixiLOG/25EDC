/*********************************************************************************
**********************************************************************************
* 文件名称: adc.c                                                         	 	     *
* 文件简述：定时器2触发ADC1通道0/1采样，DMA搬运									                   *
* 创建日期：2024.07.07                                                          	 *
* 说    明：采样率可调																			   						 				 *
**********************************************************************************
*********************************************************************************/

#include "adc.h"

u16 ADC1_Data_Rx[1*Sampl_Times];	//采样数据存储
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
	USER_DMA_Config(DMA2_Stream0,DMA_Channel_0,DMA_DIR_PeripheralToMemory,(u32)&ADC1->DR,(u32)ADC1_Data_Rx,1*Sampl_Times);	//配置DMA2
	USER_DMA_Enable(DMA2_Stream0, 1*Sampl_Times);	//使能DMA2
	
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

void ADC3_Init(void)
{
	/* 1. 使能时钟 (与原配置相同) */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); //使能ADC3时钟

	/* 2. 初始化GPIO (与原配置相同) */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA2 作为ADC3_IN2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;    //模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //不带上下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 3. 配置ADC通用参数 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// **修改点**: DMA模式禁用
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // ADCCLK = 84/4 = 21MHz
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* 4. 配置ADC3参数 */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12位分辨率
	// **修改点**: 扫描模式禁用，因为我们是单通道单点采样
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	// 非连续转换，由外部事件触发
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	// **保留**: 触发源仍然是TIM2_CC2，以保证采样率不变
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐
	// **修改点**: 规则序列中只有1个转换
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC3, &ADC_InitStructure);

	/* 5. 配置规则通道 (与原配置相同) */
	// ADC3, 通道2, 序列1, 采样时间
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);

	/* 6. **核心修改**: 使能ADC中断 */
	// 清除可能存在的EOC标志
	ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
	// 使能转换结束中断(EOC)
	ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);

	/* 7. 配置NVIC (中断控制器) */
	NVIC_InitTypeDef NVIC_InitStructure;
	// 注意: ADC1, ADC2, ADC3共享同一个中断向量 ADC_IRQn
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级 (可根据系统情况调整)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        // 子优先级 (可根据系统情况调整)
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    /* 8. **移除DMA相关配置** */
	// ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);  // 删除
	// ADC_DMACmd(ADC3, ENABLE);                        // 删除
	// USER_DMA_Config(...);                            // 删除对ADC3的DMA配置调用

	/* 9. 使能ADC3 */
	ADC_Cmd(ADC3, ENABLE);
}
