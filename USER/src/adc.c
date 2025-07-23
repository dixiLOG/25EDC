/*********************************************************************************
**********************************************************************************
* �ļ�����: adc.c                                                         	 	     *
* �ļ���������ʱ��2����ADC1ͨ��0/1������DMA����									                   *
* �������ڣ�2024.07.07                                                          	 *
* ˵    ���������ʿɵ�																			   						 				 *
**********************************************************************************
*********************************************************************************/

#include "adc.h"

u16 ADC1_Data_Rx[2*Sampl_Times];	//�������ݴ洢, <-- ��������С�ӱ�
u16 ADC3_Data_Rx[ChannelSize*Sampl_Times];	
//��ʼ��ADC
void  ADC1_Init(void)
{
	/* ʹ��ʱ�� */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ�� 168MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ�� 84MHz

  /* ��ʼ��ADC1ͨ��0/1 IO�� */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA0/PA1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����

	/* ����ADC1���� */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	//���������׶�֮����ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; //DMAʹ��
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;	//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
  ADC_CommonInit(&ADC_CommonInitStructure);	//��ʼ��

	ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ɨ��ģʽ
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//������ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;	//TIM2_CH2����ADC1
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;	//�ⲿ�����ش���
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//�Ҷ���
  ADC_InitStructure.ADC_NbrOfConversion = 1;	//ChannelSize��ת���ڹ���������
  ADC_Init(ADC1, &ADC_InitStructure);	//ADC��ʼ��
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);	//Cycles�󣬲�����׼ȷ
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles);
	
	/* ����ADC�е�DMA */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);	//ʹ��ADC��DMA
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ADC
	
	/* ����DMA */
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC, ENABLE);	//ʹ��DMA2��������ж�
	USER_DMA_Config(DMA2_Stream0,DMA_Channel_0,DMA_DIR_PeripheralToMemory,(u32)&ADC1->DR,(u32)ADC1_Data_Rx,2*Sampl_Times);	//����DMA2,// <-- ʹ��˫����С
	USER_DMA_Enable(DMA2_Stream0, 2*Sampl_Times);	//ʹ��DMA2 <-- ʹ��˫����С
	
	/* ���ò����� */
	//��ʱ��2��CH2 PWM������˫ͨ�����600k��
//	TIM2_Init(2,420000);	//������ = 84MHz/2/42000 = 1kHz
//	TIM2_Init(2,21000);	//������ = 84MHz/2/21000 = 2kHz
//	TIM2_Init(2,4200);	//������ = 84MHz/2/4200 = 10kHz
//	TIM2_Init(2,2100);	//������ = 84MHz/2/2100 = 20kHz
//	TIM2_Init(2,820);	//������ = 84MHz/2/820 = 50kHz
//	TIM2_Init(2,420);	//������ = 84MHz/2/420 = 100kHz
//	TIM2_Init(2,410);	//������ = 84MHz/2/410 = 102.44kHz
//	TIM2_Init(2,210);	//������ = 84MHz/2/210 = 200kHz
//	TIM2_Init(2,140);	//������ = 84MHz/2/140 = 300kHz
		//������ = 84MHz/2/70 = 600kHz��21MHz/(����3+12+5+3+12=35ʱ������)=600kHz��

	//��ͨ���ɴ�1.4M
//	TIM2_Init(2,42);	//������ = 84MHz/2/42 = 1MHz
//	TIM2_Init(2,41);	//������ = 84MHz/2/41 = 1.0244MHz
//	TIM2_Init(2,30);	//������ = 84MHz/2/30 = 1.4MHz��21MHz/(����3+12=15ʱ������)=1.4MHz��
}
void ADC3_Init(void){
	/* ʹ��ʱ�� */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ�� 168MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE); //ʹ��ADC3ʱ�� 84MHz

  /* ��ʼ��ADC3ͨ��0/1 IO�� */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//PA0/PA1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,ENABLE);	  //ADC3��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,DISABLE);	//��λ����

	/* ����ADC3���� */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;	//���������׶�֮����ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; //DMAʹ��
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;	//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
  ADC_CommonInit(&ADC_CommonInitStructure);	//��ʼ��

	ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ɨ��ģʽ
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//������ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;	//TIM2_CH2����ADC3
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;	//�ⲿ�����ش���
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//�Ҷ���
  ADC_InitStructure.ADC_NbrOfConversion = ChannelSize;	//ChannelSize��ת���ڹ���������
  ADC_Init(ADC3, &ADC_InitStructure);	//ADC��ʼ��
	
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);	//Cycles�󣬲�����׼ȷ
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
	/* ����ADC�е�DMA */
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);	//ʹ��ADC��DMA
	ADC_Cmd(ADC3, ENABLE);	//ʹ��ADC

	/* ����DMA */
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC, ENABLE);	//ʹ��DMA2��������ж�
	USER_DMA_Config(DMA2_Stream1,DMA_Channel_2,DMA_DIR_PeripheralToMemory,(u32)&ADC3->DR,(u32)ADC3_Data_Rx,ChannelSize*Sampl_Times);	//����DMA2
	USER_DMA_Enable(DMA2_Stream1, ChannelSize*Sampl_Times);	//ʹ��DMA2
	
	/* ���ò����� */
	//��ʱ��2��CH2 PWM������˫ͨ�����600k��
//	TIM2_Init(2,42000);	//������ = 84MHz/2/42000 = 1kHz
//	TIM2_Init(2,21000);	//������ = 84MHz/2/21000 = 2kHz
//	TIM2_Init(2,4200);	//������ = 84MHz/2/4200 = 10kHz
//	TIM2_Init(2,2100);	//������ = 84MHz/2/2100 = 20kHz
//	TIM2_Init(2,820);	//������ = 84MHz/2/820 = 50kHz
//	TIM2_Init(2,420);	//������ = 84MHz/2/420 = 100kHz
//	TIM2_Init(2,410);	//������ = 84MHz/2/410 = 102.44kHz
//	TIM2_Init(2,210);	//������ = 84MHz/2/210 = 200kHz
//	TIM2_Init(2,140);	//������ = 84MHz/2/140 = 300kHz
	;	//������ = 84MHz/2/70 = 600kHz��21MHz/(����3+12+5+3+12=35ʱ������)=600kHz��

	//��ͨ���ɴ�1.4M
//	TIM2_Init(2,42);	//������ = 84MHz/2/42 = 1MHz
//	TIM2_Init(2,41);	//������ = 84MHz/2/41 = 1.0244MHz
//	TIM2_Init(2,30);	//������ = 84MHz/2/3
}

