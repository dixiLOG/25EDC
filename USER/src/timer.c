/*********************************************************************************
**********************************************************************************
* �ļ�����: timer.c                                                         	     *
* �ļ���������ʱ������ADC��DAC								                       							 *
* �������ڣ�2024.05.06                                                          	 *
* ˵    ����																												   						 *
**********************************************************************************
*********************************************************************************/

#include "timer.h"
#include "pid.h"
#include "lcd.h"
#include "freqmeas.h"
/****************************************************************************
* ��    ��: TIM2_Init(u16 auto_data,u16 fractional)
* ��    �ܣ���ʱ��2��ʼ��
* ��ڲ�����auto_data: �Զ���װֵ
*           fractional: ʱ��Ԥ��Ƶ��
* ���ز�������
* ˵    ������ʱ�����ʱ����㷽��:Tout(us)=auto_data*fractional/Ft		FtΪ��ʱ��ʱ��(MHz)
****************************************************************************/

float kp= 0.00001;
float ki= 0.00002;
float kd=-0.00009;
//
//

void TIM2_Init(u16 auto_data,u16 fractional)
{
	/* ʹ��ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	//ʹ��TIM2ʱ�� 84MHz

	/* TIM2��ʼ������ */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_TimeBaseInitStructure.TIM_Period = auto_data-1;	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=fractional-1;	//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV4;	//�����޸�
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);	//��ʼ��TIM2

	/* PWM��ʼ������ */
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState =	TIM_OutputState_Enable;	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = auto_data/2;	//TIM_Pulse/(TIM_Period+1) ΪPWM��ռ�ձ�
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	//�������:TIM����Ƚϼ��Ե�
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);	//��ʼ������TIM2_CH2
	
	/* ʹ�� */
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);	//ʹ��TIM2Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM2,ENABLE);	//ʹ��ARPE
	TIM_CtrlPWMOutputs(TIM2, ENABLE);	// ʹ��PWM
	TIM_Cmd(TIM2,ENABLE);	//ʹ�ܶ�ʱ��2
}

/****************************************************************************
* ��    ��: TIM6_Init(u16 auto_data,u16 fractional)
* ��    �ܣ���ʱ��6��ʼ��
* ��ڲ�����auto_data: �Զ���װֵ
*           fractional: ʱ��Ԥ��Ƶ��
* ���ز�������
* ˵    ������ʱ�����ʱ����㷽��:Tout(us)=auto_data*fractional/Ft		FtΪ��ʱ��ʱ��(MHz)
****************************************************************************/
void TIM6_Init(u16 auto_data,u16 fractional)
{
	/* ʹ��ʱ�� */
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//ʹ��TIM6ʱ�� 84MHz
	
	/* TIM6��ʼ������ */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_TimeBaseInitStructure.TIM_Period = auto_data-1;	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=fractional-1;	//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;	//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV4;	//�����޸�
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);//����TIME�������Ϊ����ģʽ

	/* ʹ�� */
//	TIM_Cmd(TIM6, ENABLE);
}


void TIM4_Init(u16 period, u16 prescaler)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);		//ʹ��TIM4ʱ��
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;				//TIM4�ж�
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�0��
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);								//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
	TIM_TimeBaseStructure.TIM_Period = period-1;				//�ܵ�ֵ����Ϊ0xFFFF���趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler-1;			//Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;				//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_Cmd(TIM4, ENABLE);										//������ʱ��
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);					//������ʱ�������ж�
}

void kp_up(){
	kp+=0.00001f;
}
void kp_down(){
	kp-=0.00001f;
}
void kd_up(){
	kd+=0.0000005f;
}
void kd_down(){
	kd-=0.0000005f;
}

void TIM4_IRQHandler(void)
{
	//static u16 tmr200ms = 0;
//	static u16 tmr1s =0;
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) 			//����ж�
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);				//����жϱ�־λ
		//��������жϴ���//
	}
}


///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Ƶ�ʼƶ�ʱ������ ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/****************************************************************************
* ��    ��: void TIM7_Init(void)
* ��    �ܣ���ʱ��7 ��ʼ������ΪƵ�ʼƵĻ�׼ʱ��
* ��ڲ�������
* ���ز�������
* ˵    �����벻Ҫ�������
****************************************************************************/
void TIM7_Init(void)	
{
	/* TIM7 Init (Base TIM) --> GATE */
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStrcture;
	
	/* Freq of APB1: 168/4 = 42MHz; TIM7 Freq = 2*42 = 84MHz */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);	
	
	/* 	[Gate] Period of TIM7  Reload = (arr + 1)(psc + 1) / 84M  
	 *	[f standard] After prescaler = 84M / (psc + 1)   
	 *	Error smaller -> Gate*fs larger, Gate*fs = arr + 1
	 *	Let Gate Time = 0.1s
	 *	arr = 50000 - 1
	 *	psc = 168 - 1
	 */
	
	TIM_InitStructure.TIM_Period = 50000 - 1;				// �������� -- 50000
	TIM_InitStructure.TIM_Prescaler = 168 - 1;				// ����Ƶ�� -- 84e6 / 168 = 500 kHz
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// ����Ƶ
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// ���ϼ���
	TIM_InitStructure.TIM_RepetitionCounter = DISABLE;		// ���ظ�
	TIM_TimeBaseInit(TIM7, &TIM_InitStructure);

	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);	// ���ж�
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);	// ���ж�

	NVIC_InitStrcture.NVIC_IRQChannel = TIM7_IRQn;				// �ж�
	NVIC_InitStrcture.NVIC_IRQChannelPreemptionPriority = 0;	// ��ռ���ȼ� 0
	NVIC_InitStrcture.NVIC_IRQChannelSubPriority = 0;			// ��Ӧ���ȼ� 0
	NVIC_InitStrcture.NVIC_IRQChannelCmd = ENABLE;				// ���ж�
	NVIC_Init(&NVIC_InitStrcture);
	
	TIM_Cmd(TIM7, DISABLE);	// ֹͣ����
}

/****************************************************************************
* ��    ��: void TIM1_ETR_Init(void)
* ��    �ܣ����� ��ʱ��1 �� ETR ���ܣ�ʵ������ PA12 ���ⲿ�������
* ��ڲ�������
* ���ز�������
* ˵    �����벻Ҫ�������
****************************************************************************/
void TIM1_ETR_Init(void)
{
	/* TIM1 ETR Init (Advanced TIM)  ETR Count -- > PA12 */
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef NVIC_InitStrcture;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);		// TIM1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// GPIOA

	// GPIOA (PA12) ��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;			// PA12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  		// ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  		// ����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	// 100 MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_TIM1);

	// TIM1 ��ʼ��
	TIM_InitStructure.TIM_Period = 0xFFFF;  				// ��������
	TIM_InitStructure.TIM_Prescaler = 0;   					// ����ϵ��
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// ����Ƶ
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;	// ���ϼ���
	TIM_TimeBaseInit(TIM1, &TIM_InitStructure);
	TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);  // �ⲿʱ��Դģʽ
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);	// ���ж�
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);	// ���ж�
	
	// �ж�����
	NVIC_InitStrcture.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;		// �ж�
	NVIC_InitStrcture.NVIC_IRQChannelPreemptionPriority = 0;	// ��ռ���ȼ� 0 
	NVIC_InitStrcture.NVIC_IRQChannelSubPriority = 1;		  	// ��Ӧ���ȼ� 1
	NVIC_InitStrcture.NVIC_IRQChannelCmd = ENABLE;			  	// ���ж�
	NVIC_Init(&NVIC_InitStrcture);

	TIM_Cmd(TIM1, DISABLE);	// ֹͣ����
}

/****************************************************************************
* ��    ��: void TIM7_IRQHandler(void)
* ��    �ܣ���ʱ��7 �жϷ����������ָʾƵ�ʼ�բ�ź��쳣��ʱ
* ��ڲ�������
* ���ز�������
* ˵    �����벻Ҫ�������
****************************************************************************/
void TIM7_IRQHandler(void)
{
	if(TIM7->SR & (1<<0))	  // TIM_GetITStatus(TIM7,TIM_IT_Update) == SET
	{
		timesTIM7_UpOverLoad ++;							// ��׼�źż����������
		if(timesTIM7_UpOverLoad == 11) overtimeFlag = 0;	// 1.1s �ⲿ���廹û����, �ж�Ϊ��ʱ�쳣
		TIM7->SR &= ~(1<<0);  								// TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

/****************************************************************************
* ��    ��: void TIM1_UP_TIM10_IRQHandler(void)
* ��    �ܣ���ʱ��1 �жϷ����������ָʾ�ⲿ����������
* ��ڲ�������
* ���ز�������
* ˵    �����벻Ҫ�������
****************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM1->SR & (1<<0))
	{
		timesTIM1_UpOverLoad++;		// �ⲿ��������������
		TIM1->SR &= ~(1<<0);  		// ���ж�
	}
}