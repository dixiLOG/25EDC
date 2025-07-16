/*********************************************************************************
**********************************************************************************
* �ļ�����: led.c                                                               	 *
* �ļ�������LED��ʼ��                                                         	   *
* �������ڣ�2024.07.11                                                      			 *
* ˵    ����LED��ӦIO�ڳ�ʼ��                                                 	   *
**********************************************************************************
*********************************************************************************/

#include "led.h" 

//LED��ӦIO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//ʹ��GPIOBʱ��

  //PB7/8/9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;	//LED3��LED2��LED1��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                   //����
  GPIO_Init(GPIOB, &GPIO_InitStructure);                         //��ʼ��GPIO

	GPIO_SetBits(GPIOB, GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
}
