/*********************************************************************************
**********************************************************************************
* �ļ�����: uart.c                                                         	     *
* �ļ����������ڲ鿴����+����ADC/DAC/FFT����								                     *
* �������ڣ�2024.05.06                                                           *
**********************************************************************************
*********************************************************************************/

#include "uart.h"

char String[Str_Len];	//��ʽ���ַ���

/****************************************************************************
* ��    ��: void uart1_init(u32 bound)
* ��    �ܣ�USART1��ʼ��
* ��ڲ�����bound��������
* ���ز�������
* ˵    ���� 
****************************************************************************/
void uart1_init(u32 bound)
{ 
	/* ʹ��ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ�� 168MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ�� 84MHz

	/* USART1�˿����� */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;	//GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//����
	GPIO_Init(GPIOA,&GPIO_InitStructure);	//��ʼ��PA9��PA10
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	//GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	//GPIOA10����ΪUSART1
	
  /* USART1��ʼ������ */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure);	//��ʼ������1	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	/* USART1ʹ�� */
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
}

void uart4_init(u32 bound)
{ 
	/* ʹ��ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ�� 168MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��UART4ʱ�� 42MHz

	/* USART4�˿����� */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	//GPIOC10��GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//����
	GPIO_Init(GPIOC,&GPIO_InitStructure);	//��ʼ��PC10��PC11
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);	//GPIOC10����ΪUART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);	//GPIOC11����ΪUART4
	
  /* UART4��ʼ������ */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART4, &USART_InitStructure);	//��ʼ������4
	USART_ClearFlag(UART4, USART_FLAG_TC);
	
	/* UART4ʹ�� */
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���4
}

//����1����һ���ַ�
void uart1SendChar(u8 ch)
{      
	while((USART1->SR&0x40)==0){}	//ѭ������,ֱ���������
    USART1->DR = (u8) ch;
}

//����4����һ���ַ�
void uart4SendChar(u8 ch)
{      
	while((UART4->SR&0x40)==0){}	//ѭ������,ֱ���������
    UART4->DR = (u8) ch;
}

/****************************************************************************
* ��    ��: void uart1SendChars(u8 *str, u8 strlen)
* ��    �ܣ�����1����һ�ַ���
* ��ڲ�����*str�����͵��ַ���
            strlen���ַ�������
* ���ز�������
* ˵    ���� 
****************************************************************************/
void uart1SendChars(u8 *str, u8 strlen)
{ 
	  u8 k= 0 ; 
   do { uart1SendChar(*(str + k)); k++; }
    while (k < strlen); 
}

void uart4SendChars(u8 *str, u8 strlen)
{ 
	  u8 k= 0 ; 
   do { uart4SendChar(*(str + k)); k++; }
    while (k < strlen); 
}

/****************************************************************************
* ��    ��: int fputc(int ch, FILE *f)
* ��    �ܣ��ض�����printf���������
* ��ڲ�����
* ���ز�����
* ˵    ������printf()֮��ĺ�����ʹ���˰�����ģʽ��ʹ�ñ�׼��ᵼ�³����޷�
            ����,�����ǽ������:ʹ��΢��,��Ϊʹ��΢��Ļ�,����ʹ�ð�����ģʽ.
            ���ڹ������Եġ�Target��-����Code Generation���й�ѡ��Use MicroLIB����
            ���Ժ�Ϳ���ʹ��printf��sprintf������
****************************************************************************/
int fputc(int ch, FILE *f)   //�ض�����printf���������  
{
    uart4SendChar(ch);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
    return ch;
}
