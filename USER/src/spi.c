/****************************************************************
*****************************************************************
* �ļ�����: spi.c												*
* �ļ�����������Ӳ�� SPI MASTER									*
* �������ڣ�2024.07.11											*
* �޸����ڣ�2024.07.21											*
* ˵    ����SPI2 ֻ���� SCK �� MOSI(��������) ���� DDS			*
			SPI3 ���� CS, SCK, MOSI �� MISO, �������� W25QXX		*
*****************************************************************
****************************************************************/

#include "spi.h"

/****************************************************************************
* ��    ��: void SPI2_Init(void)
* ��    �ܣ�SPI2 Ӳ����ʼ��
* ��ڲ�������
* ���ز�������
* ˵    ����SPI2 ��ʼ�����ҽ������ó�����ģʽ B13(SCK) B15(MOSI)
			SPI2 �� CS ����Ϊ PB11��PB12 (�ɿ��� 2 ��SPI�豸)
****************************************************************************/
void SPI2_Init(void)
{	 	
	/* ʹ��ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// ʹ�� GPIOB ʱ�� 168MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);	// ʹ�� SPI2 ʱ��	42MHz
 
	/* SPI2�˿����� */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15; 	// PB13 PB15 ���ù���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			// ���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// ��ʼ��IO��
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);	// PB13����Ϊ SPI2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);	// PB15����Ϊ SPI2
	
	/* SPI2 ��ʼ������ */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);	// ��λSPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);	// ֹͣ��λSPI2
	
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// ����SPI�������˫�������ģʽ: SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						// ����SPI����ģʽ: ����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;					// ����SPI�����ݴ�С: SPI���ͽ���16λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							// ����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						// ����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							// NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������: ���
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	// ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ2
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					// ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							// CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);									// ����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	/* ��ʼ�� Ƭѡ GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12; 	// PB11 PB12 �������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			// ���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// ��ʼ��IO��
	GPIO_SetBits(GPIOB, GPIO_Pin_11|GPIO_Pin_12);			// ����
	
	/* SPI2 ʹ�� */
	SPI_Cmd(SPI2, ENABLE);		// ʹ��SPI����
	SPI2_ReadWriteByte(0xFF);	// ��������
}

/****************************************************************************
* ��    ��: void SPI3_Init(void)
* ��    �ܣ�SPI3 Ӳ����ʼ��
* ��ڲ�������
* ���ز�������
* ˵    ����SPI3 ��ʼ�����ҽ������ó�����ģʽ B3(SCK) B4(MISO) B5(MOSI) B6(���CS)
****************************************************************************/
void SPI3_Init(void)
{	 	
	/* ʹ��ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// ʹ�� GPIOB ʱ�� 168MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);	// ʹ�� SPI3 ʱ��	42MHz
 
	/* SPI3�˿����� */
	// Ӳ�� SPI ����
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5; // PB3 PB4 PB5 ���ù���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					// ���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					// �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					// ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);							// ��ʼ��IO��
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI3);	// PB3����Ϊ SPI3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3);	// PB4����Ϊ SPI3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI3);	// PB5����Ϊ SPI3

	// ��� SPI ����(Ƭѡ CS ���������)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;          	// PB6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      	// ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	// 100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);             	// ��ʼ��

	/* SPI3 ��ʼ������ */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);		// ��λ SPI3
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);	// ֹͣ��λ SPI3
	
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// ����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						// ����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					// ����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							// ����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						// ����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							// NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������: ���
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	// ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ2
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					// ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							// CRCֵ����Ķ���ʽ
	SPI_Init(SPI3, &SPI_InitStructure);									// ����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	/* SPI3 ʹ�� */
	SPI_Cmd(SPI3, ENABLE);		// ʹ��SPI����
	SPI3_ReadWriteByte(0xFF);	// ��������
}

/****************************************************************************
* ��    ��: uint16_t SPI2_ReadWriteByte(uint16_t writeData)
* ��    �ܣ�SPI2 ��д����
* ��ڲ�����writeData:Ҫд����ֽ�
* ���ز�������ȡ�����ֽ�
* ˵    ������ 		     
****************************************************************************/
uint16_t SPI2_ReadWriteByte(uint16_t writeData)
{		 			 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 	// �ȴ���������
	SPI_I2S_SendData(SPI2, writeData);  								// ͨ������ SPI2 ���� 2 ���ֽ�
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 	// �ȴ� 2 ���ֽڽ�����
	return SPI_I2S_ReceiveData(SPI2);  									// ���� SPI2 ���յ�����
}

/****************************************************************************
* ��    ��: uint8_t SPI3_ReadWriteByte(uint8_t writeData)
* ��    �ܣ�SPI3 ��д����
* ��ڲ�����writeData: Ҫд����ֽ�
* ���ز�������ȡ�����ֽ�
* ˵    ������
****************************************************************************/
uint8_t SPI3_ReadWriteByte(uint8_t writeData)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);	 // �ȴ���������
	SPI_I2S_SendData(SPI3, writeData); 								 // ͨ������ SPI3 ����һ���ֽ�
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET); // �ȴ�һ���ֽڽ�����
	return SPI_I2S_ReceiveData(SPI3);  								 // ���� SPI3 ���յ�����			    
}
