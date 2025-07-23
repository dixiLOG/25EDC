#include "i2c.h"
 
// I2C3��ʼ������
void I2C3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	// ��I2C3��GPIOA/GPIOC��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);	//ʹ��I2C3ʱ�� 42MHz
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// ����GPIOA������ΪI2C3����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ����GPIOA���Ÿ��ù���
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
	
	// ����GPIOC������ΪI2C3����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// ����GPIOC���Ÿ��ù���
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);

	// I2C3����
	I2C_InitStructure.I2C_ClockSpeed = 100000;	// ����ʱ���ٶ�Ϊ100kHz
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C3, &I2C_InitStructure);

	// ʹ��I2C3
	I2C_Cmd(I2C3, ENABLE);
}
 
// I2C3�������ݺ���
void I2C3_WriteData(uint8_t address, uint8_t reg, uint8_t data)
{
	// �ȴ�I2C3���ڿ���״̬
	while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

	// ����START�ź�
	I2C_GenerateSTART(I2C3, ENABLE);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

	// ����Ŀ���豸��ַ��дָ��
	I2C_Send7bitAddress(I2C3, address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// ���ͼĴ�����ַ
	I2C_SendData(I2C3, reg);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// ��������
	I2C_SendData(I2C3, data);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// ����STOP�ź�
	I2C_GenerateSTOP(I2C3, ENABLE);
}
 
// I2C3�������ݺ���
uint8_t I2C3_ReadData(uint8_t address, uint8_t reg)
{
	uint8_t data;

	// �ȴ�I2C3���ڿ���״̬
	while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

	// ����START�ź�
	I2C_GenerateSTART(I2C3, ENABLE);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

	// ����Ŀ���豸��ַ��дָ��
	I2C_Send7bitAddress(I2C3, address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// ���ͼĴ�����ַ
	I2C_SendData(I2C3, reg);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// ���·���START�źţ��л�������ģʽ
	I2C_GenerateSTART(I2C3, ENABLE);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

	// ����Ŀ���豸��ַ�Ͷ�ָ��
	I2C_Send7bitAddress(I2C3, address, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	// ����ACK
	I2C_AcknowledgeConfig(I2C3, ENABLE);

	// ��������
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data = I2C_ReceiveData(I2C3);

	// ����STOP�ź�
	I2C_GenerateSTOP(I2C3, ENABLE);

	return data;
}
