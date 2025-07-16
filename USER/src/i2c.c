#include "i2c.h"
 
// I2C3初始化函数
void I2C3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	// 打开I2C3和GPIOA/GPIOC的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);	//使能I2C3时钟 42MHz
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// 配置GPIOA的引脚为I2C3功能
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 配置GPIOA引脚复用功能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);
	
	// 配置GPIOC的引脚为I2C3功能
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// 配置GPIOC引脚复用功能
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);

	// I2C3配置
	I2C_InitStructure.I2C_ClockSpeed = 100000;	// 设置时钟速度为100kHz
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C3, &I2C_InitStructure);

	// 使能I2C3
	I2C_Cmd(I2C3, ENABLE);
}
 
// I2C3发送数据函数
void I2C3_WriteData(uint8_t address, uint8_t reg, uint8_t data)
{
	// 等待I2C3处于空闲状态
	while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

	// 发送START信号
	I2C_GenerateSTART(I2C3, ENABLE);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

	// 发送目标设备地址和写指令
	I2C_Send7bitAddress(I2C3, address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// 发送寄存器地址
	I2C_SendData(I2C3, reg);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// 发送数据
	I2C_SendData(I2C3, data);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// 发送STOP信号
	I2C_GenerateSTOP(I2C3, ENABLE);
}
 
// I2C3接收数据函数
uint8_t I2C3_ReadData(uint8_t address, uint8_t reg)
{
	uint8_t data;

	// 等待I2C3处于空闲状态
	while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

	// 发送START信号
	I2C_GenerateSTART(I2C3, ENABLE);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

	// 发送目标设备地址和写指令
	I2C_Send7bitAddress(I2C3, address, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// 发送寄存器地址
	I2C_SendData(I2C3, reg);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// 重新发送START信号，切换到接收模式
	I2C_GenerateSTART(I2C3, ENABLE);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

	// 发送目标设备地址和读指令
	I2C_Send7bitAddress(I2C3, address, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	// 开启ACK
	I2C_AcknowledgeConfig(I2C3, ENABLE);

	// 接收数据
	while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED));
	data = I2C_ReceiveData(I2C3);

	// 发送STOP信号
	I2C_GenerateSTOP(I2C3, ENABLE);

	return data;
}
