/****************************************************************
*****************************************************************
* 文件名称: spi.c												*
* 文件简述：配置硬件 SPI MASTER									*
* 创建日期：2024.07.11											*
* 修改日期：2024.07.21											*
* 说    明：SPI2 只配置 SCK 和 MOSI(主出从入) 驱动 DDS			*
			SPI3 配置 CS, SCK, MOSI 和 MISO, 用于驱动 W25QXX		*
*****************************************************************
****************************************************************/

#include "spi.h"

/****************************************************************************
* 名    称: void SPI2_Init(void)
* 功    能：SPI2 硬件初始化
* 入口参数：无
* 返回参数：无
* 说    明：SPI2 初始化并且将其配置成主机模式 B13(SCK) B15(MOSI)
			SPI2 的 CS 引脚为 PB11、PB12 (可控制 2 个SPI设备)
****************************************************************************/
void SPI2_Init(void)
{	 	
	/* 使能时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// 使能 GPIOB 时钟 168MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);	// 使能 SPI2 时钟	42MHz
 
	/* SPI2端口配置 */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15; 	// PB13 PB15 复用功能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			// 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// 初始化IO口
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);	// PB13复用为 SPI2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);	// PB15复用为 SPI2
	
	/* SPI2 初始化设置 */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);	// 复位SPI2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);	// 停止复位SPI2
	
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// 设置SPI单向或者双向的数据模式: SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						// 设置SPI工作模式: 设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;					// 设置SPI的数据大小: SPI发送接收16位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							// 串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						// 串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							// NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理: 软件
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	// 定义波特率预分频的值:波特率预分频值为2
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					// 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							// CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);									// 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	
	/* 初始化 片选 GPIO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12; 	// PB11 PB12 推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			// 输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);					// 初始化IO口
	GPIO_SetBits(GPIOB, GPIO_Pin_11|GPIO_Pin_12);			// 拉高
	
	/* SPI2 使能 */
	SPI_Cmd(SPI2, ENABLE);		// 使能SPI外设
	SPI2_ReadWriteByte(0xFF);	// 启动传输
}

/****************************************************************************
* 名    称: void SPI3_Init(void)
* 功    能：SPI3 硬件初始化
* 入口参数：无
* 返回参数：无
* 说    明：SPI3 初始化并且将其配置成主机模式 B3(SCK) B4(MISO) B5(MOSI) B6(软件CS)
****************************************************************************/
void SPI3_Init(void)
{	 	
	/* 使能时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// 使能 GPIOB 时钟 168MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);	// 使能 SPI3 时钟	42MHz
 
	/* SPI3端口配置 */
	// 硬件 SPI 部分
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5; // PB3 PB4 PB5 复用功能
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					// 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;					// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					// 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);							// 初始化IO口
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI3);	// PB3复用为 SPI3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3);	// PB4复用为 SPI3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI3);	// PB5复用为 SPI3

	// 软件 SPI 部分(片选 CS 由软件控制)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;          	// PB6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      	// 输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 	// 100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);             	// 初始化

	/* SPI3 初始化设置 */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);		// 复位 SPI3
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);	// 停止复位 SPI3
	
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// 设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						// 设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					// 设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							// 串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						// 串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							// NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理: 软件
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	// 定义波特率预分频的值:波特率预分频值为2
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					// 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							// CRC值计算的多项式
	SPI_Init(SPI3, &SPI_InitStructure);									// 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	
	/* SPI3 使能 */
	SPI_Cmd(SPI3, ENABLE);		// 使能SPI外设
	SPI3_ReadWriteByte(0xFF);	// 启动传输
}

/****************************************************************************
* 名    称: uint16_t SPI2_ReadWriteByte(uint16_t writeData)
* 功    能：SPI2 读写函数
* 入口参数：writeData:要写入的字节
* 返回参数：读取到的字节
* 说    明：无 		     
****************************************************************************/
uint16_t SPI2_ReadWriteByte(uint16_t writeData)
{		 			 
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 	// 等待发送区空
	SPI_I2S_SendData(SPI2, writeData);  								// 通过外设 SPI2 发送 2 个字节
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 	// 等待 2 个字节接收完
	return SPI_I2S_ReceiveData(SPI2);  									// 返回 SPI2 接收的数据
}

/****************************************************************************
* 名    称: uint8_t SPI3_ReadWriteByte(uint8_t writeData)
* 功    能：SPI3 读写函数
* 入口参数：writeData: 要写入的字节
* 返回参数：读取到的字节
* 说    明：无
****************************************************************************/
uint8_t SPI3_ReadWriteByte(uint8_t writeData)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);	 // 等待发送区空
	SPI_I2S_SendData(SPI3, writeData); 								 // 通过外设 SPI3 发送一个字节
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET); // 等待一个字节接收完
	return SPI_I2S_ReceiveData(SPI3);  								 // 返回 SPI3 接收的数据			    
}
