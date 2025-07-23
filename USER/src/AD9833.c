/****************************************************************************
*****************************************************************************
* 文件名称: ad9833.c															*
* 文件简述：驱动 DDS 芯片 AD9833												*
* 创建日期：2024.07.11														*
* 修改日期：2024.07.21														*
* 说    明：	AD9833_1: PB11(CS)  B13(SCK)  B15(MOSI)							*
			AD9833_2: PB12(CS)  B13(SCK)  B15(MOSI)							*
*****************************************************************************
*****************************************************************************/

#include "ad9833.h"

/****************************************************************************
* 名    称: void AD9833_Init(void)
* 功    能：AD9833 初始化，对应 SPI2，SPI_DataSize_16b/SPI_CPOL_High/SPI_CPHA_2Edge
* 入口参数：无
* 返回参数：无
* 说    明：可通过两个 CS 引脚控制两个 AD9833
			AD9833_1: PB11(CS)  B13(SCK)  B15(MOSI)
			AD9833_2: PB12(CS)  B13(SCK)  B15(MOSI)
****************************************************************************/
void AD9833_Init(void)
{
	SPI2_Init();
}

/****************************************************************************
* 名    称: void AD9833_Send(uint16_t data, uint8_t channel)
* 功    能：设置 AD9833 的寄存器值
* 入口参数：data: 寄存器值; channel: 控制通道
* 返回参数：无
* 说    明：可通过两个 CS 控制两个 AD9833
****************************************************************************/
void AD9833_Send(uint16_t data, uint8_t channel) 
{
	if (channel == 1) {
		SPI2_CS1 = 0;
		SPI2_ReadWriteByte(data);
		SPI2_CS1 = 1;
	}
	else if(channel == 2) {
		SPI2_CS2 = 0;
		SPI2_ReadWriteByte(data);
		SPI2_CS2 = 1;
	}
}

/****************************************************************************
* 名    称: AD9833_WaveOut(uint8_t mode, uint32_t Freq, uint16_t Phase, uint8_t channel)
* 功    能：设置 AD9833 输出的波形、频率、相位（角度制）、通道
* 入口参数：mode: 波形; Freq: 频率; Phase: 相位（角度制）; channel: 通道
* 返回参数：无
* 说    明：可通过两个 CS 控制两个 AD9833
****************************************************************************/
void AD9833_WaveOut(uint8_t mode, uint32_t Freq, uint16_t Phase, uint8_t channel)
{
	AD9833_Send(0x0100, channel);	// 复位AD9833
	AD9833_Send(0x2100, channel);	// 选择数据一次写入
	
	// 频率/相位控制字
	uint32_t Freq_Reg = 0;
	uint16_t Phase_Reg = 0;
	Freq_Reg = Freq / F_mclk * M_mclk;		// 频率转换为写入寄存器的值
	Phase_Reg = Phase / 360.0f * P_mclk;	// 相位转换为写入寄存器的值

	// 频率
	AD9833_Send((0x4000|(Freq_Reg&0x3FFF)), channel);	// 写入频率寄存器0 L14
	AD9833_Send((0x4000|(Freq_Reg>>14)), channel);		// 写入频率寄存器0 H14
	
//	AD9833_Send((0x8000|(Freq_Reg&0x3FFF)), channel);	// 写入频率寄存器1 L14
//	AD9833_Send((0x8000|(Freq_Reg>>14)), channel);		// 写入频率寄存器1 H14
	
	// 相位
	AD9833_Send(0xC000|(Phase_Reg&0x0FFF), channel);	// 写入相位寄存器0
	
//	AD9833_Send(0xE000|(Phase_Reg&0x0FFF), channel);	// 写入相位寄存器1
	
	//寄存器
	uint16_t FP_reg = 0x0000;
	//0x0000	频率寄存器0/相位寄存器0
	//0x0800	频率寄存器1/相位寄存器0
	//0x0400	频率寄存器0/相位寄存器1
	//0x0C00	频率寄存器1/相位寄存器1

	// 波形
	switch(mode) 
	{
		case SQU_WAVE:	// 方波
			AD9833_Send(0x2028|FP_reg, channel); break;
		case TRI_WAVE:	// 三角波
			AD9833_Send(0x2002|FP_reg, channel); break;
		case SIN_WAVE:	// 正弦波
			AD9833_Send(0x2000|FP_reg, channel); break;
		default: break;
	}
}
