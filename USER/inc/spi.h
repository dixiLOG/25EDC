/*********************************************************************************
**********************************************************************************
* 文件名称: spi.h                                                         	     	 *
* 文件简述：硬件SPI								                                          		 *
* 创建日期：2024.07.11                                                          	 *
* 说    明：spi2硬件cs，spi3软件cs																								 *
**********************************************************************************
*********************************************************************************/

#ifndef __SPI_H
#define __SPI_H
#include "common.h"

//////////////////////////////////////////////////////////////////////////////////	 



// SPI2 CS GPIO
#define SPI2_CS1	PBout(11)
#define SPI2_CS2	PBout(12)

// W25QXX(SPI3_CS)的片选信号
#define	W25QXX_CS PBout(6)	
	
void SPI2_Init(void);			 //初始化SPI2口
//u8 SPI2_ReadWriteByte(u8 writeData);  //SPI2总线读写一个字节
void SPI3_Init(void);
//u8 SPI3_ReadWriteByte(u8 writeData);
		 

uint16_t SPI2_ReadWriteByte(uint16_t writeData);	// SPI2总线读写两个字节
uint8_t SPI3_ReadWriteByte(uint8_t writeData);		// SPI3总线读写一个字节

#endif
