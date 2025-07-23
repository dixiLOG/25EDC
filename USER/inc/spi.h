/*********************************************************************************
**********************************************************************************
* �ļ�����: spi.h                                                         	     	 *
* �ļ�������Ӳ��SPI								                                          		 *
* �������ڣ�2024.07.11                                                          	 *
* ˵    ����spi2Ӳ��cs��spi3���cs																								 *
**********************************************************************************
*********************************************************************************/

#ifndef __SPI_H
#define __SPI_H
#include "common.h"

//////////////////////////////////////////////////////////////////////////////////	 



// SPI2 CS GPIO
#define SPI2_CS1	PBout(11)
#define SPI2_CS2	PBout(12)

// W25QXX(SPI3_CS)��Ƭѡ�ź�
#define	W25QXX_CS PBout(6)	
	
void SPI2_Init(void);			 //��ʼ��SPI2��
//u8 SPI2_ReadWriteByte(u8 writeData);  //SPI2���߶�дһ���ֽ�
void SPI3_Init(void);
//u8 SPI3_ReadWriteByte(u8 writeData);
		 

uint16_t SPI2_ReadWriteByte(uint16_t writeData);	// SPI2���߶�д�����ֽ�
uint8_t SPI3_ReadWriteByte(uint8_t writeData);		// SPI3���߶�дһ���ֽ�

#endif
