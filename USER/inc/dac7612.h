#ifndef __DAC7612_H__
#define __DAC7612_H__

//5V供电,12位双DAC
//输出范围为：0-4095mV,对应设置0x0000-0X0FFF,最大0X0FFF(4095mV)
//PC0接SDI
//PC1接CLK
//PC14接CS
//PC15接LOAD

#include "common.h"

#define DAC7612_Port GPIOC
#define DAC7612_SDI_Pin 	GPIO_Pin_0	//PC0接SDI
#define DAC7612_CLK_Pin 	GPIO_Pin_1	//PC1接CLK
#define DAC7612_CS_Pin  	GPIO_Pin_14	//PC14接CS
#define DAC7612_LOAD_Pin	GPIO_Pin_15	//PC15接LOAD

#define DAC7612_SDI_0()  PCout(0)=0
#define DAC7612_SDI_1()  PCout(0)=1

#define DAC7612_CLK_0()  PCout(1)=0
#define DAC7612_CLK_1()  PCout(1)=1

#define DAC7612_CS_0()   PCout(14)=0
#define DAC7612_CS_1()   PCout(14)=1

#define DAC7612_LOAD_0() PCout(15)=0
#define DAC7612_LOAD_1() PCout(15)=1

void DAC7612_Init(void);	//DAC7612初始化
void DAC7612_Write_CHA(uint16_t data);	//设置OUTA,data: 0x0000-0X0FFF,最大0X0FFF
void DAC7612_Write_CHB(uint16_t data);	//设置OUTB,data: 0x0000-0X0FFF,最大0X0FFF
void DAC7612_Write_CHAB(uint16_t data);	//同时设置OUTA/OUTB

#endif
